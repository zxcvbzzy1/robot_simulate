"""End-to-end STL seam planning and optional Gazebo execution node."""

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy

from .config import ConfigError, load_config
from .mesh_loader import MeshError
from .moveit_executor import (
    TrajectoryValidationError, WeldMoveItExecutor, build_robot_trajectory,
    time_parameterize,
)
from .output_writer import write_outputs
from .scene_manager import SceneSyncError, WorkpieceSynchronizer
from .seam_extractor import SeamExtractionError
from .visualization import WeldVisualization, build_markers
from .workflow import prepare_geometry, process_segment_dts, world_path_poses


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _bounds(urdf_path, joints):
    root = ET.parse(urdf_path).getroot()
    values = {joint.attrib['name']: joint.find('limit') for joint in root.findall('joint')
              if joint.attrib.get('type') == 'revolute'}
    return [(float(values[name].attrib['lower']), float(values[name].attrib['upper']))
            for name in joints]


def _result_path(output):
    output.mkdir(parents=True, exist_ok=True)
    return output / 'execution_result.json'


def _write_result(path, data):
    path.write_text(json.dumps(data, indent=2), encoding='utf-8')


def failure_category(error):
    message = str(error).lower()
    if isinstance(error, ConfigError):
        return 'configuration'
    if isinstance(error, MeshError):
        return 'stl'
    if isinstance(error, SceneSyncError):
        return 'scene_sync'
    if 'non-terminal' in message or 'terminal result' in message:
        return 'terminal_unknown'
    if isinstance(error, SeamExtractionError):
        pose_words = ('normal', 'quaternion', 'tangent', 'orientation', 'zero or cancelling')
        return 'pose' if any(word in message for word in pose_words) else 'candidate'
    if isinstance(error, TrajectoryValidationError):
        if 'ik ' in message or message.startswith('ik'):
            return 'ik'
        if 'limit' in message or 'finite' in message:
            return 'joint_limit'
        if 'jump' in message or 'continu' in message:
            return 'joint_jump'
        if 'collision' in message or 'state validity' in message:
            return 'collision'
        if 'tcp' in message or 'fk ' in message:
            return 'pose'
        return 'trajectory'
    if isinstance(error, (RuntimeError, TimeoutError)):
        return 'execution'
    return 'internal'


def run(node, config, config_path, output, execute):
    share = Path(get_package_share_directory('fr5v6_weld_planning'))
    robot_share = Path(get_package_share_directory('fr5v6_ros2_control'))
    mesh_path = (share / config['workpiece']['mesh']).resolve()
    prepared = prepare_geometry(mesh_path, config)
    result_file = _result_path(output)
    metadata = {
        'schema_version': 1,
        'frame_id': 'workpiece_frame',
        'units': {'length': 'm', 'angle': 'rad'},
        'workpiece_sha256': _sha256(mesh_path),
        'config_sha256': _sha256(config_path),
        'temporary_sim_only_tcp': bool(config['tcp']['TEMPORARY_SIM_ONLY']),
    }
    write_outputs(output, prepared.candidates, prepared.selected, prepared.path, metadata)
    visualization = WeldVisualization(node)
    visualization.publish_markers(build_markers(
        prepared.candidates, prepared.selected, prepared.samples, prepared.path,
        mesh_path.as_uri(), config['workpiece']['pose_in_world']))

    WorkpieceSynchronizer(node, config['timeouts']['startup']).apply(
        prepared.mesh, mesh_path, config['workpiece']['pose_in_world'],
        [config['workpiece']['mesh_scale']] * 3,
    )
    bounds = _bounds(robot_share / 'urdf/fr5v6.urdf', config['robot']['joints'])
    executor = WeldMoveItExecutor(node, config, bounds)
    executor.ready()
    world_poses = world_path_poses(prepared.path, config['workpiece']['pose_in_world'])
    segments = [point.segment for point in prepared.path]
    states = executor.solve_ik(world_poses, segments)
    dense_count = executor.validate_collisions(states)
    fk_metrics = executor.verify_fk(states, world_poses, segments)
    process_dts = process_segment_dts(prepared.path, config['motion']['speed_mps'])
    times = time_parameterize(
        states, config['motion']['max_joint_velocity'], config['motion']['max_joint_acceleration'],
        config['motion']['minimum_dt'], process_dts,
    )
    weld_trajectory = build_robot_trajectory(config['robot']['joints'], states, times)
    approach_trajectory = executor.plan_to(states[0])
    visualization.publish_trajectories(
        [approach_trajectory, weld_trajectory], executor.robot_state(executor.positions))

    result = {
        'started_at': datetime.now(timezone.utc).isoformat(),
        'status': 'planned_only',
        'execute_requested': bool(execute),
        'candidate_count': len(prepared.candidates),
        'selected_candidate': prepared.selected.name,
        'seam_length_m': prepared.selected.length,
        'path_point_count': len(prepared.path),
        'dense_collision_state_count': dense_count,
        **fk_metrics,
    }
    if execute:
        result['approach_execution'] = executor.execute(
            approach_trajectory, states[0], 'move_to_approach')
        result['weld_execution'] = executor.execute(
            weld_trajectory, states[-1], 'weld_path')
        result['weld_execution'].update(executor.final_tcp_errors(world_poses[-1]))
        result['status'] = 'succeeded'
    result['finished_at'] = datetime.now(timezone.utc).isoformat()
    _write_result(result_file, result)
    return result, visualization


def main():
    rclpy.init()
    node = rclpy.create_node('fr5v6_weld_demo')
    node.declare_parameter('config_file', '')
    node.declare_parameter('output_dir', '/home/gsx/FR_Weld_Seam_Planning_ws/output')
    node.declare_parameter('execute', False)
    node.declare_parameter('hold_visualization', True)
    share = Path(get_package_share_directory('fr5v6_weld_planning'))
    configured = node.get_parameter('config_file').value
    config_path = Path(configured) if configured else share / 'config/weld_project.yaml'
    output = Path(node.get_parameter('output_dir').value)
    execute = bool(node.get_parameter('execute').value)
    result_file = _result_path(output)
    exit_code = 0
    try:
        config = load_config(config_path)
        result, _visualization = run(node, config, config_path, output, execute)
        node.get_logger().info(
            f"Weld workflow {result['status']}: {result['path_point_count']} path points")
        if node.get_parameter('hold_visualization').value:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        exit_code = 1
        node.get_logger().error(str(error))
        _write_result(result_file, {
            'status': 'failed',
            'failure_category': failure_category(error),
            'detail': str(error),
            'execute_requested': execute,
            'followup_motion_suppressed': bool(execute) and (
                failure_category(error) in {'execution', 'terminal_unknown'}),
            'finished_at': datetime.now(timezone.utc).isoformat(),
        })
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
