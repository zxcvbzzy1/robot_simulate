"""Run validated external world TCP paths on the existing Gazebo controller."""
import csv
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from visualization_msgs.msg import Marker, MarkerArray

from .batch import JobFailure, run_batch
from .config import load_config
from .external_path import apply_settings, load_external
from .mesh_loader import load_stl
from .moveit_executor import WeldMoveItExecutor, validate_ready_state
from .run_weld_demo import _bounds, failure_category
from .scene_manager import WorkpieceSynchronizer
from .transforms import matrix_from_quaternion
from .visualization import WeldVisualization, _color, _marker, _point


def markers(prepared):
    result = [Marker(action=Marker.DELETEALL)]
    for item in prepared:
        for segment, color in [('seam', _color(1, .7, 0)), ('approach', _color(0, 1, 1)), ('retreat', _color(1, .3, 0))]:
            m = _marker(item.selected.name + '/' + segment, 0, Marker.LINE_STRIP, 'world')
            m.scale.x = 0.004
            m.color = color
            m.points = [_point(p.position) for p in item.path if p.segment == segment]
            result.append(m)
        for axis, color in enumerate([_color(1, 0, 0), _color(0, 1, 0), _color(0, .3, 1)]):
            m = _marker(item.selected.name + '/tcp_axes', axis, Marker.LINE_LIST, 'world')
            m.scale.x = .001
            m.color = color
            for p in item.path:
                if p.segment == 'seam':
                    m.points += [_point(p.position), _point(p.position + matrix_from_quaternion(p.quaternion)[:, axis] * .015)]
            result.append(m)
    return MarkerArray(markers=result)


def export(output, prepared, metadata):
    fields = ['seam_id', 'index', 'segment', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    with (output / 'world_path.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for item in prepared:
            for index, p in enumerate(item.path):
                values = [item.selected.name, index, p.segment] + p.position.tolist() + p.quaternion.tolist()
                writer.writerow(dict(zip(fields, values)))
    (output / 'import_metadata.json').write_text(json.dumps(metadata, indent=2))


def main():
    rclpy.init()
    node = rclpy.create_node('external_weld_task_manager')
    for name, default in [('job_file', ''), ('output_dir', ''), ('execute', False), ('hold_visualization', True)]:
        node.declare_parameter(name, default)
    output = Path(node.get_parameter('output_dir').value)
    output.mkdir(parents=True, exist_ok=True)
    execute = node.get_parameter('execute').value
    metadata = {}
    exit_code = 0
    try:
        document, prepared, metadata = load_external(node.get_parameter('job_file').value)
        share = Path(get_package_share_directory('fr5v6_weld_planning'))
        config = apply_settings(load_config(share / 'config/weld_project.yaml'), document)
        robot = Path(get_package_share_directory('fr5v6_ros2_control'))
        import xml.etree.ElementTree as ET
        origin = ET.parse(robot / 'urdf/fr5v6.urdf').find("joint[@name='tool0_to_weld_tcp']/origin")
        if origin is None or not np.allclose([float(v) for v in origin.attrib['xyz'].split()], config['tcp']['xyz']):
            raise ValueError('TCP config/URDF mismatch')
        metadata['actual_tcp'] = config['tcp']
        metadata['path_point_count'] = sum(len(p.path) for p in prepared)
        export(output, prepared, metadata)
        if document.get('workpiece'):
            workpiece = document['workpiece']
            mesh_path = (Path(node.get_parameter('job_file').value).parent / workpiece['mesh']).resolve()
            scale = {'m': 1., 'mm': .001}[workpiece['mesh_unit']]
            pose = {'xyz': workpiece['xyz'], 'rpy': np.radians(workpiece['rpy_deg']).tolist()}
            WorkpieceSynchronizer(node, config['timeouts']['startup']).apply(load_stl(mesh_path, scale), mesh_path, pose, [scale] * 3)
        else:
            node.get_logger().warn('SIMULATION_ONLY: hypothetical camera transform; no workpiece collision geometry')
        executor = WeldMoveItExecutor(node, config, _bounds(robot / 'urdf/fr5v6.urdf', config['robot']['joints']))
        executor.ready()
        validate_ready_state(executor.positions, [config['robot']['ready_joint_positions'][j] for j in executor.joints],
                             config['safety']['initial_joint_tolerance'])
        if not executor.state_valid(executor.positions):
            raise ValueError('initial robot state in collision')
        visualization = WeldVisualization(node)
        visualization.publish_markers(markers(prepared))
        initial = executor.robot_state(executor.positions)
        def persist(result):
            (output / 'execution_result.json').write_text(json.dumps(dict(result, **metadata), indent=2))
        result = run_batch(prepared, executor, config, execute, persist,
                           lambda ts: visualization.publish_trajectories(ts, initial))
        node.get_logger().info(f"External path {result['status']}: {metadata['path_point_count']} points")
        if node.get_parameter('hold_visualization').value:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        exit_code = 1
        result = dict(error.result) if isinstance(error, JobFailure) else {}
        result.update(metadata)
        result.update(status='failed', detail=str(error), failure_category=failure_category(error.cause if isinstance(error, JobFailure) else error),
                      execute_requested=bool(execute), followup_motion_suppressed=True)
        (output / 'execution_result.json').write_text(json.dumps(result, indent=2))
        node.get_logger().error(str(error))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
