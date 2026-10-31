"""One planning-only ROS job. Runs in the container with system Python."""
import hashlib
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from uuid import uuid4
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point
from moveit_msgs.msg import PlanningScene, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle

from .settings import WORKSPACE, legacy_modules
from .trajectory import load_trajectory, pose_matrix, matrix_pose, atomic_json

legacy_modules()
from fr5v6_weld_planning.moveit_executor import (WeldMoveItExecutor, densify_joint_path,
                                               time_parameterize, validate_joint_path)
from fr5v6_weld_planning.config import load_config
from fr5v6_weld_planning.mesh_loader import load_stl

CONFIG = WORKSPACE / "src/fr5v6_weld_planning/config/weld_project.yaml"
URDF = WORKSPACE / "src/fr5v6_ros2_control/urdf/fr5v6.urdf"


def model_config():
    config = load_config(CONFIG)
    root = ET.parse(URDF).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    bounds = [(float(joints[name].find("limit").get("lower")), float(joints[name].find("limit").get("upper"))) for name in config["robot"]["joints"]]
    return config, bounds


class Planner:
    def __init__(self):
        self.node = Node("weld_api_planner_" + uuid4().hex[:8])
        self.config, self.bounds = model_config()
        self.executor = WeldMoveItExecutor(self.node, self.config, self.bounds)
        self.scene = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_scene = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        self.initial = [self.config["robot"]["ready_joint_positions"][j] for j in self.executor.joints]

    def ready(self, timeout=8):
        clients = [self.executor.ik_client, self.executor.fk_client, self.executor.validity_client,
                   self.executor.plan_client, self.scene, self.get_scene]
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(c.service_is_ready() for c in clients):
                return
            rclpy.spin_once(self.node, timeout_sec=.05)
        raise TimeoutError("MoveIt planning services unavailable")

    def call(self, client, request):
        return self.executor._wait_future(client.call_async(request), 10, client.srv_name)

    def set_scene(self, workpiece):
        if workpiece is None:
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
            response = self.call(self.get_scene, request)
            if not any(item.id == "weld_api_workpiece" for item in response.scene.world.collision_objects):
                return
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        item = CollisionObject()
        item.id = "weld_api_workpiece"
        item.header.frame_id = "base_link"
        item.operation = CollisionObject.REMOVE
        if workpiece:
            mesh = load_stl(workpiece["stl_path"], .001 if workpiece["unit"] == "mm" else 1)
            shape = Mesh()
            shape.vertices = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in mesh.vertices]
            shape.triangles = [MeshTriangle(vertex_indices=list(map(int, row))) for row in mesh.triangles]
            item.meshes = [shape]
            transform = pose_matrix(workpiece["pose_in_base"])
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, transform[:3, 3])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, Rotation.from_matrix(transform[:3, :3]).as_quat())
            item.mesh_poses = [pose]
            item.operation = CollisionObject.ADD
        scene.world.collision_objects = [item]
        response = self.call(self.scene, ApplyPlanningScene.Request(scene=scene))
        if not response.success:
            raise RuntimeError("Planning scene update rejected")

    def fk(self, q, tcp):
        from moveit_msgs.srv import GetPositionFK
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = ["tool0"]
        request.robot_state = self.executor.robot_state(q)
        response = self.call(self.executor.fk_client, request)
        if response.error_code.val != 1 or not response.pose_stamped:
            raise RuntimeError("MoveIt FK failed")
        pose = response.pose_stamped[0].pose
        result = np.eye(4)
        result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        result[:3, :3] = Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_matrix()
        return result @ tcp

    def solve(self, target, seed, tcp):
        from moveit_msgs.srv import GetPositionIK
        from builtin_interfaces.msg import Duration
        flange = target @ np.linalg.inv(tcp)
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.executor.group
        ik.ik_link_name = "tool0"
        ik.robot_state = self.executor.robot_state(seed)
        ik.avoid_collisions = False
        ik.timeout = Duration(sec=0, nanosec=200000000)
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = "base_link"
        pose = ik.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, flange[:3, 3])
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, Rotation.from_matrix(flange[:3, :3]).as_quat())
        response = self.call(self.executor.ik_client, request)
        if response.error_code.val != 1:
            return None, {"reason": f"IK failed: MoveIt code {response.error_code.val}", "failure_kind": "ik"}
        values = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        q = [float(values[name]) for name in self.executor.joints]
        if not np.isfinite(q).all() or any(v < low or v > high for v, (low, high) in zip(q, self.bounds)):
            return None, {"reason": "IK violates joint limits", "failure_kind": "joint_limit"}
        actual = self.fk(q, tcp)
        error_mm = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]) * 1000)
        angle_deg = float(np.rad2deg(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude()))
        diagnostic = {"position_error_mm": error_mm, "angle_error_deg": angle_deg}
        if error_mm > 1 or angle_deg > 1:
            return None, {**diagnostic, "reason": "FK error exceeds 1 mm / 1 degree", "failure_kind": "fk"}
        if not self.executor.state_valid(q):
            return None, {**diagnostic, "reason": "State is in collision", "failure_kind": "collision"}
        return q, diagnostic

    def plan(self, arguments):
        source, raw, original, seams = load_trajectory(arguments["trajectory_path"], arguments.get("seam_id"))
        offset = arguments.get("tcp_offset_pose")
        calibrated = offset is not None
        offset = [0, 0, 200, 0, 0, 0] if offset is None else offset
        tcp = pose_matrix(offset)
        speed = arguments.get("speed_mm_s", 20)
        workpiece = arguments.get("workpiece")
        self.set_scene(workpiece)
        reachable = failures = discontinuities = 0
        transfers = []
        previous_end = self.initial
        for seam in seams:
            seed = list(previous_end)
            segment = None
            seam["segments"] = []
            for index, point in enumerate(seam["points"]):
                q, diagnostic = self.solve(pose_matrix(point["pose"]), seed, tcp)
                point.update(diagnostic)
                point.update(joints=q, time_s=0., minimum_duration_s=0., status="unreachable" if q is None else "reachable")
                if q is None:
                    failures += 1
                    segment = None
                    continue
                reachable += 1
                if segment is not None:
                    reason = None
                    if np.max(np.abs(np.asarray(q) - seed)) > self.config["safety"]["max_joint_jump"]:
                        reason = "Joint jump exceeds configured limit"
                    else:
                        dense = densify_joint_path([seed, q], self.config["safety"]["densify_max_joint_step"])
                        if not all(self.executor.state_valid(row) for row in dense):
                            reason = "Interpolated segment is in collision"
                    if reason:
                        point["reason"] = reason
                        discontinuities += 1
                        segment = None
                if segment is None:
                    segment = {"start": index, "end": index, "duration_s": 0.}
                    seam["segments"].append(segment)
                else:
                    previous = seam["points"][index - 1]
                    distance = float(np.linalg.norm(np.asarray(point["pose"][:3]) - previous["pose"][:3]))
                    minimum = time_parameterize([seed, q], self.config["motion"]["max_joint_velocity"],
                                               self.config["motion"]["max_joint_acceleration"], minimum_dt=.02)[-1]
                    duration = max(distance / speed, minimum)
                    point["minimum_duration_s"] = minimum
                    point["time_s"] = previous["time_s"] + duration
                    segment.update(end=index, duration_s=point["time_s"])
                seed = q
            if not seam["points"]:
                seam["reason"] = "缺少 TCP 姿态，仅显示焊缝几何"
            valid_points = [p for p in seam["points"] if p["joints"] is not None]
            if valid_points:
                # Transfer is separately reviewed; it never becomes an unchecked playback edge.
                try:
                    target = valid_points[0]["joints"]
                    if np.max(np.abs(np.asarray(target) - previous_end)) <= 1e-6:
                        transfer_points = [{"joints": target, "time_s": 0.}]
                    else:
                        trajectory = self.executor.plan_to(target, start=previous_end)
                        names = trajectory.joint_trajectory.joint_names
                        order = [names.index(n) for n in self.executor.joints]
                        transfer_points = [{"joints": [p.positions[i] for i in order],
                                            "time_s": p.time_from_start.sec + p.time_from_start.nanosec / 1e9}
                                           for p in trajectory.joint_trajectory.points]
                    transfers.append({"to_seam_id": seam["id"], "status": "planned", "joint_names": self.executor.joints,
                                      "points": transfer_points})
                except Exception as exc:
                    transfers.append({"to_seam_id": seam["id"], "status": "failed", "reason": str(exc)})
                previous_end = valid_points[-1]["joints"]
        summary = {"seam_count": len(seams), "point_count": sum(len(s["points"]) for s in seams),
                   "reachable_count": reachable, "unreachable_count": failures, "discontinuity_count": discontinuities,
                   "geometry_only_count": sum(not s["points"] for s in seams)}
        return {"schema_version": 1, "simulation_id": uuid4().hex, "run_id": arguments["run_id"],
                "created_at": time.time(), "robot": "fairino5_v6", "joint_names": self.executor.joints,
                "joint_velocity_limits_rad_s": self.config["motion"]["max_joint_velocity"], "joint_units": "radian",
                "joint_acceleration_limits_rad_s2": self.config["motion"]["max_joint_acceleration"],
                "frame": "robot_base", "units": {"position": "mm", "angle": "degree"},
                "tcp_offset_pose": offset, "tcp_calibrated": calibrated, "TEMPORARY_SIM_ONLY": not calibrated,
                "speed_mm_s": speed, "source_path": arguments.get("source_path", str(source)),
                "source_sha256": arguments.get("source_sha256", hashlib.sha256(raw).hexdigest()),
                "input_snapshot": original, "summary": summary, "seams": seams, "transfers": transfers,
                "planning": {"backend": "MoveIt2", "model_sha256": hashlib.sha256(URDF.read_bytes()).hexdigest(),
                             "collision_scope": "robot_and_workpiece" if workpiece else "robot_only",
                             "workpiece": workpiece,
                             "torch_collision_geometry": False, "executed": False,
                             "initial_joints": self.initial}}


def main():
    rclpy.init()
    planner = Planner()
    try:
        planner.ready()
        if sys.argv[1] == "--probe":
            print(json.dumps({"status": "ready", "backend": "MoveIt2", "joints": planner.executor.joints}))
        else:
            request = json.loads(Path(sys.argv[1]).read_text())
            try:
                result = planner.plan(request["arguments"])
                atomic_json(sys.argv[2], result)
            finally:
                planner.set_scene(None)
    finally:
        planner.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
