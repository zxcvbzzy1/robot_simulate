"""Launch MoveIt and a fixed planning state without simulator or controllers."""
import xml.etree.ElementTree as ET
import yaml
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from .settings import WORKSPACE


def description():
    control = WORKSPACE / "src/fr5v6_ros2_control"
    config_dir = WORKSPACE / "src/fr5v6_moveit_config/config"
    tree = ET.parse(control / "urdf/fr5v6.urdf")
    root = tree.getroot()
    for element in list(root):
        if element.tag in {"gazebo", "ros2_control"}:
            root.remove(element)
    for mesh in root.iter("mesh"):
        value = mesh.get("filename", "")
        mesh.set("filename", value.replace("package://fr5v6_ros2_control/", "file://" + str(control) + "/"))
    urdf = ET.tostring(root, encoding="unicode")
    common = {"robot_description": urdf,
              "robot_description_semantic": (config_dir / "fr5v6.srdf").read_text(),
              "robot_description_kinematics": yaml.safe_load((config_dir / "kinematics.yaml").read_text()),
              "robot_description_planning": yaml.safe_load((config_dir / "joint_limits.yaml").read_text())}
    planning = {**common, "planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl",
                "ompl": yaml.safe_load((config_dir / "ompl_planning.yaml").read_text()),
                "allow_trajectory_execution": False, "use_sim_time": False,
                "disable_capabilities": "move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction",
                "publish_robot_description": True, "publish_robot_description_semantic": True,
                "publish_planning_scene": True, "publish_geometry_updates": True,
                "publish_state_updates": True, "publish_transforms_updates": True}
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": urdf}]),
        Node(package="moveit_ros_move_group", executable="move_group", parameters=[planning], output="screen"),
        ExecuteProcess(
            cmd=["/usr/bin/python3", "-m", "weld_service.planning_state"], output="screen"),
    ])


if __name__ == "__main__":
    launch = LaunchService()
    launch.include_launch_description(description())
    raise SystemExit(launch.run())
