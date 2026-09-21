from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from fr5v6_weld_planning.transforms import quaternion_from_matrix, rotation_from_rpy


def generate_launch_description():
    share = Path(get_package_share_directory('fr5v6_weld_planning'))
    moveit = Path(get_package_share_directory('fr5v6_moveit_config'))
    config_path = share / 'config/weld_project.yaml'
    config = yaml.safe_load(config_path.read_text())
    pose = config['workpiece']['pose_in_world']
    quaternion = quaternion_from_matrix(rotation_from_rpy(pose['rpy']))
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(moveit / 'launch/gazebo_moveit.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
    )
    bridges = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                   arguments=['/world/empty/create@ros_gz_interfaces/srv/SpawnEntity',
                              '/world/empty/remove@ros_gz_interfaces/srv/DeleteEntity'])
    workpiece_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=['--x', str(pose['xyz'][0]), '--y', str(pose['xyz'][1]),
                   '--z', str(pose['xyz'][2]), '--qx', str(quaternion[0]),
                   '--qy', str(quaternion[1]), '--qz', str(quaternion[2]),
                   '--qw', str(quaternion[3]), '--frame-id', 'world',
                   '--child-frame-id', 'workpiece_frame'])
    runner = Node(
        package='fr5v6_weld_planning', executable='run_weld_demo', output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': str(config_path),
            'output_dir': LaunchConfiguration('output_dir'),
            'execute': ParameterValue(LaunchConfiguration('execute'), value_type=bool),
            'hold_visualization': ParameterValue(
                LaunchConfiguration('hold_visualization'), value_type=bool),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('execute', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('hold_visualization', default_value='true'),
        DeclareLaunchArgument('output_dir',
                              default_value='/home/gsx/FR_Weld_Seam_Planning_ws/output'),
        system, bridges, workpiece_tf, TimerAction(period=15.0, actions=[runner]),
    ])
