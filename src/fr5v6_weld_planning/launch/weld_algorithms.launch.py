"""Only the two Python algorithm servers; no Gazebo, MoveIt or robot motion."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='fr5v6_weld_planning', executable=name, output='screen')
        for name in ('weld_seam_detector', 'weld_path_generator')
    ])
