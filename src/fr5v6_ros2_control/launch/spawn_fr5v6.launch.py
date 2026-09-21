#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)

from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # ---------------------------------------------------------
    # 1. Package paths
    # ---------------------------------------------------------

    package_name = 'fr5v6_ros2_control'

    package_share = get_package_share_directory(package_name)
    package_share_parent = os.path.dirname(package_share)

    current_gz_resource_path = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        ''
    )

    gz_resource_path = (
        package_share_parent
        + os.pathsep
        + current_gz_resource_path
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path,
    )

    urdf_file = os.path.join(
        package_share,
        'urdf',
        'fr5v6.urdf'
    )


    # ---------------------------------------------------------
    # 2. Read FR5 URDF
    # ---------------------------------------------------------

    controller_yaml = os.path.join(
        package_share,
        'config',
        'robot_params.yaml'
    )

    if not os.path.exists(controller_yaml):
        raise FileNotFoundError(
            f'Controller YAML not found: {controller_yaml}'
        )

    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()

    # Replace ROS-style $(find ...) expression with
    # the real absolute path because this file is plain URDF,
    # not processed by xacro.
    robot_description_content = robot_description_content.replace(
        '$(find fr5v6_ros2_control)/config/robot_params.yaml',
        controller_yaml
    )

    robot_description = {
        'robot_description': robot_description_content,
        'use_sim_time': True,
    }



    # ---------------------------------------------------------
    # 3. Start Gazebo Harmonic
    # ---------------------------------------------------------

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),

        launch_arguments={
            'gz_args': '-r -v 3 empty.sdf --render-engine ogre',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ---------------------------------------------------------
    # 4. Robot State Publisher
    # ---------------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ---------------------------------------------------------
    # 5. Spawn FR5 into Gazebo
    # ---------------------------------------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_fr5v6',
        output='screen',

        arguments=[
            '-topic', 'robot_description',
            '-name', 'fr5v6',
            '-allow_renaming', 'true',
        ],
    )

    # ---------------------------------------------------------
    # 6. Joint State Broadcaster
    # ---------------------------------------------------------

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
        output='screen',
    )

    # ---------------------------------------------------------
    # 7. Joint Trajectory Controller
    # ---------------------------------------------------------

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
        output='screen',
    )

    # ---------------------------------------------------------
    # 8. Gazebo /clock -> ROS 2
    # ---------------------------------------------------------

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen',
    )

    # ---------------------------------------------------------
    # 9. Controller startup sequence
    #
    # FR5 spawned
    #     ↓
    # joint_state_broadcaster
    #     ↓
    # joint_trajectory_controller
    # ---------------------------------------------------------

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner
            ],
        )
    )

    start_joint_trajectory_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                joint_trajectory_controller_spawner
            ],
        )
    )

    # ---------------------------------------------------------
    # 10. Launch
    # ---------------------------------------------------------

    return LaunchDescription([

        set_gz_resource_path,

        gazebo,

        robot_state_publisher,

        clock_bridge,

        start_joint_state_broadcaster,

        start_joint_trajectory_controller,

        spawn_robot,

    ])
