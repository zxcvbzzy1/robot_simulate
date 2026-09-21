"""FR5 V6 Gazebo simulation with MoveIt, gated on controller readiness."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent, LogInfo
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    share = Path(get_package_share_directory('fr5v6_moveit_config'))
    sim = Path(get_package_share_directory('fr5v6_ros2_control'))
    config = (MoveItConfigsBuilder('fr5v6', package_name='fr5v6_moveit_config')
              .planning_pipelines(
                  pipelines=['ompl', 'chomp', 'stomp', 'pilz_industrial_motion_planner'],
                  default_planning_pipeline='ompl')
              .to_moveit_configs())
    description = (sim / 'urdf/fr5v6.urdf').read_text().replace(
        '$(find fr5v6_ros2_control)/config/robot_params.yaml', str(sim / 'config/robot_params.yaml'))
    config.robot_description = {'robot_description': description}
    params = config.to_dict()
    params.update({'use_sim_time': True, 'publish_robot_description_semantic': True,
                   'publish_planning_scene': True, 'publish_geometry_updates': True,
                   'publish_state_updates': True, 'publish_transforms_updates': True,
                   'allow_trajectory_execution': True, 'moveit_manage_controllers': False})
    # Keep the controller plugin mapped until process exit. Otherwise this installed
    # MoveIt build can unload callback destructors before rclcpp releases weak refs.
    controller_lib = str(Path(get_package_prefix('moveit_simple_controller_manager')) /
                         'lib/libmoveit_simple_controller_manager.so')
    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                      parameters=[params], additional_env={'LD_PRELOAD': controller_lib})
    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                arguments=['-d', str(share / 'config/moveit.rviz')],
                parameters=[config.robot_description, config.robot_description_semantic,
                            config.robot_description_kinematics, config.joint_limits,
                            config.planning_pipelines, {'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('rviz')))
    gate = Node(package='fr5v6_sim_validation', executable='wait_for_sim',
                arguments=['--timeout', '120'], output='screen', parameters=[{'use_sim_time': True}])
    def after_gate(event, context):
        if event.returncode == 0:
            return [LogInfo(msg='FR5 simulation ready; starting MoveIt and RViz.'), move_group, rviz]
        return [EmitEvent(event=Shutdown(reason='FR5 simulation readiness check failed'))]
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=after_gate)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim / 'launch/spawn_fr5v6.launch.py'))), gate])
