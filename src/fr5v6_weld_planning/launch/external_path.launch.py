from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit = Path(get_package_share_directory('fr5v6_moveit_config'))
    runner = Node(package='fr5v6_weld_planning', executable='external_weld_task_manager', output='screen',
                  parameters=[{'use_sim_time': True, 'job_file': LaunchConfiguration('job_file'),
                               'output_dir': LaunchConfiguration('output_dir'),
                               'execute': ParameterValue(LaunchConfiguration('execute'), value_type=bool),
                               'hold_visualization': ParameterValue(LaunchConfiguration('hold_visualization'), value_type=bool)}])
    def done(event, context):
        if event.returncode != 0 or LaunchConfiguration('hold_visualization').perform(context).lower() == 'false':
            return [EmitEvent(event=Shutdown(reason='External path finished'))]
        return []
    return LaunchDescription([
        DeclareLaunchArgument('job_file'), DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('execute', default_value='false'), DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('hold_visualization', default_value='true'),
        RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=done)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit / 'launch/gazebo_moveit.launch.py')),
                                 launch_arguments={'rviz': LaunchConfiguration('rviz')}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/world/empty/create@ros_gz_interfaces/srv/SpawnEntity', '/world/empty/remove@ros_gz_interfaces/srv/DeleteEntity']),
        TimerAction(period=15., actions=[runner])])
