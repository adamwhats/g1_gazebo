"""Launch IK service and task sequencer nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='Logging level')

    config_file = PathJoinSubstitution([FindPackageShare('g1_task'), 'config', 'ik_params.yaml'])

    ik_service_node = Node(
        package='g1_task',
        executable='ik_service',
        name='g1_ik_service',
        output='screen',
        parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['--ros-args', '--log-level', ['g1_ik_service:=', LaunchConfiguration('log_level')]],
    )

    task_sequencer = Node(
        package='g1_task',
        executable='task_sequencer',
        name='g1_task_sequencer',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([use_sim_time_arg, log_level_arg, ik_service_node, task_sequencer])
