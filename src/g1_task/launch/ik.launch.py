#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='Logging level')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('g1_task'), 'launch', 'sim.launch.py'))
    )

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

    task = TimerAction(
        period=10.0,
        actions=[ik_service_node, task_sequencer],
    )

    return LaunchDescription([use_sim_time_arg, log_level_arg, sim, task])
