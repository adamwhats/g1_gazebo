#!/usr/bin/env python3
"""Combined launch file: Gazebo + task system with 10s delay."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false', description='Launch RViz')
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='Logging level')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('g1_gazebo'), 'launch', 'g1_gazebo.launch.py')
        )
    )

    task = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('g1_task'), 'launch', 'g1_task.launch.py')
                ),
                launch_arguments={'log_level': LaunchConfiguration('log_level')}.items()
            )
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('g1_task'), 'config', 'g1_ik_debug.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([use_rviz_arg, log_level_arg, gazebo, task, rviz])
