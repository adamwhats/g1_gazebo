#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz')
    gui_arg = DeclareLaunchArgument('gui', default_value='false', description='Launch Gazebo GUI')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('g1_gazebo'), 'launch', 'g1_gazebo.launch.py')
        ),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('g1_task'), 'config', 'g1_ik_debug.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([use_rviz_arg, gui_arg, gazebo, rviz])
