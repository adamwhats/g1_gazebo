#!/usr/bin/env python3
"""
Combined launch file for Kobuki simulation with navigation.
This file ensures proper startup order to avoid TF transform issues.

When migrating to G1, replace the kobuki launch files with g1 equivalents.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    kobuki_pkg = get_package_share_directory('kobuki')

    # Declare launch arguments
    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Set to false to run Gazebo headless')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('aws_robomaker_small_house_world'), 'worlds', 'small_house.world'
        ),
        description='Path to the Gazebo world file',
    )

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')

    slam_arg = DeclareLaunchArgument('slam', default_value='False', description='Enable SLAM instead of localization')

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(kobuki_pkg, 'maps', 'aws_house.yaml'),
        description='Path to the map file for navigation',
    )

    rviz_arg = DeclareLaunchArgument('rviz', default_value='True', description='Launch RViz for visualization')

    startup_delay_arg = DeclareLaunchArgument(
        'startup_delay',
        default_value='10.0',
        description='Delay in seconds before starting navigation (allows TF to initialize)',
    )

    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='false',
        description='Enable RGB-D camera (disable for better performance)',
    )

    # Include Gazebo simulation launch file
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(kobuki_pkg, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            'camera': LaunchConfiguration('camera'),
        }.items(),
    )

    # Static transform publisher for map->odom (only in localization mode, not SLAM)
    # This provides an initial identity transform until AMCL takes over
    # In SLAM mode, slam_toolbox publishes this transform dynamically
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=UnlessCondition(
            PythonExpression([LaunchConfiguration('slam'), " == 'True' or ", LaunchConfiguration('slam'), " == 'true'"])
        ),
    )

    # Log message before starting navigation
    nav_starting_msg = LogInfo(msg='Waiting for robot to spawn and TF to initialize before starting navigation...')

    # Include navigation launch file with a delay
    # This delay ensures that:
    # 1. Gazebo is fully loaded
    # 2. Robot is spawned
    # 3. ros_gz_bridge is running
    # 4. TF transforms (odom -> base_footprint) are being published
    navigation = TimerAction(
        period=LaunchConfiguration('startup_delay'),
        actions=[
            LogInfo(msg='Starting navigation stack...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(kobuki_pkg, 'launch', 'navigation_sim.launch.py')),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'slam': LaunchConfiguration('slam'),
                    'map': LaunchConfiguration('map'),
                    'rviz': LaunchConfiguration('rviz'),
                }.items(),
            ),
        ],
    )

    # Create and return launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(gui_arg)
    ld.add_action(world_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_arg)
    ld.add_action(map_file_arg)
    ld.add_action(rviz_arg)
    ld.add_action(startup_delay_arg)
    ld.add_action(camera_arg)

    # Add launch sequences
    ld.add_action(simulation)
    ld.add_action(static_tf_publisher)
    ld.add_action(nav_starting_msg)
    ld.add_action(navigation)

    return ld
