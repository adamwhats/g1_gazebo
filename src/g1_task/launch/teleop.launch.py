#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_arg = DeclareLaunchArgument('device', default_value='/dev/input/event19', description='Gamepad device')
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='Log level for ROS nodes')
    gui_arg = DeclareLaunchArgument('gui', default_value='false', description='Launch Gazebo GUI')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('g1_task'), 'launch', 'sim.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
    )

    spawn_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay controller spawning until sim is ready
    delayed_controller = TimerAction(period=5.0, actions=[spawn_velocity_controller])

    # Delay joystick and teleop until controller is ready
    joy_and_teleop = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='g1_task',
                executable='joy_input',
                name='joy_input',
                output='screen',
                parameters=[{'device': LaunchConfiguration('device'), 'use_sim_time': True}],
                arguments=['--ros-args', '--log-level', ['joy_input:=', LaunchConfiguration('log_level')]],
            ),
            Node(
                package='g1_task',
                executable='teleop',
                name='teleop',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=['--ros-args', '--log-level', ['teleop:=', LaunchConfiguration('log_level')]],
            ),
        ],
    )

    # Launch rqt_image_view to show camera feed
    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/d435/image_raw'],
        output='screen',
    )

    return LaunchDescription(
        [device_arg, log_level_arg, gui_arg, sim, delayed_controller, joy_and_teleop, rqt_image_view]
    )
