import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command(
                        [
                            'xacro ',
                            os.path.join(get_package_share_directory('g1_description'), 'urdf', 'g1_gazebo.urdf.xacro'),
                        ]
                    ),
                    value_type=str,
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_state_publisher)

    return ld
