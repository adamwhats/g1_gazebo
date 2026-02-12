import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace, SetParameter


def modify_yaml_with_namespace(original_yaml_path, namespace):
    """Replace <robot_namespace> in the yaml with the actual namespace.
    Creates a temp file with the result and returns its path.
    """
    with tempfile.NamedTemporaryFile(delete=False, mode='w', encoding='utf-8') as temp_file:
        temp_yaml_path = temp_file.name
        with open(original_yaml_path, 'r') as yaml_file:
            namespace_prefix = f'/{namespace}' if namespace != '' else ''
            for line in yaml_file:
                if '<robot_namespace>' in line:
                    line = line.replace('<robot_namespace>', namespace_prefix)
                temp_file.write(line)
    return temp_yaml_path


def start_bridge(context):
    g1_gazebo_pkg = get_package_share_directory('g1_gazebo')
    namespace = LaunchConfiguration('namespace').perform(context)

    original_yaml_path = os.path.join(g1_gazebo_pkg, 'config', 'bridge', 'g1_bridge.yaml')
    modified_yaml_path = modify_yaml_with_namespace(original_yaml_path, namespace)

    bridge = GroupAction(
        [
            SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
            PushROSNamespace(namespace=LaunchConfiguration('namespace')),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge_ros_gz',
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'config_file': modified_yaml_path,
                        'expand_gz_topic_names': True,
                    }
                ],
                output='screen',
            ),
        ]
    )
    return [bridge]


def generate_launch_description():

    namespace_arg = DeclareLaunchArgument('namespace', default_value='')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('aws_robomaker_small_house_world'),
            'worlds',
            'small_house.world',
        ),
        description='Path to Gazebo world file',
    )
    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Launch Gazebo GUI')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    model_path = os.path.join(get_package_share_directory('kobuki'), 'models')
    if 'GZ_SIM_MODEL_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GZ_SIM_MODEL_PATH']

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s ', LaunchConfiguration('world')]}.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [' -g ']}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    g1_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('g1_description'), 'launch', 'g1_description.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    spawn_g1 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'g1', '-topic', 'robot_description', '-x', '5.5', '-y', '1.0', '-z', '0'],
    )

    # Spawn simple box object
    box_sdf = '''<?xml version="1.0" ?>
<sdf version="1.7">
    <model name="simple_box">
        <static>false</static>
        <link name="link">
            <inertial>
                <mass>0.2</mass>
                <inertia>
                    <ixx>0.0005</ixx>
                    <iyy>0.0009</iyy>
                    <izz>0.0007</izz>
                </inertia>
            </inertial>
            <collision name="collision">
                <geometry>
                    <box>
                        <size>0.1 0.1 0.15</size>
                    </box>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>0.9</mu>
                            <mu2>0.9</mu2>
                        </ode>
                    </friction>
                    <contact>
                        <ode>
                            <kp>1e6</kp>
                            <kd>100</kd>
                        </ode>
                    </contact>
                </surface>
            </collision>
            <visual name="visual">
                <geometry>
                    <box>
                        <size>0.1 0.1 0.15</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0.8 0.2 0.2 1</ambient>
                    <diffuse>0.8 0.2 0.2 1</diffuse>
                </material>
            </visual>
        </link>
    </model>
</sdf>'''

    box_sdf_file = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False)
    box_sdf_file.write(box_sdf)
    box_sdf_file.close()

    spawn_box = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name',
            'simple_box',
            '-file',
            box_sdf_file.name,
            '-x',
            '5.85',
            '-y',
            '1.0',
            '-z',
            '0.885',
        ],
    )

    # Controller spawners with a delay
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delayed_controllers = TimerAction(
        period=5.0,
        actions=[spawn_joint_state_broadcaster, spawn_arm_controller],
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path))
    ld.add_action(namespace_arg)
    ld.add_action(world_arg)
    ld.add_action(gui_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(g1_description)
    ld.add_action(spawn_g1)
    ld.add_action(spawn_box)
    ld.add_action(OpaqueFunction(function=start_bridge))
    ld.add_action(delayed_controllers)

    return ld
