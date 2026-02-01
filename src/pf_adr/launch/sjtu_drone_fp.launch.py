#!/usr/bin/env python3
import os
import random
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_gui = DeclareLaunchArgument("use_gui", default_value="true", choices=["true", "false"],
                                    description="Whether to execute gzclient")

    xacro_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", "sjtu_drone.urdf.xacro"
    )

    yaml_file_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', 'drone.yaml'
    )   

    robot_description_config = xacro.process_file(xacro_file, mappings={"params_path": yaml_file_path})
    robot_desc = robot_description_config.toxml()

    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"]

    world_file_default = os.path.join(
        get_package_share_directory('pf_adr'),
        'worlds',
        'empty.world'
    )

    world_file = LaunchConfiguration('world', default=world_file_default)

    world = DeclareLaunchArgument(
        name='world',
        default_value=world_file_default,
        description='Full path to world file to load'
    )

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    def launch_gzclient(context, *args, **kwargs):
        if context.launch_configurations.get('use_gui') == 'true':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                ),
                launch_arguments={'verbose': 'true'}.items()
            )]
        return []

    # Generar balizas aleatorias
    spawn_beacons_actions = []
    num_beacons = 3  # Número de balizas a generar
   
    model_path = os.path.join(
        get_package_share_directory('pf_adr'),
        'models',
        'beacon.sdf'
    )
    # Generar posiciones aleatorias para las balizas
    for i in range(num_beacons):
        if random.random() < 0.5:
            x = random.uniform(-10.0, -4.0)
        else:
            x = random.uniform(4.0, 10.0)

        if random.random() < 0.5:
            y = random.uniform(-10.0, -4.0)
        else:
            y = random.uniform(4.0, 10.0)

        z = random.uniform(0.5, 3.0)

        spawn_beacons_actions.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_beacon_{i}',
                arguments=[
                    '-entity', f'beacon_{i}',
                    '-file', model_path,
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z)
                ],
                output='screen'
            )
        )
        spawn_beacons_actions.append(
            Node(
                package='pf_adr',
                executable='beacon_node',
                name=f'beacon_node_{i}',
                parameters=[{
                    'target_position': [x, y, z],
                    'beacon_id': i,
                }],
                output='screen'
            )
        )

    return LaunchDescription([
        world,
        use_gui,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': world_file,
                'verbose': "true",
                'extra_gazebo_args': 'verbose'
            }.items()
        ),

        OpaqueFunction(function=launch_gzclient),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=model_ns,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_desc,
                "frame_prefix": model_ns + "/"
            }]
        ),

        Node(
            package="sjtu_drone_bringup",
            executable="spawn_drone",
            arguments=[robot_desc, model_ns],
            output="screen"
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{model_ns}/odom"],
            output="screen"
        ),

        # Agregar spawn de las balizas
        *spawn_beacons_actions
    ])
