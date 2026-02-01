#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def get_teleop_controller(context, *_, **kwargs) -> Node:
    controller = context.launch_configurations["controller"]
    namespace = kwargs["model_ns"]

    if controller == "joystick":
        node = Node(
            package="sjtu_drone_control",
            executable="teleop_joystick",
            namespace=namespace,
            output="screen",
        )

    else:
        node = Node(
            package="sjtu_drone_control",
            executable="teleop",
            namespace=namespace,
            output="screen",
            prefix="xterm -e",
        )

    return [node]


def rviz_node_generator(context, rviz_path):
    """Return a Node action for RViz, omitting --fixed-frame if empty."""
    fixed_frame_value = LaunchConfiguration('fixed_frame').perform(context)

    rviz_arguments = ['-d', rviz_path]

    if fixed_frame_value:
        rviz_arguments.extend(['--fixed-frame', fixed_frame_value])

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_arguments,
            output='screen',
        )
    ]


def generate_launch_description():
    pf_adr_bringup_path = get_package_share_directory('pf_adr')

    rviz_path = os.path.join(
        pf_adr_bringup_path, "rviz", "5_PF_EKF.rviz"
    )

    yaml_file_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', 'drone.yaml'
    )

    model_ns = "drone"

    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"]

    # Número de beacons a lanzar
    num_beacons = 3

    # Lista de nodos de filtros de partículas, uno por beacon
    particle_filter_nodes = []
    for i in range(int(num_beacons)):
        particle_filter_nodes.extend([
            Node(
                package='pf_adr',
                executable='particle_filter_node_ekf',
                name=f'particle_filter_node_{i}',
                parameters=[{
                    'num_particles': 5000,
                    'sigma': 0.1,
                    'noise_std': 0.1,
                    'radius': 1.0,
                    'beacon_id': i  # si el nodo usa este parámetro
                }],
                output='screen',
            ),

            Node(
                package='pf_adr',
                executable='ekf_filter',
                name=f'ekf_filter_{i}',
                parameters=[{
                    'process_noise': 0.001,
                    'measurement_noise': 0.1,
                    'beacon_id': i  # si el nodo usa este parámetro
                }],
                output='screen',
            )
        ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "controller",
            default_value="keyboard",
            description="Type of controller: keyboard (default) or joystick",
        ),

        DeclareLaunchArgument(
            'fixed_frame',
            default_value='',
            description='If provided, sets the fixed frame in RViz.'
        ),

        OpaqueFunction(
            function=rviz_node_generator,
            kwargs={'rviz_path': rviz_path},
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pf_adr_bringup_path, 'launch', 'sjtu_drone_fp.launch.py')
            )
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            namespace=model_ns,
            output='screen',
        ),

        # Añadimos todos los nodos de filtros de partículas aquí
        *particle_filter_nodes,

        OpaqueFunction(
            function=get_teleop_controller,
            kwargs={'model_ns': model_ns},
        ),
    ])
