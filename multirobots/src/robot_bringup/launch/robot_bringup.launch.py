import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. DIRECTORIOS Y RUTAS ---
    pkg_practica1 = get_package_share_directory('practica1')
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_practica1, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_bringup, 'world', 'world.sdf')
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'config.rviz')

    # ### NUEVO ###: Definimos la ruta al archivo YAML
    bridge_config = os.path.join(pkg_bringup, 'config', 'bridge.yaml')

    # --- 2. NODOS Y LANZAMIENTOS ---

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True
        }]
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'mi_robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.3'],
        output='screen'
    )

    # ### MODIFICADO ###: BRIDGE USANDO YAML
    # Ya no usamos 'arguments=[...]', ahora usamos 'parameters' cargando el fichero
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        rviz
    ])