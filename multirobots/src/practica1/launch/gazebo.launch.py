import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # 1. OBTEBER RUTAS
    pkg_practica1 = get_package_share_directory('practica1')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Archivo Xacro principal
    xacro_file = os.path.join(pkg_practica1, 'urdf', 'robot.urdf.xacro')

    # 2. INICIAR GAZEBO (Mundo vacío)
    # -r: run (arrancar simulación automáticamente)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. PROCESAR XACRO (state_publisher)
    # Ejecutamos el comando 'xacro' para convertir el .xacro a XML puro
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True # Importante para sincronizar con Gazebo
        }]
    )

    # 4. SPAWNEAR (Crear) EL ROBOT
    # Toma la descripción del topic y lo crea en (0,0,0.5)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'mi_robot',
                   '-z', '0.5'], 
        output='screen'
    )

    # 5. BRIDGE (Puente ROS-Gazebo)
    # Necesario para que el reloj de ROS vaya al ritmo de Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn,
        bridge
    ])