import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declarar el argumento de modo (Default: Modo 1)
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='1',
        description='Modo 1: Full Sensores | Modo 2: Solo Encoders'
    )
    mode = LaunchConfiguration('mode')

    # 2. Rutas de archivos
    pkg_sim = get_package_share_directory('carro_simulacion')
    world_path = os.path.join(pkg_sim, 'worlds', 'pista.world')

    # 3. Incluir Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 4. Bridge condicional (La "Tabla de Ruteo")
    # Este bridge se lanza SIEMPRE para las TFs y el reloj
    base_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    # Bridge para MODO 1: Lidar + Cámara + cmd_vel
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        condition=launch.conditions.IfCondition(PythonExpression([mode, ' == 1'])),
        arguments=[
            '/model/carro/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/carro/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    # Bridge para MODO 2: Solo Encoders/Odometría (Ajustar según tus tópicos de motor_msgs)
    encoder_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        condition=launch.conditions.IfCondition(PythonExpression([mode, ' == 2'])),
        arguments=[
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen'
    )

    return LaunchDescription([
        mode_arg,
        gz_sim,
        base_bridge,
        sensor_bridge,
        encoder_bridge
    ])