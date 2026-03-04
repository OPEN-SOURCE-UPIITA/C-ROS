import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('carro_simulacion')
    
    # FIX: Ruta para que Gazebo encuentre el STL
    resource_path = os.path.join(pkg_share, '..')
    set_gz_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[resource_path])

    manual_mode = LaunchConfiguration('manual_mode')
    manual_mode_arg = DeclareLaunchArgument('manual_mode', default_value='true')

    # Nodos de Simulación
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {os.path.join(pkg_share, "worlds", "pista.world")}'}.items()
    )

    spawn = Node(package='ros_gz_sim', executable='create', arguments=['-name', 'carro', '-file', os.path.join(pkg_share, 'urdf', 'carro.urdf'), '-z', '0.1'])

    # Bridge de Sensores (Nombres corregidos para Jazzy)
    sensors_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Movimiento (Bidireccional)
            '/model/carro/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Tiempo (Gazebo -> ROS)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Transformadas/Posición (Gazebo -> ROS)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Lidar
            '/model/carro/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Cámara RealSense Calibrada
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    return LaunchDescription([set_gz_path, manual_mode_arg, gazebo, spawn, sensors_bridge])