import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Encuentra automáticamente la ruta de configuración sin importar dónde esté el workspace
    config_path = os.path.join(
        get_package_share_directory('ascamera'),
        'configurationfiles'
    )
    
    return LaunchDescription([
        Node(
            namespace='ascamera',
            package='ascamera',
            executable='ascamera_node',
            name='ascamera_node',
            output='both',
            parameters=[{
                'confiPath': config_path
            }]
        )
    ])
