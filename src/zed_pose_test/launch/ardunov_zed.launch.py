import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. ZED kamerası ile Aracın (Orange Cube / base_link) arasındaki eksen farkını belirten TF (Transform)
    # Parametreler sırasıyla: x, y, z, yaw, pitch, roll
    # Pitch eksenindeki 1.5708 radyan, tam 90 derecelik asagi dönüse esittir.
    # Mesafeleri buraya gireceksiniz (Örn: X=0.2 metre, Z=-0.1 metre vs)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_zed',
        arguments=['0.38', '0.0', '0.0', '0', '1.5708', '0', 'base_link', 'zed_camera_link']
    )

    # 2. Orijinal zed_wrapper launch dosyasini çagirma islemi
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={'camera_model': 'zed2'}.items()
    )

    return LaunchDescription([
        static_tf_node,
        zed_wrapper_launch
    ])
