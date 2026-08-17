import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Orijinal zed_wrapper launch dosyasini kendi aci ve mesafe parametrelerimizle cagiriyoruz
    # Bu yontem zed_wrapper'in kendi icindeki TF'leri ezerek kamerayi asagi dondurur.
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={
            'camera_model': 'zed2',
            'cam_pos_x': '0.38',
            'cam_pos_y': '0.0',
            'cam_pos_z': '0.0',
            'cam_roll': '0.0',
            'cam_pitch': '1.5708', # 90 derece asagi
            'cam_yaw': '0.0'
        }.items()
    )

    return LaunchDescription([
        zed_wrapper_launch
    ])
