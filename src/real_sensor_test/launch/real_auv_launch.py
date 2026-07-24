import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Parametreler
    zed_pitch_angle_arg = DeclareLaunchArgument(
        'zed_pitch_angle',
        default_value='20.0',
        description='ZED kamerasinin asagi dogru egim acisi (Derece cinsinden)'
    )

    # EKF Konfigürasyon dosyasının yolu
    ekf_config_path = os.path.join(
        get_package_share_directory('real_sensor_test'),
        'config',
        'ekf_real.yaml'
    )

    # 1. ZED Kamerasi Icin Statik Transform (TF2) - Pitch Acisi Duzeltmesi
    # Dereceyi radyanyana çevirmemiz gerekiyor. PythonExpression ile launch anında hesaplayacağız.
    pitch_rad_expr = PythonExpression(["math.radians(", LaunchConfiguration('zed_pitch_angle'), ")"])

    zed_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_static_tf_publisher',
        output='screen',
        # Argüman sırası: X, Y, Z, Yaw, Pitch, Roll, frame_id, child_frame_id
        # Aracın önü (X ekseni) kameranın önü ile aynı, sadece Pitch (Y ekseni etrafında) eğik.
        arguments=['0.0', '0.0', '0.0', '0.0', pitch_rad_expr, '0.0', 'base_link', 'zed_camera_link']
    )

    # 2. EKF (Sensor Fuzyon) Node'u
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            ('/odometry/filtered', '/real_auv/odometry/filtered')
        ]
    )

    # 3. Path Follower Node'u (zed_pose_test paketinden cagiriliyor)
    path_follower_node = Node(
        package='zed_pose_test',
        executable='path_follower',
        name='path_follower_node',
        output='screen',
        remappings=[
            # Eger eski kod /odometry/filtered dinliyorsa, bizim yeni topic'e yonlendiriyoruz
            ('/odometry/filtered', '/real_auv/odometry/filtered')
        ]
    )

    return LaunchDescription([
        zed_pitch_angle_arg,
        LogInfo(msg=["ZED Kamera Egim Acisi (Pitch): ", LaunchConfiguration('zed_pitch_angle'), " derece."]),
        zed_tf_node,
        ekf_node,
        path_follower_node
    ])
