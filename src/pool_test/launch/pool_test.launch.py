import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    distance_arg = DeclareLaunchArgument('distance', default_value='5.0', description='Target distance to move forward')
    fcu_url_arg = DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:115200', description='MAVROS FCU URL')
    auto_arm_arg = DeclareLaunchArgument('auto_arm', default_value='True', description='Auto arm and GUIDED mode')

    distance = LaunchConfiguration('distance')
    fcu_url = LaunchConfiguration('fcu_url')
    auto_arm = LaunchConfiguration('auto_arm')

    # zed_wrapper launch
    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')
        ]),
        launch_arguments={'camera_model': 'zed2'}.items()
    )

    # mavros launch
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('mavros'), 'launch', 'apm.launch')
        ]),
        launch_arguments={'fcu_url': fcu_url}.items()
    )

    # pool_test node
    pool_test_node = Node(
        package='pool_test',
        executable='pool_test_node',
        name='pool_test_node',
        output='screen',
        parameters=[{
            'distance': distance,
            'auto_arm': auto_arm,
            'speed': 0.5,
            'turn_speed': 0.5,
            'obstacle_threshold': 1.5
        }]
    )

    return LaunchDescription([
        distance_arg,
        fcu_url_arg,
        auto_arm_arg,
        zed_wrapper_launch,
        mavros_launch,
        pool_test_node
    ])
