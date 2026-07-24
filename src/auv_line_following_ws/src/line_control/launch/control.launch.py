import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('line_control')
    default_params = os.path.join(pkg_share, 'config', 'control_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Kontrol düğümü için parametre dosyası',
    )

    controller_node = Node(
        package='line_control',
        executable='line_follower_controller_node',
        name='line_follower_controller_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_file_arg, controller_node])
