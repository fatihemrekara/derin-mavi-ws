import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('line_perception')
    default_params = os.path.join(pkg_share, 'config', 'perception_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Algılama düğümleri için parametre dosyası',
    )

    yolo_node = Node(
        package='line_perception',
        executable='yolo_segmentation_node',
        name='yolo_segmentation_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    extraction_node = Node(
        package='line_perception',
        executable='line_extraction_node',
        name='line_extraction_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_file_arg, yolo_node, extraction_node])
