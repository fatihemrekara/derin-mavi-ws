import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('zed_pose_test'),
        'config'
    )

    return LaunchDescription([
        # 1. Coordinate Conversion Node
        Node(
            package='zed_pose_test',
            executable='gps_to_local_node.py',
            name='gps_to_local_node'
        ),
        
        # 2. Square Route Planner
        Node(
            package='zed_pose_test',
            executable='square_route_planner.py',
            name='square_route_planner'
        ),
        
        # 3. IMU Alignment (ZED Raw to ENU)
        Node(
            package='zed_pose_test',
            executable='imu_alignment_node.py',
            name='imu_alignment_node'
        ),
        
        # 4. ROS 2 EKF (robot_localization)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(config_dir, 'ekf_fusion.yaml')],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered')
            ]
        ),
        
        # 5. Advanced Blind Follower
        Node(
            package='zed_pose_test',
            executable='advanced_blind_follower.py',
            name='advanced_blind_follower',
            output='screen'
        )
    ])
