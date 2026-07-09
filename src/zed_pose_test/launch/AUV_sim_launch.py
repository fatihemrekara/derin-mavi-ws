import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_name = 'zed_pose_test'

    # 1. GPS to Local Node
    gps_node = Node(
        package=package_name,
        executable='gps_to_local_node', # setup.py'deki console_scripts adı
        name='gps_to_local',
        output='screen'
    )

    # 2. Route Planner Node
    route_node = Node(
        package=package_name,
        executable='route_planner',     # setup.py'deki console_scripts adı
        name='route_planner',
        output='screen'
    )

    # 3. Path Follower Node
    follower_node = Node(
        package=package_name,
        executable='path_follower', # setup.py'deki console_scripts adı
        name='path_follower',
        output='screen',
        parameters=[{'active_window_size': 5}] # İstersen parametreni buradan da verebilirsin
    )

    # 4. Fake Robot Sim Node
    sim_node = Node(
        package=package_name,
        executable='fake_robot_sim',    # setup.py'deki console_scripts adı
        name='fake_robot_sim',
        output='screen'
    )

    # 5. RViz2 (Harici bir araç olduğu için ExecuteProcess ile çağırıyoruz)
    rviz_cmd = ExecuteProcess(
        cmd=['rviz2'],
        output='screen'
    )

    # Tüm süreçleri ROS 2 Launch sistemine teslim ediyoruz
    return LaunchDescription([
        gps_node,
        route_node,
        follower_node,
        sim_node,
        rviz_cmd
    ])