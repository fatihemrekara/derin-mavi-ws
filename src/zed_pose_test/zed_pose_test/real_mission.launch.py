#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. ZED 2 to MAVROS Bridge
        # ZED'in odometrisini MAVROS'un anlayacagi formata cevirir.
        Node(
            package='zed_pose_test',
            executable='zed_to_mavros_bridge',
            name='zed_bridge',
            output='screen',
            parameters=[{
                'zed_odom_topic': '/zed/zed_node/odom',
                'mavros_vision_topic': '/mavros/vision_pose/pose_cov'
            }]
        ),
        
        # 2. Path Follower (RC Override)
        # videotask'in cizdigi rotayi takip eder. Artik /cmd_vel degil RC Override basiyor.
        Node(
            package='zed_pose_test',
            executable='path_follower',
            name='path_follower',
            output='screen',
            parameters=[{
                'path_topic': '/planned_route',
                'pose_topic': '/mavros/local_position/pose',
                'rc_override_topic': '/mavros/rc/override',
                'v_max': 0.4,
                'w_max': 0.6,
                'lookahead': 1.0,
                'wp_pass_radius': 0.6
            }]
        ),
        
        # 3. Video Task Mission Manager
        # Rotalari cizer, dalis/cikis yapar, donusleri bizzat RC Override ile yonetir.
        Node(
            package='zed_pose_test',
            executable='videotask',
            name='videotask',
            output='screen',
            parameters=[{
                'path_topic': '/planned_route',
                'pose_topic': '/mavros/local_position/pose',
                'rc_topic': '/mavros/rc/override'
            }]
        )
    ])
