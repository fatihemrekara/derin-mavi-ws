#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sahte ZED2 Kamera Simulatoru
----------------------------
/robot/ground_truth_pose topic'ini dinler.
Üzerine Gaussian White Noise (anlik titresim) ve Random Walk (drift) ekleyerek
/zed/zed_node/pose topic'ine basar.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def quat_to_yaw(ox, oy, oz, ow):
    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    return math.atan2(siny_cosp, cosy_cosp)

class FakeZEDCamera(Node):

    def __init__(self):
        super().__init__('fake_zed_camera')

        self.declare_parameter('gt_pose_topic', '/robot/ground_truth_pose')
        self.declare_parameter('zed_pose_topic', '/zed/zed_node/pose')
        
        # Kullanici tarafindan verilen VIO Odometry Gurultu Parametreleri
        # Gaussian White Noise (Anlik Ölçüm Gürültüsü) standart sapmasi
        self.declare_parameter('noise_density', 0.03) # metre/radyan
        # Random Walk / Brownian Motion (Zamanla Biriken Kayma - Drift) standart sapmasi
        self.declare_parameter('drift_density', 0.015) # metre/radyan

        gp = lambda n: self.get_parameter(n).value
        
        self.noise_density = float(gp('noise_density'))
        self.drift_density = float(gp('drift_density'))
        
        # Zamanla biriken hata (drift) vektorleri [x, y, yaw]
        self.current_drift = np.zeros(3)

        self.zed_pose_pub = self.create_publisher(
            PoseStamped, str(gp('zed_pose_topic')), 10)
            
        self.zed_path_pub = self.create_publisher(
            Path, '/zed/zed_node/path', 10)
        self.zed_path_msg = Path()
            
        self.gt_sub = self.create_subscription(
            PoseStamped, str(gp('gt_pose_topic')), self.on_gt_pose, 10)

        self.get_logger().info(
            f'Sahte ZED2 Kamera hazir. noise_density={self.noise_density}, '
            f'drift_density={self.drift_density}. /robot/ground_truth_pose bekleniyor...')

    def on_gt_pose(self, msg: PoseStamped):
        # Gercek koordinatlari al
        true_x = msg.pose.position.x
        true_y = msg.pose.position.y
        o = msg.pose.orientation
        true_yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        
        # 1. Anlik Beyaz Gürültü (Gaussian Noise)
        white_noise = np.random.normal(0, self.noise_density, size=3)
        
        # 2. Rastgele Yürüyüş (Random Walk / Drift) - Önceki hataya eklenir
        self.current_drift += np.random.normal(0, self.drift_density, size=3)
        
        # 3. Gerçek veriye gürültülerin eklenmesi
        simulated_x = true_x + white_noise[0] + self.current_drift[0]
        simulated_y = true_y + white_noise[1] + self.current_drift[1]
        simulated_yaw = true_yaw + white_noise[2] + self.current_drift[2]
        
        # Aciyi normalize et [-pi, pi] araligina
        simulated_yaw = math.atan2(math.sin(simulated_yaw), math.cos(simulated_yaw))
        
        # Simulated mesaji olustur
        sim_msg = PoseStamped()
        sim_msg.header = msg.header # Ayni frame_id ve stamp
        sim_msg.pose.position.x = simulated_x
        sim_msg.pose.position.y = simulated_y
        sim_msg.pose.orientation.z = math.sin(simulated_yaw / 2.0)
        sim_msg.pose.orientation.w = math.cos(simulated_yaw / 2.0)
        
        self.zed_pose_pub.publish(sim_msg)

        # Path'i guncelle
        self.zed_path_msg.header = sim_msg.header
        self.zed_path_msg.poses.append(sim_msg)
        if len(self.zed_path_msg.poses) > 2000:
            self.zed_path_msg.poses.pop(0)
        self.zed_path_pub.publish(self.zed_path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeZEDCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
