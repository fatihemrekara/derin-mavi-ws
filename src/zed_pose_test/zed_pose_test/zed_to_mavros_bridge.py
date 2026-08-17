#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZED 2'den MAVROS'a (ArduSub EKF3) Sensör Füzyonu Köprüsü
- Matematiksel Eksen Düzeltmesi Eklendi - 

ZED Wrapper'dan gelen görsel odometriyi (nav_msgs/Odometry) MAVROS'un
görsel pozisyon topic'ine (geometry_msgs/PoseWithCovarianceStamped) 
38 cm offset ve 90 derece Pitch kaymasını düzelterek aktarır.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def quat_mult(p, q):
    """Kuaterniyon çarpımı (q1 * q2)"""
    w = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3]
    x = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2]
    y = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1]
    z = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0]
    return [w, x, y, z]

class ZedToMavrosBridge(Node):
    def __init__(self):
        super().__init__('zed_to_mavros_bridge')
        
        self.declare_parameter('zed_odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('mavros_vision_topic', '/mavros/vision_pose/pose_cov')
        
        # Kamera ile Araç Merkezi Arasındaki Donanımsal Mesafe & Açı Tanımlamaları
        # Pitch: 90 derece aşağı (+1.5707963 radyan)
        # Offset: 0.38 m X ekseninde ileri
        self.cam_pitch_rad = 1.5707963
        self.cam_offset_x = 0.38
        
        # q_bc: base_link -> zed_camera_link rotasyonu
        pitch_half = self.cam_pitch_rad / 2.0
        # q_bc = [w, x, y, z]
        self.q_bc = [math.cos(pitch_half), 0.0, math.sin(pitch_half), 0.0]
        
        # Matematiksel ters işlemi (q_bc_inv)
        self.q_bc_inv = [self.q_bc[0], -self.q_bc[1], -self.q_bc[2], -self.q_bc[3]]
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        zed_topic = self.get_parameter('zed_odom_topic').value
        mav_topic = self.get_parameter('mavros_vision_topic').value

        self.pub = self.create_publisher(PoseWithCovarianceStamped, mav_topic, pub_qos)
        self.sub = self.create_subscription(Odometry, zed_topic, self.odom_callback, sensor_qos)
        
        self.get_logger().info(f"ZED -> MAVROS Bridge (Ekseni Düzeltmeli) Başlatıldı.")
        self.get_logger().info(f"Dinlenen: {zed_topic} | Yayınlanan: {mav_topic}")

    def odom_callback(self, msg: Odometry):
        out_msg = PoseWithCovarianceStamped()
        
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = 'odom' 
        
        # 1- Kamera Orijininden Base Link'e Rotasyon Düzeltmesi
        # q_oc = ZED'den gelen ham lens yönelimi
        q_oc = [msg.pose.pose.orientation.w, 
                msg.pose.pose.orientation.x, 
                msg.pose.pose.orientation.y, 
                msg.pose.pose.orientation.z]
                
        # q_ob = Araç gövdesinin (base_link) dünyaya olan yönelimi = q_oc * q_bc_inv
        q_ob = quat_mult(q_oc, self.q_bc_inv)
        
        # 2- Kamera Merkezli Konumu, Araç Merkezli Konuma Çevirme (Offset Düzeltmesi)
        # R_ob rotasyon matrisinin ilk kolonu (X ekseni vektörü)
        r00 = 1.0 - 2.0 * (q_ob[2]*q_ob[2] + q_ob[3]*q_ob[3])
        r10 = 2.0 * (q_ob[1]*q_ob[2] + q_ob[0]*q_ob[3])
        r20 = 2.0 * (q_ob[1]*q_ob[3] - q_ob[0]*q_ob[2])
        
        # t_ob = t_oc - R_ob * t_bc
        # t_bc_x = 0.38m
        t_oc_x = msg.pose.pose.position.x
        t_oc_y = msg.pose.pose.position.y
        t_oc_z = msg.pose.pose.position.z
        
        t_ob_x = t_oc_x - (r00 * self.cam_offset_x)
        t_ob_y = t_oc_y - (r10 * self.cam_offset_x)
        t_ob_z = t_oc_z - (r20 * self.cam_offset_x)
        
        # Değerleri mesaja yaz
        out_msg.pose.pose.position.x = float(t_ob_x)
        out_msg.pose.pose.position.y = float(t_ob_y)
        out_msg.pose.pose.position.z = float(t_ob_z)
        
        out_msg.pose.pose.orientation.w = float(q_ob[0])
        out_msg.pose.pose.orientation.x = float(q_ob[1])
        out_msg.pose.pose.orientation.y = float(q_ob[2])
        out_msg.pose.pose.orientation.z = float(q_ob[3])
        
        out_msg.pose.covariance = msg.pose.covariance
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZedToMavrosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
