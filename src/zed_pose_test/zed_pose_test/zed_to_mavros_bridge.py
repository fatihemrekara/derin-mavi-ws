#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZED 2'den MAVROS'a (ArduSub EKF3) Sensör Füzyonu Köprüsü
- Matematiksel Eksen ve Offset Düzeltmeleri KODDAN KALDIRILDI - 

ZED Wrapper'dan gelen görsel odometriyi (nav_msgs/Odometry) MAVROS'un
görsel pozisyon topic'ine (geometry_msgs/PoseWithCovarianceStamped) aktarır.

Not: Hem offset mesafe (X,Y,Z) hem de sensör yönelimi (VISO_ORIENT=Down)
ayarları QGroundControl üzerinden EKF3'e yaptırılmaktadır. Bu kod sadece
veri tipini (Odometry -> PoseWithCovarianceStamped) Pürüzsüzce MAVROS'a iletmekten sorumludur.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class ZedToMavrosBridge(Node):
    def __init__(self):
        super().__init__('zed_to_mavros_bridge')
        
        self.declare_parameter('zed_odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('mavros_vision_topic', '/mavros/vision_pose/pose_cov')
        
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
        
        self.get_logger().info("ZED -> MAVROS Bridge (Ham Veri / QGC EKF3 Ayarlı) Başlatıldı.")
        self.get_logger().info(f"Dinlenen: {zed_topic} | Yayınlanan: {mav_topic}")

    def odom_callback(self, msg: Odometry):
        out_msg = PoseWithCovarianceStamped()
        
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = 'odom' 
        
        # Kamera pozisyonunu (X, Y, Z) aliyoruz
        out_msg.pose.pose.position = msg.pose.pose.position
        
        # ZED'in ic ice gecmis IMU ve su alti gorsel odometrisinden kaynaklanan 
        # aci/rotasyon suruklenmesini kokten cozmek icin Yonelimi (Orientation) sabitliyoruz.
        # Boylece MAVROS (ArduPilot) ZED'in kendi kendine takla atmasini umursamiyor,
        # sadece yer degistirme miktarini (X,Y) ZED'den, yatay kalma (Pitch/Roll) isini ise
        # dogrudan kendi mukemmel Cube Orange IMU'sundan cozuyor.
        out_msg.pose.pose.orientation.x = 0.0
        out_msg.pose.pose.orientation.y = 0.0
        out_msg.pose.pose.orientation.z = 0.0
        out_msg.pose.pose.orientation.w = 1.0

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

