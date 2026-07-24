#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZED 2'den MAVROS'a (ArduSub EKF3) Sensör Füzyonu Köprüsü

ZED Wrapper'dan gelen görsel odometriyi (nav_msgs/Odometry) MAVROS'un
görsel pozisyon topic'ine (geometry_msgs/PoseWithCovarianceStamped) aktarır.
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
        
        # QoS profilleri: ZED ve MAVROS genellikle BEST_EFFORT veya RELIABLE kullanabilir.
        # Sensör verisi olduğu için BEST_EFFORT uyumluluğunu da destekleyen bir yapı kuruyoruz.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # MAVROS vision topic genellikle reliable bekler ama biz de publish edelim.
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        zed_topic = self.get_parameter('zed_odom_topic').value
        mav_topic = self.get_parameter('mavros_vision_topic').value

        self.pub = self.create_publisher(PoseWithCovarianceStamped, mav_topic, pub_qos)
        
        # Odom dinleyicisi (ZED'in default yayın türüne uymak için BEST_EFFORT)
        self.sub = self.create_subscription(Odometry, zed_topic, self.odom_callback, sensor_qos)
        
        self.get_logger().info(f"ZED -> MAVROS Bridge Başlatıldı.\nDinlenen: {zed_topic}\nYayınlanan: {mav_topic}")

    def odom_callback(self, msg: Odometry):
        out_msg = PoseWithCovarianceStamped()
        
        # Başlık aktarımı (Zaman damgası MAVROS için önemlidir)
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = 'odom'  # ArduSub EKF3 genellikle 'odom' bekler
        
        # Pozisyon ve Yönelim aktarımı
        out_msg.pose.pose.position = msg.pose.pose.position
        out_msg.pose.pose.orientation = msg.pose.pose.orientation
        
        # Kovaryans matrisi aktarımı (ZED görüşü kaybettiğinde buralar yüksek gelecektir, 
        # ArduSub EKF3 bu sayede görüşü iptal edip IMU'ya geçer).
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
