#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Basit Kalman Filtresi (Sensor Fusion)
-------------------------------------
/zed/zed_node/pose (Gürültülü x, y, yaw) ve /imu/data (Gürültülü açısal hız w)
verilerini birleştirerek daha kararlı bir /robot/filtered_pose üretir.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path

def quat_to_yaw(ox, oy, oz, ow):
    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    return math.atan2(siny_cosp, cosy_cosp)

class SensorFusionEKF(Node):
    def __init__(self):
        super().__init__('sensor_fusion_ekf')
        
        self.declare_parameter('zed_pose_topic', '/zed/zed_node/pose')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('filtered_pose_topic', '/robot/filtered_pose')
        
        gp = lambda n: self.get_parameter(n).value
        
        # Kalman Filtresi Durum Matrisi X = [x, y, yaw]^T
        self.X = np.zeros(3)
        
        # Hata Kovaryans Matrisi P
        self.P = np.eye(3) * 1.0
        
        # Süreç Gürültüsü (Process Noise) Q
        # IMU ne kadar güvenilir? (Düşük değer = IMU'ya çok güven)
        self.Q = np.diag([0.01, 0.01, 0.005]) 
        
        # Ölçüm Gürültüsü (Measurement Noise) R
        # ZED ne kadar güvenilir? (Yüksek değer = ZED'in gürültülü/driftli olduğunu kabul et)
        self.R = np.diag([0.5, 0.5, 0.5])
        
        self.last_imu_time = None
        
        self.pose_pub = self.create_publisher(PoseStamped, str(gp('filtered_pose_topic')), 10)
        self.path_pub = self.create_publisher(Path, str(gp('filtered_pose_topic')) + '_path', 10)
        self.path_msg = Path()
        
        self.imu_sub = self.create_subscription(Imu, str(gp('imu_topic')), self.on_imu, 10)
        self.zed_sub = self.create_subscription(PoseStamped, str(gp('zed_pose_topic')), self.on_zed, 10)
        
        self.get_logger().info("Sensor Fusion (Kalman Filter) başlatıldı. IMU ve ZED verileri bekleniyor...")

    def on_imu(self, msg: Imu):
        # Tahmin (Predict) Adımı
        current_time = self.get_clock().now()
        
        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return
            
        dt = (current_time - self.last_imu_time).nanoseconds * 1e-9
        self.last_imu_time = current_time
        
        if dt <= 0: return
        
        w = msg.angular_velocity.z
        
        # Durumu güncelle: Sadece yaw değişiyor (DVL olmadığı için x ve y hızlarını bilmiyoruz)
        self.X[2] += w * dt
        self.X[2] = math.atan2(math.sin(self.X[2]), math.cos(self.X[2]))
        
        # Kovaryansı güncelle
        self.P = self.P + self.Q
        
        self.publish_filtered_pose(msg.header.stamp)

    def on_zed(self, msg: PoseStamped):
        # Güncelleme (Update) Adımı
        z_x = msg.pose.position.x
        z_y = msg.pose.position.y
        z_yaw = quat_to_yaw(msg.pose.orientation.x, msg.pose.orientation.y, 
                            msg.pose.orientation.z, msg.pose.orientation.w)
        
        Z = np.array([z_x, z_y, z_yaw])
        
        # İnovasyon (Fark)
        Y = Z - self.X
        # Açı farkını normalize et [-pi, pi]
        Y[2] = math.atan2(math.sin(Y[2]), math.cos(Y[2]))
        
        # Kalman Kazancı (Kalman Gain)
        S = self.P + self.R
        try:
            K = self.P @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
            
        # Durumu yeni ölçümle düzelt
        self.X = self.X + K @ Y
        self.X[2] = math.atan2(math.sin(self.X[2]), math.cos(self.X[2]))
        
        # Kovaryansı küçült
        self.P = (np.eye(3) - K) @ self.P
        
        self.publish_filtered_pose(msg.header.stamp)

    def publish_filtered_pose(self, stamp):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp
        
        msg.pose.position.x = float(self.X[0])
        msg.pose.position.y = float(self.X[1])
        msg.pose.position.z = 0.0
        
        yaw = float(self.X[2])
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.pose_pub.publish(msg)
        
        # Path için ekleme yap (RViz'de iz bırakmak için)
        self.path_msg.header = msg.header
        self.path_msg.poses.append(msg)
        if len(self.path_msg.poses) > 2000:
            self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionEKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
