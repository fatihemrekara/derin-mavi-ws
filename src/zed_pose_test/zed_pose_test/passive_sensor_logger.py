#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64
from mavros_msgs.msg import VfrHud
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import math
import os

class PassiveSensorLogger(Node):
    def __init__(self):
        super().__init__('passive_sensor_logger')
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
            
        self.hdg_sub = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.hdg_cb, sensor_qos)
        self.vfr_sub = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.vfr_cb, sensor_qos)
        self.rel_alt_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.rel_alt_cb, sensor_qos)
        
        self.ekf_pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.ekf_cb, sensor_qos)
        self.zed_pose_sub = self.create_subscription(PoseStamped, '/zed/zed_node/pose', self.zed_cb, sensor_qos)
        self.imu_sub = self.create_subscription(Imu, '/mavros/imu/data', self.imu_cb, sensor_qos)
        
        self.compass_hdg = 0.0
        self.altitude = 0.0
        self.rel_alt = 0.0
        
        self.ekf_x, self.ekf_y, self.ekf_z = 0.0, 0.0, 0.0
        self.zed_x, self.zed_y, self.zed_z = 0.0, 0.0, 0.0
        self.imu_acc_x, self.imu_acc_y, self.imu_acc_z = 0.0, 0.0, 0.0
        
        self.start_time = self.get_clock().now()
        
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'passive_sensor_log.csv')
        self.log_file = open(log_path, 'w')
        self.log_file.write("Time(s),RelAlt(m),Compass_Hdg(deg),EKF_X,EKF_Y,EKF_Z,ZED_X,ZED_Y,ZED_Z,IMU_AccX,IMU_AccY,IMU_AccZ\n")
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Passive Sensor Logger (PASIF TEST) scripti baslatildi.")
        self.get_logger().info(f"Loglar {log_path} dosyasina kaydediliyor...")
        self.get_logger().info("Durdurmak icin CTRL+C yapin.")
        
    def hdg_cb(self, msg): self.compass_hdg = msg.data
    def vfr_cb(self, msg): self.altitude = msg.altitude
    def rel_alt_cb(self, msg): self.rel_alt = msg.data
    
    def ekf_cb(self, msg):
        self.ekf_x = msg.pose.position.x
        self.ekf_y = msg.pose.position.y
        self.ekf_z = msg.pose.position.z
        
    def zed_cb(self, msg):
        self.zed_x = msg.pose.position.x
        self.zed_y = msg.pose.position.y
        self.zed_z = msg.pose.position.z
        
    def imu_cb(self, msg):
        self.imu_acc_x = msg.linear_acceleration.x
        self.imu_acc_y = msg.linear_acceleration.y
        self.imu_acc_z = msg.linear_acceleration.z

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        
        if int(elapsed * 10) % 10 == 0:
            self.get_logger().info(f"LOGLANIYOR... EKF(X:{self.ekf_x:.2f}, Y:{self.ekf_y:.2f}) ZED(X:{self.zed_x:.2f}, Y:{self.zed_y:.2f})")
            
        t_sec = now.nanoseconds * 1e-9
        self.log_file.write(f"{t_sec:.2f},{self.rel_alt:.2f},{self.compass_hdg:.2f},{self.ekf_x:.2f},{self.ekf_y:.2f},{self.ekf_z:.2f},{self.zed_x:.2f},{self.zed_y:.2f},{self.zed_z:.2f},{self.imu_acc_x:.2f},{self.imu_acc_y:.2f},{self.imu_acc_z:.2f}\n")
        
def main(args=None):
    rclpy.init(args=args)
    node = PassiveSensorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Test durduruluyor, log dosyasi kapatiliyor...")
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
