#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import os
import math

class PassiveSensorLogger(Node):
    def __init__(self):
        super().__init__('passive_sensor_logger')
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        
        bridge_qos = QoSProfile(  # bridge publisher uses RELIABLE
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
            
        self.ekf_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.ekf_cb, sensor_qos)
        self.zed_odom_sub = self.create_subscription(Odometry, '/zed/zed_node/odom', self.zed_odom_cb, sensor_qos)
        self.bridge_sub = self.create_subscription(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', self.bridge_cb, bridge_qos)
        self.zed_imu_sub = self.create_subscription(Imu, '/zed/zed_node/imu/data', self.zed_imu_cb, sensor_qos)
        self.cube_imu_sub = self.create_subscription(Imu, '/mavros/imu/data', self.cube_imu_cb, sensor_qos)
        
        self.ekf_x, self.ekf_y, self.ekf_z = 0.0, 0.0, 0.0
        self.ekf_qx, self.ekf_qy, self.ekf_qz, self.ekf_qw = 0.0, 0.0, 0.0, 1.0
        
        self.zed_x, self.zed_y, self.zed_z = 0.0, 0.0, 0.0
        self.zed_qx, self.zed_qy, self.zed_qz, self.zed_qw = 0.0, 0.0, 0.0, 1.0
        
        self.bridge_x, self.bridge_y, self.bridge_z = 0.0, 0.0, 0.0
        self.bridge_qx, self.bridge_qy, self.bridge_qz, self.bridge_qw = 0.0, 0.0, 0.0, 1.0
        
        self.zed_imu_ax, self.zed_imu_ay, self.zed_imu_az = 0.0, 0.0, 0.0
        self.cube_imu_ax, self.cube_imu_ay, self.cube_imu_az = 0.0, 0.0, 0.0
        
        self.start_time = self.get_clock().now()
        
        from datetime import datetime
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'passive_sensor_log_{stamp}.csv')
        self.log_file = open(log_path, 'w')
        self.log_file.write(
            "Time(s),EKF_X,EKF_Y,EKF_Z,ZED_X,ZED_Y,ZED_Z,Bridge_X,Bridge_Y,Bridge_Z,"
            "ZED_IMU_Ax,ZED_IMU_Ay,ZED_IMU_Az,Cube_IMU_Ax,Cube_IMU_Ay,Cube_IMU_Az\n"
        )
        
        self.timer = self.create_timer(1.0, self.control_loop)
        self.get_logger().info("PASIF KAYIT (Passive Sensor Logger) baslatildi.")
        self.get_logger().info("Arac suda rölantideyken sensor kaymalari gozlemleniyor...")
        self.get_logger().info(f"Log dizini: {log_path}")
        
    def ekf_cb(self, msg):
        self.ekf_x = msg.pose.position.x
        self.ekf_y = msg.pose.position.y
        self.ekf_z = msg.pose.position.z
        self.ekf_qx = msg.pose.orientation.x
        self.ekf_qy = msg.pose.orientation.y
        self.ekf_qz = msg.pose.orientation.z
        self.ekf_qw = msg.pose.orientation.w
        
    def zed_odom_cb(self, msg):
        self.zed_x = msg.pose.pose.position.x
        self.zed_y = msg.pose.pose.position.y
        self.zed_z = msg.pose.pose.position.z
        self.zed_qx = msg.pose.pose.orientation.x
        self.zed_qy = msg.pose.pose.orientation.y
        self.zed_qz = msg.pose.pose.orientation.z
        self.zed_qw = msg.pose.pose.orientation.w
        
    def bridge_cb(self, msg):
        self.bridge_x = msg.pose.pose.position.x
        self.bridge_y = msg.pose.pose.position.y
        self.bridge_z = msg.pose.pose.position.z
        self.bridge_qx = msg.pose.pose.orientation.x
        self.bridge_qy = msg.pose.pose.orientation.y
        self.bridge_qz = msg.pose.pose.orientation.z
        self.bridge_qw = msg.pose.pose.orientation.w
        
    def zed_imu_cb(self, msg):
        self.zed_imu_ax = msg.linear_acceleration.x
        self.zed_imu_ay = msg.linear_acceleration.y
        self.zed_imu_az = msg.linear_acceleration.z

    def cube_imu_cb(self, msg):
        self.cube_imu_ax = msg.linear_acceleration.x
        self.cube_imu_ay = msg.linear_acceleration.y
        self.cube_imu_az = msg.linear_acceleration.z

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.degrees(math.atan2(t0, t1))
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.degrees(math.asin(t2))
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.degrees(math.atan2(t3, t4))
        
        return roll, pitch, yaw

    def control_loop(self):
        now = self.get_clock().now()
        t_sec = (now - self.start_time).nanoseconds * 1e-9
        
        ekf_r, ekf_p, ekf_y = self.euler_from_quaternion(self.ekf_qx, self.ekf_qy, self.ekf_qz, self.ekf_qw)
        zed_r, zed_p, zed_y = self.euler_from_quaternion(self.zed_qx, self.zed_qy, self.zed_qz, self.zed_qw)
        
        self.get_logger().info(f"--- T={t_sec:.1f}s ---")
        self.get_logger().info(f"EKF Pos:  X={self.ekf_x:.2f}, Y={self.ekf_y:.2f}, Z={self.ekf_z:.2f} | R={ekf_r:.1f}, P={ekf_p:.1f}, Y={ekf_y:.1f}")
        self.get_logger().info(f"ZED Pos:  X={self.zed_x:.2f}, Y={self.zed_y:.2f}, Z={self.zed_z:.2f} | R={zed_r:.1f}, P={zed_p:.1f}, Y={zed_y:.1f}")
        self.get_logger().info(f"IMU AccZ: Cube={self.cube_imu_az:.2f}, ZED={self.zed_imu_az:.2f}")
        
        self.log_file.write(
            f"{t_sec:.2f},{self.ekf_x:.3f},{self.ekf_y:.3f},{self.ekf_z:.3f},"
            f"{self.zed_x:.3f},{self.zed_y:.3f},{self.zed_z:.3f},"
            f"{self.bridge_x:.3f},{self.bridge_y:.3f},{self.bridge_z:.3f},"
            f"{self.zed_imu_ax:.3f},{self.zed_imu_ay:.3f},{self.zed_imu_az:.3f},"
            f"{self.cube_imu_ax:.3f},{self.cube_imu_ay:.3f},{self.cube_imu_az:.3f}\n"
        )
        
def main(args=None):
    rclpy.init(args=args)
    node = PassiveSensorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Test bitti, log kaydedildi.")
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
