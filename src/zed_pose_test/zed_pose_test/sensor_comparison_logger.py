#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import csv
import datetime
import math
import tf_transformations

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

class SensorComparisonLogger(Node):
    def __init__(self):
        super().__init__("sensor_comparison_logger")

        # Parametreler
        self.declare_parameter('log_rate_hz', 20.0)
        log_rate_hz = self.get_parameter('log_rate_hz').value

        # Saklanacak en son veriler
        self.latest_pixhawk_imu = None
        self.latest_zed_imu = None
        self.latest_zed_pose = None
        self.latest_pixhawk_compass = None

        # Abonelikler
        self.sub_px4_imu = self.create_subscription(Imu, '/mavros/imu/data', self.cb_px4_imu, 10)
        self.sub_zed_imu = self.create_subscription(Imu, '/zed/zed_node/imu/data', self.cb_zed_imu, 10)
        self.sub_zed_pose = self.create_subscription(PoseStamped, '/zed/zed_node/pose', self.cb_zed_pose, 10)
        self.sub_px4_compass = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.cb_px4_compass, 10)

        # Log Dosyasi
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"sensor_comparison_{stamp}.csv"
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        
        # Baslik (Header)
        self.writer.writerow([
            "time_sec", 
            "px4_roll", "px4_pitch", "px4_yaw",
            "zed_imu_roll", "zed_imu_pitch", "zed_imu_yaw",
            "zed_pose_x", "zed_pose_y", "zed_pose_z",
            "zed_pose_roll", "zed_pose_pitch", "zed_pose_yaw",
            "px4_compass_hdg"
        ])

        # Zamanlayici (Logger Timer)
        self.timer = self.create_timer(1.0 / log_rate_hz, self.log_timer_callback)
        self.start_time = None
        
        self.get_logger().info(f"Sensor Comparison Node Basladi | Dosya: {self.csv_path}")
        self.get_logger().info("Araci hareket ettirip (tam olarak yaw, pitch, roll yaparak) test edebilirsiniz.")

    def cb_px4_imu(self, msg: Imu):
        self.latest_pixhawk_imu = msg
        
    def cb_zed_imu(self, msg: Imu):
        self.latest_zed_imu = msg
        
    def cb_zed_pose(self, msg: PoseStamped):
        self.latest_zed_pose = msg
        
    def cb_px4_compass(self, msg: Float64):
        self.latest_pixhawk_compass = msg

    def euler_from_imu(self, imu_msg):
        if not imu_msg: return 0.0, 0.0, 0.0
        q = imu_msg.orientation
        r, p, y = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return math.degrees(r), math.degrees(p), math.degrees(y)
        
    def log_timer_callback(self):
        t_now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = t_now
        rel_t = t_now - self.start_time
        
        # Pixhawk IMU
        px4_r, px4_p, px4_y = self.euler_from_imu(self.latest_pixhawk_imu)
        
        # ZED IMU
        zed_i_r, zed_i_p, zed_i_y = self.euler_from_imu(self.latest_zed_imu)
        
        # ZED Pose
        zp_x = zp_y = zp_z = 0.0
        zp_r = zp_p = zp_y_ang = 0.0
        if self.latest_zed_pose:
            p = self.latest_zed_pose.pose.position
            o = self.latest_zed_pose.pose.orientation
            zp_x, zp_y, zp_z = p.x, p.y, p.z
            r, p_, y = tf_transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
            zp_r, zp_p, zp_y_ang = math.degrees(r), math.degrees(p_), math.degrees(y)
            
        # Compass
        compass = 0.0
        if self.latest_pixhawk_compass:
            compass = self.latest_pixhawk_compass.data
            
        self.writer.writerow([
            f"{rel_t:.3f}",
            f"{px4_r:.2f}", f"{px4_p:.2f}", f"{px4_y:.2f}",
            f"{zed_i_r:.2f}", f"{zed_i_p:.2f}", f"{zed_i_y:.2f}",
            f"{zp_x:.3f}", f"{zp_y:.3f}", f"{zp_z:.3f}",
            f"{zp_r:.2f}", f"{zp_p:.2f}", f"{zp_y_ang:.2f}",
            f"{compass:.2f}"
        ])
        
    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SensorComparisonLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
