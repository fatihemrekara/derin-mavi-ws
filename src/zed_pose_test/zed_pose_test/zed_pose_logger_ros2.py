#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZED 2 - Koridor SLAM / Drift Testi (ROS 2 node)
------------------------------------------------
zed-ros2-wrapper'in yayinladigi pose topic'ine abone olur,
anlik pozisyonu metre cinsinden terminale ve CSV'ye loglar.
Node kapatildiginda (Ctrl+C) drift raporu yazar.

Once ZED wrapper'i baslatin:
    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2

Sonra bu node'u calistirin:
    ros2 run <paket_adi> zed_pose_logger_ros2
    # veya dogrudan:
    python3 zed_pose_logger_ros2.py

Parametreler:
    pose_topic      (default: /zed/zed_node/pose)
    console_log_hz  (default: 5.0)

Not: Wrapper zaten acilis pozunu origin (0,0,0) alir. Testi
istediginiz anda sifirlamak icin:
    ros2 service call /zed/zed_node/reset_pos_tracking std_srvs/srv/Trigger
"""

import math
import csv
import datetime
import tf_transformations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class ZedPoseLogger(Node):
    def __init__(self):
        super().__init__("zed_pose_logger")

        self.declare_parameter("pose_topic", "/zed/zed_node/pose")
        self.declare_parameter("console_log_hz", 5.0)

        topic = self.get_parameter("pose_topic").value
        self.console_period = 1.0 / float(self.get_parameter("console_log_hz").value)

        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"zed_pose_log_{stamp}.csv"
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["t_sec", "x_m", "y_m", "z_m",
                              "roll_deg", "pitch_deg", "yaw_deg", "dist_from_origin_m"])

        self.t0 = None
        self.last_console_t = -1e9
        self.last_xyz = (0.0, 0.0, 0.0)
        self.msg_count = 0

        self.sub = self.create_subscription(PoseStamped, topic, self.cb_pose, 10)

        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Pose topic: {topic}")
        self.get_logger().info(f"CSV: {self.csv_path}")
        self.get_logger().info("Acilis noktasi = (0,0,0). Ctrl+C -> drift raporu.")
        self.get_logger().info("=" * 50)

    def cb_pose(self, msg: PoseStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        rel_t = t - self.t0

        p = msg.pose.position
        o = msg.pose.orientation
        x, y, z = p.x, p.y, p.z
        
        # Tam roll, pitch, yaw aliniyor
        roll, pitch, yaw_rad = tf_transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
        yaw = math.degrees(yaw_rad)
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        
        dist = math.sqrt(x * x + y * y + z * z)

        self.writer.writerow([f"{rel_t:.3f}", f"{x:.4f}", f"{y:.4f}",
                              f"{z:.4f}", f"{roll_deg:.2f}", f"{pitch_deg:.2f}", f"{yaw:.2f}", f"{dist:.4f}"])
        self.last_xyz = (x, y, z)
        self.msg_count += 1

        if rel_t - self.last_console_t >= self.console_period:
            self.last_console_t = rel_t
            self.get_logger().info(
                f"[{rel_t:7.2f}s] X={x:+7.3f}m Y={y:+7.3f}m Z={z:+7.3f}m "
                f"R={roll_deg:+7.1f} P={pitch_deg:+7.1f} Y={yaw:+7.1f}deg |d|={dist:6.3f}m")

    def report_and_close(self):
        self.csv_file.close()
        x, y, z = self.last_xyz
        dist = math.sqrt(x * x + y * y + z * z)
        print("\n" + "=" * 50)
        print("TEST BITTI - DRIFT RAPORU")
        print(f"  Alinan mesaj       : {self.msg_count}")
        print(f"  Son pozisyon       : X={x:+.3f} m, Y={y:+.3f} m, Z={z:+.3f} m")
        print(f"  Baslangica uzaklik : {dist:.3f} m   <-- drift (ideali ~0)")
        print(f"  CSV                : {self.csv_path}")
        print("=" * 50)


def main(args=None):
    rclpy.init(args=args)
    node = ZedPoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report_and_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()