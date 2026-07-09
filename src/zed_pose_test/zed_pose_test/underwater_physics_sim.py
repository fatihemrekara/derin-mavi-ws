#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sualti Arac Fizik Simulatoru (kinematik / unicycle model)
---------------------------------------------------------
/cmd_vel'i dinler, basit tek-izli (unicycle) modelle gercek pozu hesaplar ve
/robot/ground_truth_pose topic'inden yayinlar.
Dalga ve akinti etkileri su an icin deaktif edilmistir.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path


class UnderwaterPhysicsSim(Node):

    def __init__(self):
        super().__init__('underwater_physics_sim')

        self.declare_parameter('gt_pose_topic', '/robot/ground_truth_pose')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw_deg', 0.0)
        self.declare_parameter('cmd_timeout', 0.5)

        gp = lambda n: self.get_parameter(n).value
        self.dt = 1.0 / float(gp('rate_hz'))
        self.x = float(gp('start_x'))
        self.y = float(gp('start_y'))
        self.yaw = math.radians(float(gp('start_yaw_deg')))
        self.cmd_timeout = float(gp('cmd_timeout'))

        self.v = 0.0
        self.w = 0.0
        self.last_cmd_time = None

        # capraz iz hatasi istatistikleri (gercek fiziksel konum uzerinden)
        self.path_pts = []
        self.cte_max = 0.0
        self.cte_sum = 0.0
        self.cte_n = 0

        self.gt_pose_pub = self.create_publisher(
            PoseStamped, str(gp('gt_pose_topic')), 10)
        self.gt_path_pub = self.create_publisher(
            Path, '/robot/ground_truth_path', 10)
        
        self.gt_path_msg = Path()

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd, 10)

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.path_sub = self.create_subscription(
            Path, '/planned_route', self.on_path, latched)

        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f'Fiziksel arac hazir: baslangic=({self.x:.2f}, {self.y:.2f}, '
            f'{math.degrees(self.yaw):.0f} derece). /cmd_vel bekleniyor...')

    def on_cmd(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def on_path(self, msg: Path):
        self.path_pts = [(p.pose.position.x, p.pose.position.y)
                         for p in msg.poses]
        self.cte_max = 0.0
        self.cte_sum = 0.0
        self.cte_n = 0
        self.get_logger().info(
            f'Rota alindi ({len(self.path_pts)} nokta), Gercek Konum ile CTE olcumu basladi.')

    def step(self):
        # komut bayatsa dur
        if self.last_cmd_time is not None:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                self.v = 0.0
                self.w = 0.0

        # unicycle entegrasyonu (su an saf kinematik, akinti/dalga eklenebilir)
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += self.w * self.dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.gt_pose_pub.publish(msg)

        # Path'i guncelle
        self.gt_path_msg.header = msg.header
        self.gt_path_msg.poses.append(msg)
        if len(self.gt_path_msg.poses) > 2000:
            self.gt_path_msg.poses.pop(0)
        self.gt_path_pub.publish(self.gt_path_msg)

        # hareket halindeyken capraz iz hatasini olc
        if self.path_pts and abs(self.v) > 1e-3:
            cte = self.cross_track_error()
            self.cte_max = max(self.cte_max, cte)
            self.cte_sum += cte
            self.cte_n += 1
            self.get_logger().info(
                f'Gercek Poz=({self.x:+6.2f}, {self.y:+6.2f})  '
                f'yaw={math.degrees(self.yaw):+6.1f}deg  '
                f'v={self.v:+.2f}  w={self.w:+.2f}  CTE={cte:.3f} m',
                throttle_duration_sec=1.0)

    def cross_track_error(self):
        """Aracin rotadaki en yakin SEGMENTE dik uzakligi."""
        best = float('inf')
        px, py = self.x, self.y
        for i in range(len(self.path_pts) - 1):
            ax, ay = self.path_pts[i]
            bx, by = self.path_pts[i + 1]
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy
            if L2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
            dx, dy = px - (ax + t * vx), py - (ay + t * vy)
            d = math.hypot(dx, dy)
            if d < best:
                best = d
        return best

    def report(self):
        print('\n' + '=' * 50)
        print('FIZIKSEL SIMULASYON RAPORU')
        if self.cte_n:
            print(f'  Ornek sayisi        : {self.cte_n}')
            print(f'  Ortalama CTE        : {self.cte_sum / self.cte_n:.3f} m')
            print(f'  Maksimum CTE        : {self.cte_max:.3f} m')
        else:
            print('  Hic hareket olcumu yok.')
        print(f'  Son poz             : ({self.x:+.2f}, {self.y:+.2f}), '
              f'yaw={math.degrees(self.yaw):+.1f} deg')
        print('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = UnderwaterPhysicsSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
