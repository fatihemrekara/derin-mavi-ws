#!/usr/bin/env python3
"""
Lokal Metrik Waypoint Publisher

Kullanıcının belirlediği X-Y koordinatlı test senaryolarını 
(Başlangıç, Bitiş, Şamandıra) doğrudan metre cinsinden
`route_planner_node` düğümünün istediği formatta (/gps_waypoints) fırlatır.

Senaryolar:
1: Şamandıra=(5,5) Bitiş=(5,-10)
2: Şamandıra=(5,-5) Bitiş=(5,5)
3: Şamandıra=(10,10) Bitiş=(0,0)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseArray, Pose

class LocalWaypointsNode(Node):

    def __init__(self):
        super().__init__('local_waypoints_node')

        self.declare_parameter('scenario', 1)
        self.declare_parameter('frame_id', 'map')
        
        self.scenario = self.get_parameter('scenario').value
        self.frame_id = str(self.get_parameter('frame_id').value)

        # "Latched" QoS (route planner gec acilsa da bu mesaji alsın diye)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
            
        self.pub = self.create_publisher(PoseArray, 'gps_waypoints', qos)

        self.get_logger().info(f'Lokal Waypoint Publisher başlatıldı. Seçilen Senaryo: {self.scenario}')
        self.publish_scenario()

    def publish_scenario(self):
        # Ortak Başlangıç Noktası (Her zaman 0,0)
        start = (0.0, 0.0)
        
        # Senaryolara göre Şamandıra ve Bitiş değerleri
        if self.scenario == 1:
            buoy = (5.0, 5.0)
            end = (5.0, -10.0)
        elif self.scenario == 2:
            buoy = (5.0, -5.0)
            end = (5.0, 5.0)
        elif self.scenario == 3:
            buoy = (10.0, 10.0)
            end = (0.0, 0.0)
        else:
            self.get_logger().warn("1, 2 veya 3 disinda bir senaryo girildi! Varsayilan olarak 1 secildi.")
            buoy = (5.0, 5.0)
            end = (5.0, -10.0)

        # route_planner_node sıralama beklentisi: [BASLANGIC, BITIS, SAMANDIRA]
        points = [
            ("Başlangıç (Orijin)", start),
            ("Bitiş Noktası", end),
            ("Şamandıra Noktası", buoy)
        ]

        # PoseArray oluştur ve noktaları ekle
        pa = PoseArray()
        pa.header.frame_id = self.frame_id
        pa.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info('='*50)
        self.get_logger().info(f' GÖNDERİLEN SENARYO {self.scenario} KOORDİNATLARI (Metre)')
        self.get_logger().info('='*50)
        
        for name, (px, py) in points:
            p = Pose()
            p.position.x = float(px)
            p.position.y = float(py)
            p.position.z = 0.0
            p.orientation.w = 1.0
            pa.poses.append(p)
            self.get_logger().info(f'{name:<20}: X= {px:>6.1f} , Y= {py:>6.1f}')
            
        self.pub.publish(pa)
        self.get_logger().info('='*50)
        self.get_logger().info('Noktalar route_planner (veya şoför) için başarıyla /gps_waypoints topic\'ine bırakıldı!')


def main(args=None):
    rclpy.init(args=args)
    node = LocalWaypointsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
