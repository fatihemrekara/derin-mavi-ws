#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from mavros_msgs.msg import VfrHud
from geometry_msgs.msg import PoseArray, Pose

def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a

class DynamicCompassWaypoint(Node):
    def __init__(self):
        super().__init__('dynamic_straight_waypoint')
        
        self.distance = 10.0  # Gidilecek hedef metre uzaklığı
        
        # MAVROS'tan anlık pusulayı oku
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.vfr_sub = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.on_vfr, sensor_qos)
        
        # Rotayı (X,Y metre olarak) planner'a fırlat
        latched_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(PoseArray, '/gps_waypoints', latched_qos)
        
        self.published = False
        self.get_logger().info('Canli Pusula Dinleyici Baslatildi. Aractan pusula acisi bekleniyor...')

    def on_vfr(self, msg: VfrHud):
        if self.published:
            return  # Rotayı sadece node ilk açıldığında 1 kere oluştur ve sabitle
            
        compass = msg.heading
        ned_rad = math.radians(compass)
        
        # MAVROS Pusulasını ROS Metrik (ENU) sistemine dönüştür
        enu_rad = normalize_angle(math.pi/2.0 - ned_rad)
        
        # Dümdüz gidilecek X ve Y metre ofsetlerini Trigonometri ile hesapla
        buoy_x = self.distance * math.cos(enu_rad)
        buoy_y = self.distance * math.sin(enu_rad)
        
        # Senaryo 4 Geri Dönüş Formatı
        points = [
            ("Başlangıç", 0.0, 0.0),
            ("Bitiş", 0.0, 0.0),
            ("Şamandıra", buoy_x, buoy_y)
        ]
        
        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = self.get_clock().now().to_msg()
        
        print('\n' + '='*60)
        print(f' ARAÇ PUSULASI (MAVROS):   {compass} DERECE')
        print(f' BUNUN ROS (ENU) KARŞILIĞI:{math.degrees(enu_rad):.1f} DERECE (Trigonometrik iç açı)')
        print(f' {self.distance} METRE DÜZ GİDİŞ İÇİN MATEMATİKSEL İZ DÜŞÜMLERİ HESAPLANDI.')
        print('='*60)
        
        for name, px, py in points:
            p = Pose()
            p.position.x = float(px)
            p.position.y = float(py)
            p.position.z = 0.0
            p.orientation.w = 1.0
            pa.poses.append(p)
            print(f'> {name:<12}: X Ekseni = {px:>6.2f} Metre, Y Ekseni = {py:>6.2f} Metre')
            
        print('='*60)
        
        self.pub.publish(pa)
        self.published = True
        self.get_logger().info('Bu noktalar route_planner icin /gps_waypoints kanalina yayinlandi. (Ctrl+C basana kadar acik tutun)')

def main(args=None):
    rclpy.init(args=args)
    node = DynamicCompassWaypoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
