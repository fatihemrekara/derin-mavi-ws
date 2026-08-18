#!/usr/bin/env python3
"""
Dinamik Düz Rota Planlayici (Straight Route Planner)

Aracın o anki bulunduğu konumdan (current_x, current_y) başlayıp,
o an baktığı yöne (current_yaw) doğru dümdüz bir rota oluşturur.
Böylece araç başlangıçta olduğu yerde dönmeye (hizalanmaya) gerek duymaz.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

def quat_to_yaw(ox, oy, oz, ow):
    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    return math.atan2(siny_cosp, cosy_cosp)

class StraightRoutePlannerNode(Node):

    def __init__(self):
        super().__init__('straight_route_planner')

        self.declare_parameter('length', 10.0)
        self.declare_parameter('step', 0.5)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')

        self.length = float(self.get_parameter('length').value)
        self.step = float(self.get_parameter('step').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
            
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.path_pub = self.create_publisher(Path, 'planned_route', latched)
        self.marker_pub = self.create_publisher(MarkerArray, 'route_markers', latched)
        
        # Aracın o anki pozunu almak için (2 saniye bekleme eklendi)
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, sensor_qos)
        
        self.route_published = False
        self.pose_samples = []
        self.sampling_start_time = None
        self.sampling_duration = 2.0  # saniye

        self.get_logger().info(
            f'Dinamik Düz Rota Planlayici basladi. İlk {self.sampling_duration} saniye veri toplanacak... '
            f'(Topic: {self.pose_topic}, Uzunluk={self.length} m)')

    def on_pose(self, msg: PoseStamped):
        if self.route_published:
            return  # Rota 1 kere yayınlanır
            
        now = self.get_clock().now()
        if self.sampling_start_time is None:
            self.sampling_start_time = now
            self.get_logger().info(f'[{self.get_name()}] Başlangıç pozisyonu için {self.sampling_duration} saniye dinleniyor...')
            
        elapsed = (now - self.sampling_start_time).nanoseconds * 1e-9
        
        yaw = quat_to_yaw(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.pose_samples.append((msg.pose.position.x, msg.pose.position.y, yaw))
        
        if elapsed >= self.sampling_duration:
            if not self.pose_samples:
                return
            
            x0 = sum(p[0] for p in self.pose_samples) / len(self.pose_samples)
            y0 = sum(p[1] for p in self.pose_samples) / len(self.pose_samples)
            
            # Açıların aritmetik ortalaması yerine vektörel ortalaması (atan2)
            sum_sin = sum(math.sin(p[2]) for p in self.pose_samples)
            sum_cos = sum(math.cos(p[2]) for p in self.pose_samples)
            yaw0 = math.atan2(sum_sin, sum_cos)
            
            self.get_logger().info(f"Aracın başlangıç pozu SABİTLENDİ (Ortalama): X={x0:.2f}, Y={y0:.2f}, Yaw={math.degrees(yaw0):.1f} derece;")
            self.publish_route(x0, y0, yaw0)
            
            self.route_published = True

    def publish_route(self, start_x, start_y, yaw):
        pts = []
        n_points = max(2, int(math.ceil(self.length / self.step)))
        
        for i in range(n_points + 1):
            dist = float(i * self.step)
            # Aracin baktigi aciya (yaw) gore X ve Y eksenine izdusumleri ekle
            x = start_x + dist * math.cos(yaw)
            y = start_y + dist * math.sin(yaw)
            pts.append((x, y))

        self.publish_path(pts, yaw)
        self.publish_markers(pts)

        self.get_logger().info(f'Dinamik dümdüz rota yayinlandi: {len(pts)} nokta, toplam {self.length} m.')

    def publish_path(self, pts, yaw):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(pts):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            
            # Tüm noktalar aracın mevcut yönünde (ileriye) bakacak şekilde yönlendirilir
            qx, qy, qz, qw = yaw_to_quat(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        self.path_pub.publish(path)

    def publish_markers(self, pts):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def base(mid, mtype):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = 'route'
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        m = base(1, Marker.LINE_STRIP)
        m.scale.x = 0.06
        m.color.g, m.color.a = 1.0, 1.0
        for x, y in pts:
            m.points.append(Point(x=x, y=y))
        ma.markers.append(m)
        
        m_end = base(2, Marker.SPHERE)
        m_end.pose.position.x = pts[-1][0]
        m_end.pose.position.y = pts[-1][1]
        m_end.scale.x = m_end.scale.y = m_end.scale.z = 0.5
        m_end.color.r, m_end.color.a = 1.0, 1.0
        ma.markers.append(m_end)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = StraightRoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
