#!/usr/bin/env python3
"""
Geri Dönüş / Kare Rota Planlayıcı (Return Route Planner)

- mode="out_and_back" (U-Dönüşü): Araç düz (length kadar) gider, sonra başladığı yere aynı çizgi üzerinden geri döner (180 derece).
- mode="square" (Kare): Araç düz gider (length), 90 derece döner length gider, tekrar 90 döner... başladığı noktada kareyi tamamlar.
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

class ReturnRoutePlannerNode(Node):

    def __init__(self):
        super().__init__('return_route_planner')

        self.declare_parameter('length', 10.0)
        self.declare_parameter('step', 0.5)
        self.declare_parameter('mode', 'square')  # 'out_and_back' veya 'square'
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')

        self.length = float(self.get_parameter('length').value)
        self.step = float(self.get_parameter('step').value)
        self.mode = str(self.get_parameter('mode').value).lower()
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
        
        # Aracın pozunu bekle
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, sensor_qos)
        self.route_published = False

        self.get_logger().info(
            f'Geri Donus Planlayici basladi. (Mod: {self.mode}, Uzunluk: {self.length}m)')
        self.get_logger().info('Aracın ilk pozisyonu bekleniyor...')

    def on_pose(self, msg: PoseStamped):
        if self.route_published:
            return
            
        x0 = msg.pose.position.x
        y0 = msg.pose.position.y
        yaw0 = quat_to_yaw(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        
        self.get_logger().info(f"Başlangıç pozu (Start) Kaydedildi: X={x0:.2f}, Y={y0:.2f}, Açısı={math.degrees(yaw0):.1f} deg")
        
        if self.mode == 'square':
            self.publish_square_route(x0, y0, yaw0)
        else:
            self.publish_out_and_back_route(x0, y0, yaw0)
            
        self.route_published = True

    def _rotate_point(self, local_x, local_y, ox, oy, yaw):
        """Yerel (Robot) koordinatını Global harita koordinatına çevirir"""
        gx = ox + local_x * math.cos(yaw) - local_y * math.sin(yaw)
        gy = oy + local_x * math.sin(yaw) + local_y * math.cos(yaw)
        return (gx, gy)

    def publish_out_and_back_route(self, start_x, start_y, yaw):
        pts = []
        n_points = max(2, int(math.ceil(self.length / self.step)))
        
        # 1. Ileri gidis yolu (Start -> Gidis Noktasi)
        for i in range(n_points + 1):
            dist = float(i * self.step)
            pts.append(self._rotate_point(dist, 0.0, start_x, start_y, yaw))
            
        # 2. Geri donus yolu (Gidis Noktasi -> Start)
        for i in range(n_points - 1, -1, -1):
            dist = float(i * self.step)
            pts.append(self._rotate_point(dist, 0.0, start_x, start_y, yaw))

        self.publish_path(pts, yaw)
        self.publish_markers(pts)
        self.get_logger().info(f'GİDİP-GELME (180 Derece) rotası yayınlandı. Toplam {len(pts)} wp.')

    def publish_square_route(self, start_x, start_y, yaw):
        pts = []
        n_points = max(2, int(math.ceil(self.length / self.step)))
        
        # KARE Rota (Her kose 90 derece oldugu icin Pure Pursuit dogal olarak o koselerde 90 derece donecektir)
        # Kenar 1: İleri (+X Yönü)
        for i in range(n_points + 1):
            pts.append(self._rotate_point(float(i * self.step), 0.0, start_x, start_y, yaw))
            
        # Kenar 2: Sola (+Y Yönü)
        for i in range(1, n_points + 1):
            pts.append(self._rotate_point(self.length, float(i * self.step), start_x, start_y, yaw))

        # Kenar 3: Geriye (-X Yönü)
        for i in range(1, n_points + 1):
            pts.append(self._rotate_point(self.length - float(i * self.step), self.length, start_x, start_y, yaw))
            
        # Kenar 4: Sağa doğru asıl başlangıca dönüş (-Y Yönü)
        for i in range(1, n_points + 1):
            pts.append(self._rotate_point(0.0, self.length - float(i * self.step), start_x, start_y, yaw))

        self.publish_path(pts, yaw)
        self.publish_markers(pts)
        self.get_logger().info(f'KARE (90 Derecelik dönüşler) rotası yayınlandı. Toplam {len(pts)} wp.')


    def publish_path(self, pts, yaw):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(pts):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
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

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp
        m.ns = 'return_route'
        m.id = 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.06
        m.color.g, m.color.a = 1.0, 1.0
        for x, y in pts:
            m.points.append(Point(x=x, y=y))
        ma.markers.append(m)
        
        m_end = Marker()
        m_end.header = m.header
        m_end.ns = 'return_route'
        m_end.id = 2
        m_end.type = Marker.SPHERE
        m_end.action = Marker.ADD
        m_end.pose.position.x = pts[-1][0]
        m_end.pose.position.y = pts[-1][1]
        m_end.scale.x = m_end.scale.y = m_end.scale.z = 0.5
        m_end.color.r, m_end.color.g, m_end.color.b, m_end.color.a = 1.0, 0.0, 0.0, 1.0
        ma.markers.append(m_end)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ReturnRoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
