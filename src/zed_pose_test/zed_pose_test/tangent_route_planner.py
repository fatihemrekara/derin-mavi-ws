#!/usr/bin/env python3
"""
tangent_route_planner.py
Samandira etrafinda tam donusleri u-donus formatinda hayali teget cizgilerinin kesisimi ile hesaplayan rota planlayici.
Baslangic ve bitis noktalarindan, samandira etrafinda olusturulan sanal cemberin uygun tegetlerine cizgiler cekilir.
Bu iki teget dogrusunun kesisimi aracin donus yapacagi tek kesisim (vertex) noktasini olusturur.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, PoseArray
from visualization_msgs.msg import Marker, MarkerArray

def cross2(ax, ay, bx, by):
    return ax * by - ay * bx

def tangent_points(p, c, r):
    dx, dy = p[0] - c[0], p[1] - c[1]
    d = math.hypot(dx, dy)
    if d <= r:
        r = max(0.01, d * 0.99)
    base = math.atan2(dy, dx)
    alpha = math.acos(r / d)
    t1 = (c[0] + r * math.cos(base + alpha), c[1] + r * math.sin(base + alpha))
    t2 = (c[0] + r * math.cos(base - alpha), c[1] + r * math.sin(base - alpha))
    return t1, t2

def pick_tangent_opposite(p, c, r, ref, logger, label):
    """
    Teget seciminde, referans noktasinin bulundugu alanin ZIT yonundeki tegeti secer.
    Yani (Start -> Buoy) cizgisinin ikiye boldugu alanda End noktasinin tersi tarafinda kalan teget.
    """
    t1, t2 = tangent_points(p, c, r)
    v_pc = (c[0] - p[0], c[1] - p[1])
    side_ref = cross2(v_pc[0], v_pc[1], ref[0] - p[0], ref[1] - p[1])
    side_t1 = cross2(v_pc[0], v_pc[1], t1[0] - p[0], t1[1] - p[1])
    
    if abs(side_ref) < 1e-3:
        # Eger referans noktasi (Bitis) tam cizginin ustundeyse, herhangi bir teget secilebilir
        logger.info(f"[{label}] Referans noktasi cizgi uzerinde, standart teget secildi.")
        return t1
        
    return t1 if side_ref * side_t1 < 0 else t2

def line_intersection(p1, p2, p3, p4):
    """
    p1, p2'den gecen 1. dogru ile p3, p4'ten gecen 2. dogrunun kesisim noktasini dondurur.
    Paralelse None doner.
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-6:
        return None
        
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    px = x1 + t * (x2 - x1)
    py = y1 + t * (y2 - y1)
    return (px, py)

def sample_segment(a, b, step):
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(math.ceil(dist / step)))
    return [(a[0] + (b[0] - a[0]) * i / n,
             a[1] + (b[1] - a[1]) * i / n) for i in range(1, n + 1)]

def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

class TangentRoutePlannerNode(Node):

    def __init__(self):
        super().__init__('tangent_route_planner')

        self.declare_parameter('radius', 3.0)
        self.declare_parameter('step', 0.5)
        self.declare_parameter('frame_id', 'map')

        self.radius = float(self.get_parameter('radius').value)
        self.step = float(self.get_parameter('step').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.path_pub = self.create_publisher(Path, 'planned_route', latched)
        self.marker_pub = self.create_publisher(MarkerArray, 'route_markers', latched)

        self.sub = self.create_subscription(
            PoseArray, 'gps_waypoints', self.on_waypoints, latched)

        self.get_logger().info(
            f'Tangent route planner (Kesisim odakli U Donus) hazir. (r={self.radius} m, step={self.step})')

    def on_waypoints(self, msg: PoseArray):
        if len(msg.poses) < 3:
            self.get_logger().error(f'Beklenen En az 3 nokta (Start, End, Buoy), gelen {len(msg.poses)}. Yoksayildi.')
            return

        start = (msg.poses[0].position.x, msg.poses[0].position.y)
        end = (msg.poses[1].position.x, msg.poses[1].position.y)
        buoy = (msg.poses[2].position.x, msg.poses[2].position.y)

        self.get_logger().info(f'Noktalar: S={start}, E={end}, B={buoy}. Rota hesaplaniyor...')

        try:
            pts = self.build_route(start, end, buoy)
        except ValueError as exc:
            self.get_logger().error(f'Rota hesaplanamadi: {exc}')
            return

        self.publish_path(pts)
        self.publish_markers(start, end, buoy, pts)

        total = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                    for i in range(len(pts) - 1))
        self.get_logger().info(f'Rota yayinlandi: {len(pts)} waypoint, toplam {total:.2f} m.')

    def build_route(self, start, end, buoy):
        r = self.radius
        log = self.get_logger()

        # Start noktasindan End'in ZİT yonunde kalan buoya dogru teget secimi
        t_start = pick_tangent_opposite(start, buoy, r, end, log, 'Baslangic tegeti')
        
        # End noktasindan Start'in ZİT yonunde kalan buoya dogru teget secimi
        t_end = pick_tangent_opposite(end, buoy, r, start, log, 'Bitis tegeti')

        # Teget dogrularinin kesisimi (origin->t_start ve origin->t_end teget dogrulari)
        intersect = line_intersection(start, t_start, end, t_end)
        
        if intersect is None:
            log.warn('Teget cizgileri paralel, kesisim bulunamadi. Samandiranin arkasinda ortalama bir u-donus noktasi belirleniyor.')
            # Alternatif kesisim noktasi: samandiranin arkasinda 'r' kadar uzaklikta
            mid_angle = math.atan2(end[1]-start[1], end[0]-start[0])
            # Baslangictan bitise dogru yonelimin samandiradaki dikey ekseni
            intersect = (buoy[0] + r * math.cos(mid_angle + math.pi/2), 
                         buoy[1] + r * math.sin(mid_angle + math.pi/2))

        # Marker gosterimi icin hafizada tutalim
        self.intersect_point = intersect
        self.t_start = t_start
        self.t_end = t_end

        pts = [start]
        pts += sample_segment(start, intersect, self.step)
        pts += sample_segment(intersect, end, self.step)

        return pts

    def publish_path(self, pts):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(pts):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            if i < len(pts) - 1:
                yaw = math.atan2(pts[i + 1][1] - y, pts[i + 1][0] - x)
            else:
                yaw = math.atan2(y - pts[i - 1][1], x - pts[i - 1][0])
            qx, qy, qz, qw = yaw_to_quat(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        self.path_pub.publish(path)

    def publish_markers(self, start, end, buoy, pts):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def base(mid, mtype):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = stamp
            m.ns = 'route_tangent'
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        # Samandira
        m = base(0, Marker.CYLINDER)
        m.pose.position.x, m.pose.position.y = buoy
        m.scale.x = m.scale.y = self.radius * 2.0
        m.scale.z = 0.5
        m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
        m.color.a = 0.3 # Saydam kirmizi
        ma.markers.append(m)

        # Aracin gidecegi yol
        m = base(2, Marker.LINE_STRIP)
        m.scale.x = 0.06
        m.color.g, m.color.a = 1.0, 1.0
        for x, y in pts:
            m.points.append(Point(x=x, y=y))
        ma.markers.append(m)

        # Baslangic ve Bitis noktalari
        for mid, (x, y), (cr, cg, cb) in ((3, start, (0.0, 0.3, 1.0)),
                                          (4, end, (1.0, 0.9, 0.0))):
            m = base(mid, Marker.SPHERE)
            m.pose.position.x, m.pose.position.y = x, y
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = cr, cg, cb, 1.0
            ma.markers.append(m)

        # Kesisim Noktasi (Donus yapilacak yer)
        if hasattr(self, 'intersect_point'):
            m = base(5, Marker.SPHERE)
            m.pose.position.x, m.pose.position.y = self.intersect_point
            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 1.0 # Mor
            ma.markers.append(m)
            
            # Hayali teget cizgileri (Start -> Intersect)
            m_line1 = base(6, Marker.LINE_STRIP)
            m_line1.scale.x = 0.02
            m_line1.color.r, m_line1.color.g, m_line1.color.b, m_line1.color.a = 0.5, 0.0, 0.5, 0.8
            m_line1.points.append(Point(x=start[0], y=start[1]))
            m_line1.points.append(Point(x=self.intersect_point[0], y=self.intersect_point[1]))
            ma.markers.append(m_line1)
            
            # Hayali teget cizgileri (Intersect -> End)
            m_line2 = base(7, Marker.LINE_STRIP)
            m_line2.scale.x = 0.02
            m_line2.color.r, m_line2.color.g, m_line2.color.b, m_line2.color.a = 0.5, 0.0, 0.5, 0.8
            m_line2.points.append(Point(x=self.intersect_point[0], y=self.intersect_point[1]))
            m_line2.points.append(Point(x=end[0], y=end[1]))
            ma.markers.append(m_line2)

        self.marker_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = TangentRoutePlannerNode()
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
