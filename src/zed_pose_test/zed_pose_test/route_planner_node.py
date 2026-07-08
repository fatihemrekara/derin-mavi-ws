#!/usr/bin/env python3
"""
Samandira etrafinda donus rotasi planlayici (Subscriber versiyonu)

/gps_waypoints (geometry_msgs/PoseArray) topic'inden [baslangic, bitis, samandira]
sirasindaki 3 noktayi dinler. Veri gelince rotayi hesaplar ve yayinlar:
  - /planned_route (nav_msgs/Path)
  - /route_markers (visualization_msgs/MarkerArray)

Parametreler:
  radius   : samandira etrafindaki hayali cember yaricapi (m, default 2.0)
  step     : rotadaki waypoint araligi (m, default 0.5)
  frame_id : SLAM/harita frame'i (default 'map')
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, PoseArray
from visualization_msgs.msg import Marker, MarkerArray


# ------------------------------------------------------------------ geometri

def cross2(ax, ay, bx, by):
    return ax * by - ay * bx


def tangent_points(p, c, r):
    dx, dy = p[0] - c[0], p[1] - c[1]
    d = math.hypot(dx, dy)
    if d <= r:
        raise ValueError(
            f'Nokta ({p[0]:.2f}, {p[1]:.2f}) cemberin icinde/uzerinde '
            f'(uzaklik {d:.2f} m <= r={r:.2f} m). Teget cizilemez.')
    base = math.atan2(dy, dx)
    alpha = math.acos(r / d)
    t1 = (c[0] + r * math.cos(base + alpha), c[1] + r * math.sin(base + alpha))
    t2 = (c[0] + r * math.cos(base - alpha), c[1] + r * math.sin(base - alpha))
    return t1, t2


def pick_tangent(p, c, r, ref, logger, label):
    t1, t2 = tangent_points(p, c, r)
    v_pc = (c[0] - p[0], c[1] - p[1])
    side_ref = cross2(v_pc[0], v_pc[1], ref[0] - p[0], ref[1] - p[1])
    side_t1 = cross2(v_pc[0], v_pc[1], t1[0] - p[0], t1[1] - p[1])
    if abs(side_ref) < 1e-9:
        logger.warn(f'{label}: referans nokta merkez dogrusu uzerinde, '
                    f't1 secildi.')
        return t1
    return t1 if side_ref * side_t1 > 0 else t2


def sample_segment(a, b, step):
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(math.ceil(dist / step)))
    return [(a[0] + (b[0] - a[0]) * i / n,
             a[1] + (b[1] - a[1]) * i / n) for i in range(1, n + 1)]


def sample_arc(c, r, a0, sweep, step):
    arc_len = abs(sweep) * r
    n = max(2, int(math.ceil(arc_len / step)))
    return [(c[0] + r * math.cos(a0 + sweep * i / n),
             c[1] + r * math.sin(a0 + sweep * i / n)) for i in range(1, n + 1)]


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# ------------------------------------------------------------------ node

class RoutePlannerNode(Node):

    def __init__(self):
        super().__init__('buoy_route_planner')

        self.declare_parameter('radius', 2.0)
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
            f'Route planner hazir. /gps_waypoints topic\'i bekleniyor. '
            f'(r={self.radius} m, step={self.step} m, frame={self.frame_id})')

    def on_waypoints(self, msg: PoseArray):
        if len(msg.poses) < 3:
            self.get_logger().error(
                f'Beklenen 3 nokta, gelen {len(msg.poses)}. Yoksayildi.')
            return

        start = (msg.poses[0].position.x, msg.poses[0].position.y)
        end = (msg.poses[1].position.x, msg.poses[1].position.y)
        buoy = (msg.poses[2].position.x, msg.poses[2].position.y)

        self.get_logger().info(
            f'Noktalar alindi: S={start}, E={end}, B={buoy}. Rota hesaplaniyor...')

        try:
            pts = self.build_route(start, end, buoy)
        except ValueError as exc:
            self.get_logger().error(f'Rota hesaplanamadi: {exc}')
            return

        self.publish_path(pts)
        self.publish_markers(start, end, buoy, pts)

        total = sum(math.hypot(pts[i + 1][0] - pts[i][0],
                               pts[i + 1][1] - pts[i][1])
                    for i in range(len(pts) - 1))
        self.get_logger().info(
            f'Rota yayinlandi: {len(pts)} waypoint, toplam {total:.2f} m.')

    def build_route(self, start, end, buoy):
        r = self.radius
        log = self.get_logger()

        t_in = pick_tangent(start, buoy, r, end, log, 'Giris tegeti')
        t_out = pick_tangent(end, buoy, r, start, log, 'Cikis tegeti')

        u = (t_in[0] - start[0], t_in[1] - start[1])
        rad = (t_in[0] - buoy[0], t_in[1] - buoy[1])
        ccw = cross2(rad[0], rad[1], u[0], u[1]) > 0.0

        a0 = math.atan2(t_in[1] - buoy[1], t_in[0] - buoy[0])
        a1 = math.atan2(t_out[1] - buoy[1], t_out[0] - buoy[0])
        if ccw:
            sweep = (a1 - a0) % (2.0 * math.pi)
        else:
            sweep = -((a0 - a1) % (2.0 * math.pi))

        yon = 'saat yonu tersi (CCW)' if ccw else 'saat yonu (CW)'
        log.info(f'Yay: {math.degrees(abs(sweep)):.1f} derece, donus {yon}')

        pts = [start]
        pts += sample_segment(start, t_in, self.step)
        pts += sample_arc(buoy, r, a0, sweep, self.step)
        pts += sample_segment(t_out, end, self.step)
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
            m.ns = 'route'
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        m = base(0, Marker.CYLINDER)
        m.pose.position.x, m.pose.position.y = buoy
        m.scale.x = m.scale.y = 0.4
        m.scale.z = 0.8
        m.color.r, m.color.a = 1.0, 1.0
        ma.markers.append(m)

        m = base(1, Marker.LINE_STRIP)
        m.scale.x = 0.03
        m.color.r = m.color.g = m.color.b = 0.6
        m.color.a = 1.0
        for k in range(65):
            a = 2.0 * math.pi * k / 64.0
            m.points.append(Point(x=buoy[0] + self.radius * math.cos(a),
                                  y=buoy[1] + self.radius * math.sin(a)))
        ma.markers.append(m)

        m = base(2, Marker.LINE_STRIP)
        m.scale.x = 0.06
        m.color.g, m.color.a = 1.0, 1.0
        for x, y in pts:
            m.points.append(Point(x=x, y=y))
        ma.markers.append(m)

        for mid, (x, y), (cr, cg, cb) in ((3, start, (0.0, 0.3, 1.0)),
                                          (4, end, (1.0, 0.9, 0.0))):
            m = base(mid, Marker.SPHERE)
            m.pose.position.x, m.pose.position.y = x, y
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = cr, cg, cb, 1.0
            ma.markers.append(m)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = RoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
