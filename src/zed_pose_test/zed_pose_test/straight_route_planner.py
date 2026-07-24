#!/usr/bin/env python3
"""
Duz Rota Planlayici (Straight Route Planner)

Aracin bulundugu konumdan (0,0) baslayip, baktigi yone (X ekseni)
dogru dumduz bir rota olusturur ve yayinlar.
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


class StraightRoutePlannerNode(Node):

    def __init__(self):
        super().__init__('straight_route_planner')

        self.declare_parameter('length', 10.0)
        self.declare_parameter('step', 0.5)
        self.declare_parameter('frame_id', 'map')

        self.length = float(self.get_parameter('length').value)
        self.step = float(self.get_parameter('step').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.path_pub = self.create_publisher(Path, 'planned_route', latched)
        self.marker_pub = self.create_publisher(MarkerArray, 'route_markers', latched)

        self.get_logger().info(
            f'Duz Rota Planlayici basladi. '
            f'(Uzunluk={self.length} m, step={self.step} m, frame={self.frame_id})')

        self.publish_route()

    def publish_route(self):
        pts = []
        n_points = max(2, int(math.ceil(self.length / self.step)))
        for i in range(n_points + 1):
            x = float(i * self.step)
            y = 0.0
            pts.append((x, y))

        self.publish_path(pts)
        self.publish_markers(pts)

        self.get_logger().info(f'Dumduz rota yayinlandi: {len(pts)} nokta, toplam {self.length} m.')

    def publish_path(self, pts):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(pts):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            # Rota boyunca dumduz ileri (yaw = 0) bakacak
            qx, qy, qz, qw = yaw_to_quat(0.0)
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
        
        # Bitis noktasi (kirmizi kure)
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
