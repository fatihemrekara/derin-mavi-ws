#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rota Takip / Hareket Node'u — SIRALI WAYPOINT TAKIBI + Pure Pursuit
-------------------------------------------------------------------
Girisler:
  - /planned_route     (nav_msgs/Path)          : planlanan rota
  - /robot/filtered_pose (geometry_msgs/PoseStamped) : EKF'den anlik poz
Cikis:
  - /cmd_vel           (geometry_msgs/Twist)
  - /follower_markers  (visualization_msgs/MarkerArray)

TAKIP MANTIGI (onceki surumden temel fark):
  Rotada "en yakin nokta" ARANMAZ. Cunku samandira yayi 180 dereceyi
  asinca rota kurdele gibi kendini keser ve mesafe tabanli arama,
  kesisimde rotanin ileriki bacagina atlayip turu pas gecirir.

  Bunun yerine SIRALI ilerleme kullanilir:
    - Arac her an tek bir "siradaki waypoint"i (indeks wi) kovalar.
    - wi SADECE su iki durumdan biri gerceklesince 1 artar:
        a) araca wp_pass_radius'tan yakin (nokta "alindi"), veya
        b) aracin, noktanin yerel yol yonune dik gecis duzlemini
           asmis olmasi (nokta "gecildi", icten kesse bile).
    - Baska hicbir noktayla mesafe kiyaslamasi yapilmadigi icin
      rotanin baska bir bacagi ne kadar yakin gecerse gecsin
      indeks oraya sicrayamaz. Atlama yapisal olarak imkansizdir.

  Direksiyon yine pure pursuit: hedef nokta, siradaki waypoint'ten
  itibaren rota boyunca 'lookahead' kadar ileride secilir. Arac
  siradaki waypoint'e lookahead'den uzaksa dogrudan o waypoint
  hedeflenir (rotaya geri donus).

Parametreler (oncekilere ek):
  wp_pass_radius (default: 0.8 m) : waypoint "alindi" sayilma yaricapi.
                                    Rota adim araliginin ~1.5 kati iyi.
Diger parametreler onceki surumle ayni.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import OverrideRCIn
from visualization_msgs.msg import Marker, MarkerArray


def quat_to_yaw(ox, oy, oz, ow):
    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class PathFollowerNode(Node):

    def __init__(self):
        super().__init__('path_follower')

        self.declare_parameter('path_topic', '/planned_route')
        self.declare_parameter('pose_topic', '/robot/filtered_pose')
        self.declare_parameter('rc_override_topic', '/mavros/rc/override')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('lookahead', 1.2)
        self.declare_parameter('wp_pass_radius', 0.8)
        self.declare_parameter('v_max', 0.8)
        self.declare_parameter('v_min', 0.15)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('slow_radius', 1.5)
        self.declare_parameter('rotate_in_place_deg', 70.0)
        self.declare_parameter('k_heading', 1.8)
        self.declare_parameter('pose_timeout', 0.6)
        self.declare_parameter('initial_yaw_offset_deg', 0.0)

        gp = lambda n: self.get_parameter(n).value
        self.lookahead = float(gp('lookahead'))
        self.pass_r = float(gp('wp_pass_radius'))
        self.v_max = float(gp('v_max'))
        self.v_min = float(gp('v_min'))
        self.w_max = float(gp('w_max'))
        self.goal_tol = float(gp('goal_tolerance'))
        self.slow_radius = float(gp('slow_radius'))
        self.rotate_thresh = math.radians(float(gp('rotate_in_place_deg')))
        self.k_heading = float(gp('k_heading'))
        self.pose_timeout = float(gp('pose_timeout'))
        self.yaw_offset = math.radians(float(gp('initial_yaw_offset_deg')))
        rate = float(gp('control_rate_hz'))

        self.path_pts = []
        self.cum = []
        self.path_frame = 'map'
        self.wp_idx = 0            # SIRADAKI waypoint (sadece gecilince artar)
        self.pose = None
        self.last_pose_time = None
        self.goal_reached = False
        self.startup_aligned = False
        self.align_thresh = math.radians(5.0)
        
        self._warned_no_path = False
        self._warned_no_pose = False
        self._warned_stale = False

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.path_sub = self.create_subscription(
            Path, str(gp('path_topic')), self.on_path, latched)
        self.pose_sub = self.create_subscription(
            PoseStamped, str(gp('pose_topic')), self.on_pose, sensor_qos)

        self.rc_pub = self.create_publisher(OverrideRCIn, str(gp('rc_override_topic')), 10)
        self.stopped_published = False
        
        self.marker_pub = self.create_publisher(MarkerArray, 'follower_markers', 10)

        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f'Path follower (SIRALI takip) hazir. lookahead={self.lookahead} m, '
            f'wp_pass_radius={self.pass_r} m, v_max={self.v_max} m/s, '
            f'goal_tol={self.goal_tol} m')

    # ================= callback'ler =================

    def on_path(self, msg: Path):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if len(pts) < 2:
            self.get_logger().info('Bos rota alindi, mevcut rota siliniyor ve arac durdurulacak (Geri kontrol videotask\'te).')
            self.path_pts = []
            self.stop()
            return
        self.path_pts = pts
        cum = [0.0]
        for i in range(1, len(pts)):
            cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                            pts[i][1] - pts[i - 1][1]))
        self.cum = cum
        self.path_frame = msg.header.frame_id or 'map'
        self.wp_idx = 0
        self.goal_reached = False
        self.startup_aligned = False
        self._warned_no_path = False
        self.stopped_published = False
        self.get_logger().info(
            f'Yeni rota alindi: {len(pts)} waypoint, {cum[-1]:.1f} m '
            f'(frame: {self.path_frame}). Sirali takip basliyor.')

    def on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        yaw = normalize_angle(quat_to_yaw(o.x, o.y, o.z, o.w) + self.yaw_offset)
        if abs(self.yaw_offset) > 1e-9:
            c, s = math.cos(self.yaw_offset), math.sin(self.yaw_offset)
            x = c * p.x - s * p.y
            y = s * p.x + c * p.y
        else:
            x, y = p.x, p.y
        self.pose = (x, y, yaw)
        self.last_pose_time = self.get_clock().now()

    # ================= kontrol dongusu =================

    def control_loop(self):
        if not self.path_pts:
            if not self._warned_no_path:
                self.get_logger().warn('Rota bekleniyor (/planned_route)...')
                self._warned_no_path = True
            return

        if self.pose is None:
            if not self._warned_no_pose:
                self.get_logger().warn('SLAM pozu bekleniyor (pose_topic)...')
                self._warned_no_pose = True
            return
        self._warned_no_pose = False

        age = (self.get_clock().now() - self.last_pose_time).nanoseconds * 1e-9
        if age > self.pose_timeout:
            if not self._warned_stale:
                self.get_logger().warn(
                    f'Poz verisi {age:.2f} s eski! Arac durduruluyor.')
                self._warned_stale = True
            self.stop()
            return
        self._warned_stale = False

        if self.goal_reached:
            self.stop()
            return

        x, y, yaw = self.pose

        # ---- SIRALI ilerleme: siradaki waypoint gecildiyse indeksi artir ----
        self.advance_waypoint(x, y)

        goal = self.path_pts[-1]
        dist_goal = math.hypot(goal[0] - x, goal[1] - y)

        # ---- hedefe varildi mi? (son waypoint'e siramiz gelmis olmali) ----
        if self.wp_idx >= len(self.path_pts) - 1 and dist_goal <= self.goal_tol:
            self.goal_reached = True
            self.stop()
            self.get_logger().info(
                f'HEDEFE ULASILDI! Kalan mesafe: {dist_goal:.2f} m. Arac durduruldu.')
            return

        # ---- hedef (lookahead) noktasini sec ----
        target = self.find_target(x, y)

        # ---- kontrol hesabi (pure pursuit) ----
        dx, dy = target[0] - x, target[1] - y
        target_dist = math.hypot(dx, dy)
        heading_err = normalize_angle(math.atan2(dy, dx) - yaw)

        # Başlangıçta rotaya tam hizalanma (Açıyı döndürüp sonra yola çıkma)
        if not self.startup_aligned:
            if abs(heading_err) > self.align_thresh:
                rc_msg = OverrideRCIn()
                rc_msg.channels = [65535] * 18
                rc_msg.channels[4] = 1500  # Forward neutral
                
                w = max(-self.w_max, min(self.w_max, self.k_heading * heading_err))
                pwm_yaw = int(1500 + (w / self.w_max) * 400) if self.w_max > 0 else 1500
                rc_msg.channels[3] = max(1100, min(1900, pwm_yaw))
                
                self.rc_pub.publish(rc_msg)
                self.publish_markers(target)
                self.get_logger().info(
                    f'Baslangic hizalamasi: Hata = {math.degrees(heading_err):.1f} derece',
                    throttle_duration_sec=0.5)
                return
            else:
                self.get_logger().info('Baslangic hizalanmasi tamamlandi! Ileri hareket basliyor.')
                self.startup_aligned = True

        rc_msg = OverrideRCIn()
        rc_msg.channels = [65535] * 18

        if abs(heading_err) > self.rotate_thresh:
            v = 0.0
            w = max(-self.w_max, min(self.w_max, self.k_heading * heading_err))
        else:
            v = self.v_max * max(0.0, math.cos(heading_err))
            if dist_goal < self.slow_radius:
                v *= max(dist_goal / self.slow_radius, 0.2)
            v = max(self.v_min, min(self.v_max, v))

            L = max(target_dist, 1e-3)
            w = 2.0 * v * math.sin(heading_err) / L
            if abs(w) > self.w_max:
                scale = self.w_max / abs(w)
                w *= scale
                v = max(self.v_min * 0.5, v * scale)

        pwm_fwd = int(1500 + (v / self.v_max) * 400) if self.v_max > 0 else 1500
        pwm_yaw = int(1500 + (w / self.w_max) * 400) if self.w_max > 0 else 1500

        rc_msg.channels[4] = max(1100, min(1900, pwm_fwd))
        rc_msg.channels[3] = max(1100, min(1900, pwm_yaw))

        self.rc_pub.publish(rc_msg)
        self.publish_markers(target)

        self.get_logger().info(
            f'PWM FWD={rc_msg.channels[4]} YAW={rc_msg.channels[3]}  '
            f'aci_hata={math.degrees(heading_err):+6.1f}deg  '
            f'wp={self.wp_idx}/{len(self.path_pts) - 1}  '
            f'hedefe={dist_goal:6.2f} m',
            throttle_duration_sec=0.5)

    # ================= yardimcilar =================

    def advance_waypoint(self, x, y):
        """Siradaki waypoint'i SADECE gecildiyse ilerletir."""
        pts = self.path_pts
        while self.wp_idx < len(pts) - 1:
            px, py = pts[self.wp_idx]
            
            # 1. Mesafe Kosulu (Noktanin icine girildi mi?)
            if math.hypot(px - x, py - y) <= self.pass_r:
                self.wp_idx += 1
                continue
            
            # 2. Gecis Duzlemi Kosulu (Noktanin hizasi asildi mi?)
            # Vektor, GIDIS yonune (nx, ny) degil, GELIS yonune gore alinmali!
            if self.wp_idx > 0:
                prev_x, prev_y = pts[self.wp_idx - 1]
                dirx = px - prev_x
                diry = py - prev_y
            else:
                # Eger ilk waypoint'teysek, mecburen bir sonrakini referans aliriz
                nx, ny = pts[self.wp_idx + 1]
                dirx = nx - px
                diry = ny - py
                
            # Robotun, noktanin dik duzlemini gecip gecmedigi kontrolu
            if (x - px) * dirx + (y - py) * diry > 0.0:
                self.wp_idx += 1
                continue
                
            break

    def find_target(self, x, y):
        """Gerçek Pure Pursuit Hedef Noktasi (Lookahead Çemberi Kesişimi).

        1. Aracın rotaya olan en yakın noktasını (local izdüşüm) bulur.
        2. Bu noktadan ileriye doğru bakarak, araca tam `lookahead` mesafesinde
           olan rotadaki kesişim noktasını (havucu) hedefler.
        """
        pts = self.path_pts
        if not pts:
            return (x, y)
        if len(pts) == 1:
            return pts[0]

        wi = self.wp_idx
        
        # 1. Local Search: Rotaya olan en yakin noktayi bul.
        # Tum rotayi aramak yerine sadece wi'den baslayarak belirli bir pencere
        # icinde arariz. Boylece rota kendini kestiginde (loop) yanlis yola atlamayiz.
        search_window = int(self.lookahead / 0.5) * 4 # Yaklasik 4x lookahead mesafesi kadar ileri bak
        end_idx = min(len(pts), wi + search_window + 2)
        
        min_dist = float('inf')
        closest_idx = wi
        for i in range(wi, end_idx):
            d = math.hypot(pts[i][0] - x, pts[i][1] - y)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        # 2. Eger rotadan lookahead'den daha uzaksak (cember rotayi kesmiyorsa),
        # dogrudan en yakin noktayi hedefle ki araba yola hizlica donsun.
        if min_dist >= self.lookahead:
            return pts[closest_idx]

        # 3. Lookahead cemberinin rotayi kestigi noktayi bul
        # closest_idx'den baslayarak ileriye dogru ilk lookahead disi noktayi ara
        for i in range(closest_idx + 1, len(pts)):
            d = math.hypot(pts[i][0] - x, pts[i][1] - y)
            if d >= self.lookahead:
                # pts[i-1] (iceride) ile pts[i] (disarida) arasinda cember kesisimi interpolasyonu
                prev_d = math.hypot(pts[i-1][0] - x, pts[i-1][1] - y)
                # Basit lineer oran (cember yayina cok yakin bir yaklasik deger)
                ratio = (self.lookahead - prev_d) / (d - prev_d) if (d - prev_d) > 1e-5 else 0.0
                tx = pts[i-1][0] + ratio * (pts[i][0] - pts[i-1][0])
                ty = pts[i-1][1] + ratio * (pts[i][1] - pts[i-1][1])
                return (tx, ty)
                
        # Eger rotanin sonuna ulasildiysa ve son nokta hala cemberin icindeyse
        return pts[-1]

    def stop(self):
        if not self.stopped_published:
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[3] = 1500  # Yaw neutral
            rc_msg.channels[4] = 1500  # Forward neutral
            self.rc_pub.publish(rc_msg)
            self.stopped_published = True

    def publish_markers(self, target):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        m = Marker()
        m.header.frame_id = self.path_frame
        m.header.stamp = stamp
        m.ns = 'follower'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y = target
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 1.0
        ma.markers.append(m)

        # siradaki waypoint (turuncu kucuk kure): atlama olup olmadigi
        # RViz'de ciplak gozle izlenebilsin
        wp = self.path_pts[self.wp_idx]
        m2 = Marker()
        m2.header = m.header
        m2.ns = 'follower'
        m2.id = 1
        m2.type = Marker.SPHERE
        m2.action = Marker.ADD
        m2.pose.position.x, m2.pose.position.y = wp
        m2.pose.orientation.w = 1.0
        m2.scale.x = m2.scale.y = m2.scale.z = 0.18
        m2.color.r, m2.color.g, m2.color.a = 1.0, 0.5, 1.0
        ma.markers.append(m2)

        if self.pose is not None:
            m3 = Marker()
            m3.header = m.header
            m3.ns = 'follower'
            m3.id = 2
            m3.type = Marker.LINE_STRIP
            m3.action = Marker.ADD
            m3.pose.orientation.w = 1.0
            m3.scale.x = 0.03
            m3.color.r, m3.color.g, m3.color.b, m3.color.a = 1.0, 0.5, 0.0, 1.0
            m3.points.append(Point(x=self.pose[0], y=self.pose[1]))
            m3.points.append(Point(x=target[0], y=target[1]))
            ma.markers.append(m3)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()