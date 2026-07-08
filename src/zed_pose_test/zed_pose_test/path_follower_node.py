"""

Node hazır. Kısaca nasıl çalıştığı:
Girişler ve çıkışlar

/planned_route (Path) rota planlayıcıdan latched QoS ile alınır (node geç açılsa bile rotayı yakalar), ZED pozu /zed/zed_node/pose'dan gelir.
Çıkış /cmd_vel (Twist): linear.x ileri hız, angular.z dönüş hızı. Motor sürücü node'un bu topic'i dinlemeli.

Kontrol mantığı (Pure Pursuit)

Rotada araca en yakın nokta bulunur (geriye sıçramayı önlemek için ileri yönde sınırlı pencerede aranır — 240-250 noktalık rotada verimli).
Oradan lookahead (varsayılan 1.2 m) kadar ileride bir hedef nokta seçilir ve w = 2·v·sin(α)/L eğriliğiyle dönüş hızı hesaplanır.
Açı hatası 70°'den büyükse yerinde dönüş yapar; hedefe slow_radius içinde yavaşlar, goal_tolerance (0.35 m) içine girince durur.
Güvenlik: poz verisi 0.6 s'den eskirse veya Ctrl+C ile kapatılırsa araç sıfır hız komutuyla durdurulur.
RViz için /follower_markers'da lookahead noktasını ve hedef çizgisini gösterir.

Dikkat etmen gereken kritik nokta — frame uyumu: Rotanız ENU tabanlı map frame'inde (x=Doğu, y=Kuzey, başlangıç GPS noktası origin), ZED ise açıldığı anı origin ve baktığı yönü +x kabul ediyor. İki sistemin örtüşmesi için aracı başlangıç GPS noktasında başlatmalısın ve araç doğuya bakmıyorsa aradaki açıyı initial_yaw_offset_deg parametresiyle vermelisin (örn. araç kuzeye bakıyorsa 90). Node bu offset'le hem yaw'ı hem pozisyonu döndürür. Uzun vadede en temizi bu dönüşümü bir TF static transform ile yayınlamak olur; istersen onu da ekleyebilirim.
Ayarlanabilir parametreler: lookahead, v_max, v_min, w_max, goal_tolerance, slow_radius, rotate_in_place_deg, pose_timeout, control_rate_hz. Araç bir deniz aracıysa (samandıra dolayısıyla tahmin ediyorum) ve diferansiyel/thruster sürüşü yerine dümen açısıyla kontrol ediliyorsa, çıkışı ona göre uyarlayabilirim — aracın tahrik tipini söylersen düzenlerim.

"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rota Takip / Hareket Node'u (Pure Pursuit)
------------------------------------------
Girisler:
  - /planned_route  (nav_msgs/Path)        : route_planner_node'un yayinladigi rota
  - /zed/zed_node/pose (geometry_msgs/PoseStamped) : SLAM'den anlik poz

Cikis:
  - /cmd_vel        (geometry_msgs/Twist)  : dogrusal + acisal hiz komutu
  - /follower_markers (visualization_msgs/MarkerArray) : lookahead noktasi (RViz)

Calisma mantigi (Pure Pursuit):
  1. Rotada araca en yakin nokta bulunur (geri gitmemek icin ilerleme
     monotonik tutulur).
  2. O noktadan itibaren rota boyunca 'lookahead' mesafesi kadar ileride
     bir hedef nokta secilir.
  3. Hedefe olan aci hatasina gore acisal hiz, hataya ve hedefe yakinliga
     gore dogrusal hiz hesaplanir.
  4. Son noktaya 'goal_tolerance' kadar yaklasilinca durulur.

Guvenlik:
  - Poz verisi 'pose_timeout' saniyeden eski ise arac durdurulur.
  - Aci hatasi cok buyukse (arac rotaya ters bakiyorsa) yerinde donus yapilir.

ONEMLI VARSAYIM:
  Rota 'map' frame'inde, ZED pozu ise kameranin acildigi ani origin kabul
  eden kendi frame'inde gelir. Bu iki frame'in cakismasi icin arac,
  BASLANGIC GPS noktasinda ve rotanin ilk yonune yakin bakacak sekilde
  baslatilmalidir (ya da aradaki donusum TF ile saglanmalidir).
  ZED'in x ekseni ileriyi gosterir; ENU'da x=Dogu'dur. Arac dogudan farkli
  bir yone bakarak baslarsa 'initial_yaw_offset_deg' parametresi ile
  duzeltme yapilabilir.

Parametreler:
  path_topic            (default: /planned_route)
  pose_topic            (default: /zed/zed_node/pose)
  cmd_vel_topic         (default: /cmd_vel)
  control_rate_hz       (default: 20.0)   kontrol dongusu frekansi
  lookahead             (default: 1.2 m)  hedef nokta uzakligi
  v_max                 (default: 0.8 m/s) maksimum dogrusal hiz
  v_min                 (default: 0.15 m/s) hareket halindeki minimum hiz
  w_max                 (default: 1.0 rad/s) maksimum acisal hiz
  goal_tolerance        (default: 0.35 m) hedefe varis toleransi
  slow_radius           (default: 1.5 m)  hedefe yaklasirken yavaslama yaricapi
  rotate_in_place_deg   (default: 70.0)   bu acidan buyuk hata -> yerinde don
  k_heading             (default: 1.8)    yerinde donus kazanci
  pose_timeout          (default: 0.6 s)  poz verisi eskirse dur
  initial_yaw_offset_deg(default: 0.0)    ZED frame -> map frame yaw duzeltmesi
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray


def quat_to_yaw(ox, oy, oz, ow):
    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    """Aciyi (-pi, pi] araligina indirger."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class PathFollowerNode(Node):

    def __init__(self):
        super().__init__('path_follower')

        # ---------------- parametreler ----------------
        self.declare_parameter('path_topic', '/planned_route')
        self.declare_parameter('pose_topic', '/zed/zed_node/pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('lookahead', 1.2)
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

        # ---------------- durum ----------------
        self.path_pts = []          # [(x, y), ...]
        self.path_frame = 'map'
        self.nearest_idx = 0        # rotadaki ilerleme (monotonik)
        self.pose = None            # (x, y, yaw)
        self.last_pose_time = None
        self.goal_reached = False
        self._warned_no_path = False
        self._warned_no_pose = False
        self._warned_stale = False

        # ---------------- QoS / iletisim ----------------
        # Rota latched (transient_local) yayinlaniyor; ayni sekilde dinle
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.path_sub = self.create_subscription(
            Path, str(gp('path_topic')), self.on_path, latched)
        self.pose_sub = self.create_subscription(
            PoseStamped, str(gp('pose_topic')), self.on_pose, 10)

        self.cmd_pub = self.create_publisher(Twist, str(gp('cmd_vel_topic')), 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'follower_markers', 10)

        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f'Path follower hazir. lookahead={self.lookahead} m, '
            f'v_max={self.v_max} m/s, w_max={self.w_max} rad/s, '
            f'goal_tol={self.goal_tol} m')

    # ================= callback'ler =================

    def on_path(self, msg: Path):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if len(pts) < 2:
            self.get_logger().error(f'Gecersiz rota: {len(pts)} nokta. Yoksayildi.')
            return
        self.path_pts = pts
        self.path_frame = msg.header.frame_id or 'map'
        self.nearest_idx = 0
        self.goal_reached = False
        self._warned_no_path = False
        self.get_logger().info(
            f'Yeni rota alindi: {len(pts)} waypoint '
            f'(frame: {self.path_frame}). Takip basliyor.')

    def on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        yaw = normalize_angle(quat_to_yaw(o.x, o.y, o.z, o.w) + self.yaw_offset)
        # yaw_offset varsa pozisyonu da map frame'ine dondur
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
        # Rota yoksa bekle
        if not self.path_pts:
            if not self._warned_no_path:
                self.get_logger().warn('Rota bekleniyor (/planned_route)...')
                self._warned_no_path = True
            return

        # Poz yoksa bekle
        if self.pose is None:
            if not self._warned_no_pose:
                self.get_logger().warn('SLAM pozu bekleniyor (pose_topic)...')
                self._warned_no_pose = True
            return
        self._warned_no_pose = False

        # Poz verisi bayat ise guvenlik icin dur
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
        goal = self.path_pts[-1]
        dist_goal = math.hypot(goal[0] - x, goal[1] - y)

        # ---- hedefe varildi mi? ----
        if dist_goal <= self.goal_tol:
            self.goal_reached = True
            self.stop()
            self.get_logger().info(
                f'HEDEFE ULASILDI! Kalan mesafe: {dist_goal:.2f} m. Arac durduruldu.')
            return

        # ---- en yakin noktayi guncelle (monotonik ilerleme) ----
        self.update_nearest(x, y)

        # ---- lookahead hedef noktasini sec ----
        target = self.find_lookahead(x, y, dist_goal)

        # ---- kontrol hesabi ----
        dx, dy = target[0] - x, target[1] - y
        target_dist = math.hypot(dx, dy)
        heading_err = normalize_angle(math.atan2(dy, dx) - yaw)

        cmd = Twist()
        if abs(heading_err) > self.rotate_thresh:
            # Rotaya ters bakiyoruz: yerinde don
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.w_max,
                                min(self.w_max, self.k_heading * heading_err))
        else:
            # Pure pursuit egriligi: w = 2 * v * sin(alpha) / L
            # Hiz: aci hatasi buyudukce ve hedefe yaklastikca dusur
            v = self.v_max * max(0.0, math.cos(heading_err))
            if dist_goal < self.slow_radius:
                v *= max(dist_goal / self.slow_radius, 0.2)
            v = max(self.v_min, min(self.v_max, v))

            L = max(target_dist, 1e-3)
            w = 2.0 * v * math.sin(heading_err) / L
            if abs(w) > self.w_max:
                # Acisal limit asildiysa hizi orantili dusur ki egrilik korunsun
                scale = self.w_max / abs(w)
                w *= scale
                v = max(self.v_min * 0.5, v * scale)

            cmd.linear.x = v
            cmd.angular.z = w

        self.cmd_pub.publish(cmd)
        self.publish_markers(target)

        self.get_logger().info(
            f'v={cmd.linear.x:+.2f} m/s  w={cmd.angular.z:+.2f} rad/s  '
            f'aci_hata={math.degrees(heading_err):+6.1f}deg  '
            f'wp={self.nearest_idx}/{len(self.path_pts)-1}  '
            f'hedefe={dist_goal:6.2f} m',
            throttle_duration_sec=0.5)

    # ================= yardimcilar =================

    def update_nearest(self, x, y):
        """Aracin rotadaki en yakin noktasini bulur.
        Geriye sicramamak icin sadece mevcut indeksten ileriye,
        sinirli bir pencere icinde arar."""
        best_i = self.nearest_idx
        best_d = float('inf')
        window_end = min(len(self.path_pts), self.nearest_idx + 40)
        for i in range(self.nearest_idx, window_end):
            px, py = self.path_pts[i]
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d = d
                best_i = i
        self.nearest_idx = best_i

    def find_lookahead(self, x, y, dist_goal):
        """En yakin noktadan itibaren rota boyunca lookahead mesafesi
        kadar ileride bir hedef nokta secer."""
        # Hedefe lookahead'den yakinsak dogrudan son noktaya git
        if dist_goal <= self.lookahead:
            return self.path_pts[-1]

        acc = 0.0
        i = self.nearest_idx
        while i < len(self.path_pts) - 1:
            ax, ay = self.path_pts[i]
            bx, by = self.path_pts[i + 1]
            seg = math.hypot(bx - ax, by - ay)
            if acc + seg >= self.lookahead:
                # Segment icinde enterpolasyon
                t = (self.lookahead - acc) / seg if seg > 1e-9 else 0.0
                return (ax + (bx - ax) * t, ay + (by - ay) * t)
            acc += seg
            i += 1
        return self.path_pts[-1]

    def stop(self):
        self.cmd_pub.publish(Twist())  # tum alanlar 0.0

    def publish_markers(self, target):
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = self.path_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'follower'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y = target
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 1.0
        ma.markers.append(m)

        if self.pose is not None:
            m2 = Marker()
            m2.header = m.header
            m2.ns = 'follower'
            m2.id = 1
            m2.type = Marker.LINE_STRIP
            m2.action = Marker.ADD
            m2.pose.orientation.w = 1.0
            m2.scale.x = 0.03
            m2.color.r, m2.color.g, m2.color.b, m2.color.a = 1.0, 0.5, 0.0, 1.0
            m2.points.append(Point(x=self.pose[0], y=self.pose[1]))
            m2.points.append(Point(x=target[0], y=target[1]))
            ma.markers.append(m2)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cikarken araci guvenli sekilde durdur
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
