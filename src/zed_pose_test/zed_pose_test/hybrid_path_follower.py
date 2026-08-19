#!/usr/bin/env python3
"""
Hibrit Rota Takipçisi — Pusula Heading + ZED XY Pozisyon + Cube Derinlik
=========================================================================

Sensör görev dağılımı:
  - Heading (yaw) kontrolü     → Pusula (VFR_HUD compass heading, NED→ENU)
  - XY mesafe / segment bitişi → ZED VIO pozisyonu (/zed/zed_node/pose)
  - Derinlik kontrolü          → Orange Cube (rel_alt + ALT_HOLD modu)

blind_path_follower.py'den temel farkları:
  1. FORWARD state'te segment bitişi zaman yerine ZED XY mesafe ile belirlenir
  2. İleri giderken pusula heading kapalı çevrim düzeltme yapar
  3. ZED verisi kesilirse zaman bazlı fallback'e düşer

Düz rota (straight_route_planner) veya kare rota (square_route_planner) ile çalışır.
"""

import rclpy
from rclpy.node import Node
import math
import csv
import datetime
import signal
import sys
import os
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Path
from mavros_msgs.msg import VfrHud, OverrideRCIn


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


def simplify_path(pts, heading_tol_deg=2.0):
    """
    Rotadaki noktaları makro köşelere (keskin dönüşlere) indirger.
    Düz çizgi üzerindeki ara noktaları eler.
    """
    if len(pts) < 2:
        return pts
    simplified = [pts[0]]
    current_heading = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
    for i in range(1, len(pts) - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            continue
        h = math.atan2(dy, dx)
        if abs(normalize_angle(h - current_heading)) > math.radians(heading_tol_deg):
            simplified.append(pts[i])
            current_heading = h
    simplified.append(pts[-1])
    return simplified


class HybridPathFollowerNode(Node):
    def __init__(self):
        super().__init__('hybrid_path_follower')

        # ── Parametreler ──
        self.declare_parameter('path_topic', '/planned_route')
        self.declare_parameter('rc_override_topic', '/mavros/rc/override')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('speed_mps', 0.4)        # Fallback zaman hesabı için tahmini hız
        self.declare_parameter('fwd_pwm', 1750)          # İleri PWM
        self.declare_parameter('yaw_tolerance_deg', 15.0) # Dönüş tamamlama toleransı
        self.declare_parameter('k_heading', 3.0)          # Dönüş P kazancı
        self.declare_parameter('k_heading_fwd', 1.5)      # İleri giderken heading düzeltme kazancı
        self.declare_parameter('w_max', 1.0)              # Maks yaw rate (normalize)
        self.declare_parameter('segment_tolerance_m', 0.5) # Segment bitiş mesafe toleransı
        self.declare_parameter('zed_timeout_sec', 3.0)     # ZED verisi kesilirse fallback'e geçiş süresi
        self.declare_parameter('target_depth_m', -1.0)     # Hedef derinlik
        self.declare_parameter('dive_pwm', 1200)           # Dalış itkisi
        self.declare_parameter('log_dir', '')               # Boşsa mevcut dizin

        gp = lambda n: self.get_parameter(n).value
        self.speed_mps = float(gp('speed_mps'))
        self.fwd_pwm = int(gp('fwd_pwm'))
        self.yaw_tol = math.radians(float(gp('yaw_tolerance_deg')))
        self.k_heading = float(gp('k_heading'))
        self.k_heading_fwd = float(gp('k_heading_fwd'))
        self.w_max = float(gp('w_max'))
        self.segment_tol = float(gp('segment_tolerance_m'))
        self.zed_timeout = float(gp('zed_timeout_sec'))
        self.target_depth = float(gp('target_depth_m'))
        self.dive_pwm = int(gp('dive_pwm'))
        rate = float(gp('control_rate_hz'))

        # ── Rota Durumu ──
        self.path_pts = []
        self.macro_segments = []
        self.wp_idx = 0
        self.state = 'WAITING_FOR_PATH'
        self.dive_start_time = None
        self.fwd_start_time = None
        self.target_duration = 0.0  # Fallback zaman

        # ── Sensör Verileri ──
        self.heading_rad = None       # Pusula (ENU radyan)
        self.rel_alt = None
        self.vfr_alt = None

        # ZED verisi
        self.zed_x = 0.0
        self.zed_y = 0.0
        self.zed_yaw = 0.0
        self.last_zed_time = None     # Son ZED mesaj zamanı

        # Segment başlangıcı (ZED koordinatları)
        self.seg_start_zed_x = 0.0
        self.seg_start_zed_y = 0.0
        self.seg_ref_heading = 0.0    # Segment başındaki pusula heading'i (ileri heading düzeltmesi)

        # Orange Cube IMU
        self.cube_ax = 0.0; self.cube_ay = 0.0; self.cube_az = 0.0
        self.cube_gx = 0.0; self.cube_gy = 0.0; self.cube_gz = 0.0

        # ZED IMU
        self.zed_ax = 0.0; self.zed_ay = 0.0; self.zed_az = 0.0
        self.zed_gx = 0.0; self.zed_gy = 0.0; self.zed_gz = 0.0

        # EKF (sadece loglama)
        self.ekf_x = 0.0; self.ekf_y = 0.0; self.ekf_z = 0.0; self.ekf_yaw = 0.0

        # ZED FPS
        self.zed_fps = 0.0
        self.zed_pose_count = 0
        self.last_fps_calc_time = None
        self.zed_diag_msg = "OK"

        # PWM çıktıları (loglama için)
        self.fwd_out = 1500
        self.yaw_out = 1500

        # ZED fallback flag
        self.using_zed_fallback = False

        # ── CSV Logging ──
        log_dir = str(gp('log_dir'))
        if not log_dir:
            log_dir = os.getcwd()
        os.makedirs(log_dir, exist_ok=True)

        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = os.path.join(log_dir, f'hybrid_follower_log_{stamp}.csv')
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            'timestamp', 'state', 'wp_idx',
            'heading_deg', 'target_heading_deg', 'ref_heading_deg',
            'rel_alt', 'vfr_alt', 'fwd_pwm', 'yaw_pwm',
            'distance_traveled', 'segment_length', 'duration_left',
            'zed_x', 'zed_y', 'zed_yaw_deg',
            'cube_ax', 'cube_ay', 'cube_az', 'cube_gx', 'cube_gy', 'cube_gz',
            'zed_ax', 'zed_ay', 'zed_az', 'zed_gx', 'zed_gy', 'zed_gz',
            'ekf_x', 'ekf_y', 'ekf_z', 'ekf_yaw_deg',
            'zed_fps', 'zed_diag', 'using_fallback'
        ])

        # ── QoS ──
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ── Abonelikler ──
        self.path_sub = self.create_subscription(Path, str(gp('path_topic')), self.on_path, latched)
        self.vfr_sub = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.on_vfr, sensor_qos)
        self.rel_alt_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.on_rel_alt, sensor_qos)
        self.zed_sub = self.create_subscription(PoseStamped, '/zed/zed_node/pose', self.on_zed_pose, sensor_qos)
        self.ekf_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_ekf_pose, sensor_qos)
        self.cube_imu_sub = self.create_subscription(Imu, '/mavros/imu/data', self.on_cube_imu, sensor_qos)
        self.zed_imu_sub = self.create_subscription(Imu, '/zed/zed_node/imu/data', self.on_zed_imu, sensor_qos)
        self.diag_sub = self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, sensor_qos)

        # ── Yayıncılar & Servisler ──
        self.rc_pub = self.create_publisher(OverrideRCIn, str(gp('rc_override_topic')), 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # ── Kontrol Döngüsü ──
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info("=" * 60)
        self.get_logger().info("HİBRİT Rota Takipçisi Başladı")
        self.get_logger().info(f"  Heading Kaynağı : Pusula (VFR_HUD)")
        self.get_logger().info(f"  XY Kaynağı      : ZED VIO (/zed/zed_node/pose)")
        self.get_logger().info(f"  Derinlik        : Orange Cube (rel_alt + ALT_HOLD)")
        self.get_logger().info(f"  İleri PWM       : {self.fwd_pwm}")
        self.get_logger().info(f"  Hedef Derinlik  : {self.target_depth} m")
        self.get_logger().info(f"  ZED Timeout     : {self.zed_timeout} s")
        self.get_logger().info(f"  Kayıt Dosyası   : {self.log_filename}")
        self.get_logger().info("=" * 60)

    # ═══════════════ CALLBACK'LER ═══════════════

    def on_path(self, msg: Path):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if len(pts) < 2:
            self.stop()
            self.state = 'WAITING_FOR_PATH'
            return

        self.macro_segments = simplify_path(pts, 2.0)
        self.path_pts = pts
        self.wp_idx = 0

        total_dist = sum(math.hypot(
            self.macro_segments[i+1][0] - self.macro_segments[i][0],
            self.macro_segments[i+1][1] - self.macro_segments[i][1]
        ) for i in range(len(self.macro_segments) - 1))

        self.get_logger().info(
            f'Rota alındı: {len(pts)} nokta → {len(self.macro_segments)} makro segment, '
            f'toplam {total_dist:.1f} m')

        if self.state in ['WAITING_FOR_PATH', 'DONE']:
            self.state = 'ARMING'

    def on_vfr(self, msg: VfrHud):
        """VFR HUD pusula verisi (NED → ENU dönüşümü)."""
        ned_rad = math.radians(msg.heading)
        self.heading_rad = normalize_angle(math.pi / 2.0 - ned_rad)
        self.vfr_alt = msg.altitude

    def on_rel_alt(self, msg: Float64):
        self.rel_alt = msg.data

    def on_zed_pose(self, msg: PoseStamped):
        """ZED VIO pozisyon — XY pozisyon segment takibinde kullanılır."""
        self.zed_pose_count += 1
        self.zed_x = msg.pose.position.x
        self.zed_y = msg.pose.position.y
        o = msg.pose.orientation
        self.zed_yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        self.last_zed_time = self.get_clock().now()

    def on_ekf_pose(self, msg: PoseStamped):
        """EKF pose — sadece loglama."""
        self.ekf_x = msg.pose.position.x
        self.ekf_y = msg.pose.position.y
        self.ekf_z = msg.pose.position.z
        o = msg.pose.orientation
        self.ekf_yaw = quat_to_yaw(o.x, o.y, o.z, o.w)

    def on_cube_imu(self, msg: Imu):
        self.cube_ax = msg.linear_acceleration.x
        self.cube_ay = msg.linear_acceleration.y
        self.cube_az = msg.linear_acceleration.z
        self.cube_gx = msg.angular_velocity.x
        self.cube_gy = msg.angular_velocity.y
        self.cube_gz = msg.angular_velocity.z

    def on_zed_imu(self, msg: Imu):
        self.zed_ax = msg.linear_acceleration.x
        self.zed_ay = msg.linear_acceleration.y
        self.zed_az = msg.linear_acceleration.z
        self.zed_gx = msg.angular_velocity.x
        self.zed_gy = msg.angular_velocity.y
        self.zed_gz = msg.angular_velocity.z

    def on_diagnostics(self, msg: DiagnosticArray):
        errs = []
        for status in msg.status:
            lvl = status.level[0] if isinstance(status.level, bytes) else status.level
            if "zed" in status.name.lower() and lvl > 0:
                clean_msg = status.message.replace(',', ';')
                errs.append(f"LVL{lvl}:{clean_msg}")
        self.zed_diag_msg = " | ".join(errs) if errs else "OK"

    # ═══════════════ KONTROL ═══════════════

    def stop(self):
        """Motorları durdur ve DISARM."""
        rc_msg = OverrideRCIn()
        rc_msg.channels = [0] * 18
        self.fwd_out = 1500
        self.yaw_out = 1500
        self.rc_pub.publish(rc_msg)

        if self.arm_client.wait_for_service(timeout_sec=0.5):
            req = CommandBool.Request()
            req.value = False
            self.arm_client.call_async(req)

    def is_zed_alive(self, now):
        """ZED verisinin timeout süresi içinde gelip gelmediğini kontrol et."""
        if self.last_zed_time is None:
            return False
        age = (now - self.last_zed_time).nanoseconds * 1e-9
        return age < self.zed_timeout

    def control_loop(self):
        now = self.get_clock().now()

        # ── FPS Hesabı ──
        if self.last_fps_calc_time is None:
            self.last_fps_calc_time = now
        else:
            dt = (now - self.last_fps_calc_time).nanoseconds * 1e-9
            if dt >= 1.0:
                self.zed_fps = self.zed_pose_count / dt
                self.zed_pose_count = 0
                self.last_fps_calc_time = now

        # ── LOGGING (20 Hz) ──
        self._log_row(now)

        if self.state in ['WAITING_FOR_PATH', 'DONE']:
            return

        if self.heading_rad is None or self.rel_alt is None:
            self.get_logger().info('Pusula veya rel_alt bekleniyor...', throttle_duration_sec=2.0)
            return

        # ═══ STATE MACHINE ═══

        if self.state == 'ARMING':
            self._do_arming(now)
            return

        if self.state == 'DIVING':
            self._do_diving(now)
            return

        # ROTATING ve FORWARD kontrol
        if self.wp_idx >= len(self.macro_segments) - 1:
            self.get_logger().info('HEDEFE ULAŞILDI. Araç durduruluyor.')
            self.stop()
            self.state = 'DONE'
            return

        cur_p = self.macro_segments[self.wp_idx]
        nxt_p = self.macro_segments[self.wp_idx + 1]
        target_heading = math.atan2(nxt_p[1] - cur_p[1], nxt_p[0] - cur_p[0])
        seg_length = math.hypot(nxt_p[0] - cur_p[0], nxt_p[1] - cur_p[1])
        heading_err = normalize_angle(target_heading - self.heading_rad)

        if self.state == 'ROTATING':
            self._do_rotating(now, heading_err, target_heading, seg_length)

        elif self.state == 'FORWARD':
            self._do_forward(now, seg_length)

    # ═══════════════ STATE İŞLEYİCİLERİ ═══════════════

    def _do_arming(self, now):
        if not hasattr(self, '_arm_state_start'):
            self._arm_state_start = now
        elapsed = (now - self._arm_state_start).nanoseconds * 1e-9

        if not hasattr(self, '_last_arm_attempt') or (now - self._last_arm_attempt).nanoseconds * 1e-9 > 2.0:
            self._last_arm_attempt = now
            if self.mode_client.wait_for_service(timeout_sec=0.5):
                req = SetMode.Request()
                req.custom_mode = 'ALT_HOLD'
                self.mode_client.call_async(req)
            if self.arm_client.wait_for_service(timeout_sec=0.5):
                req = CommandBool.Request()
                req.value = True
                self.arm_client.call_async(req)
            self.get_logger().info(f'ALT_HOLD + ARM gönderildi (elapsed={elapsed:.1f}s)')

        # Güvenlik nötr PWM
        rc = OverrideRCIn()
        rc.channels = [65535] * 18
        rc.channels[2] = 1500
        rc.channels[3] = 1500
        rc.channels[4] = 1500
        self.rc_pub.publish(rc)

        if elapsed > 4.0:
            self.state = 'DIVING'
            self.dive_start_time = now
            self.get_logger().info(f'Araç hazır! Dalış başlıyor ({self.dive_pwm} PWM)...')

    def _do_diving(self, now):
        rc = OverrideRCIn()
        rc.channels = [65535] * 18
        rc.channels[2] = self.dive_pwm
        rc.channels[3] = 1500
        rc.channels[4] = 1500
        rc.channels[5] = 1500
        self.fwd_out = 1500
        self.yaw_out = 1500
        self.rc_pub.publish(rc)

        elapsed = (now - self.dive_start_time).nanoseconds * 1e-9
        self.get_logger().info(
            f'Dalış: {self.dive_pwm} PWM, rel_alt: {self.rel_alt:.2f}m, süre: {elapsed:.1f}s',
            throttle_duration_sec=2.0)

        if elapsed > 30.0 or self.rel_alt < self.target_depth:
            self.state = 'ROTATING'
            self.get_logger().info(
                f'Hedef derinliğe ulaşıldı (rel_alt: {self.rel_alt:.2f}m). Rota takibine geçiliyor.')

    def _do_rotating(self, now, heading_err, target_heading, seg_length):
        """Pusula heading ile hedef açıya dönüş. Tolerans içindeyse FORWARD'a geç."""
        if abs(heading_err) <= self.yaw_tol:
            # Segment başlangıcını kaydet
            self.seg_start_zed_x = self.zed_x
            self.seg_start_zed_y = self.zed_y
            self.seg_ref_heading = self.heading_rad  # İleri giderken referans heading
            self.fwd_start_time = now
            self.target_duration = seg_length / self.speed_mps  # Fallback zaman
            self.using_zed_fallback = False

            self.state = 'FORWARD'
            self.get_logger().info(
                f"Açı hizalandı. Segment: {seg_length:.1f}m, "
                f"Fallback süresi: {self.target_duration:.1f}s, PWM: {self.fwd_pwm}")
            return

        # P kontrol ile yaw PWM
        w = max(-self.w_max, min(self.w_max, self.k_heading * heading_err))
        pwm_yaw = int(1500 - (w / self.w_max) * 200) if self.w_max > 0 else 1500

        # Derinlik kontrolü
        depth_err = self.target_depth - (self.rel_alt if self.rel_alt is not None else 0.0)
        pwm_thr = int(1450 + 300.0 * depth_err)

        rc = OverrideRCIn()
        rc.channels = [65535] * 18
        rc.channels[2] = max(1300, min(1700, pwm_thr))
        rc.channels[3] = max(1100, min(1900, pwm_yaw))
        rc.channels[4] = 1500  # İleri duruyor
        rc.channels[5] = 1500  # Yanal kilitli

        self.fwd_out = 1500
        self.yaw_out = rc.channels[3]
        self.rc_pub.publish(rc)

        self.get_logger().info(
            f"Dönülüyor... Hata: {math.degrees(heading_err):.1f}°, "
            f"Yaw PWM: {self.yaw_out}",
            throttle_duration_sec=1.0)

    def _do_forward(self, now, seg_length):
        """İleri hareket — ZED XY mesafe ile segment bitişi, pusula ile heading düzeltmesi."""
        elapsed = (now - self.fwd_start_time).nanoseconds * 1e-9

        # ── ZED ile mesafe hesabı ──
        zed_alive = self.is_zed_alive(now)
        dist_traveled = math.hypot(
            self.zed_x - self.seg_start_zed_x,
            self.zed_y - self.seg_start_zed_y
        )

        # ── Segment bitiş kontrolü ──
        segment_done = False

        if zed_alive:
            # ZED XY mesafe bazlı bitiş (birincil)
            self.using_zed_fallback = False
            if dist_traveled >= (seg_length - self.segment_tol):
                segment_done = True
                self.get_logger().info(
                    f"Segment TAMAMLANDI (ZED XY). Kat edilen: {dist_traveled:.2f}m / Hedef: {seg_length:.1f}m")
        else:
            # Fallback: Zaman bazlı bitiş
            self.using_zed_fallback = True
            if elapsed >= self.target_duration:
                segment_done = True
                self.get_logger().info(
                    f"Segment TAMAMLANDI (FALLBACK zaman). Süre: {elapsed:.1f}s / Hedef: {self.target_duration:.1f}s")
            if not hasattr(self, '_warned_zed_fallback') or not self._warned_zed_fallback:
                self.get_logger().warn(
                    f'ZED verisi kesildi! Zaman bazlı fallback aktif (timeout={self.zed_timeout}s)')
                self._warned_zed_fallback = True

        if segment_done:
            self.wp_idx += 1
            self.state = 'ROTATING'
            self._warned_zed_fallback = False
            self.stop_forward()
            return

        # ── Heading düzeltmesi (pusula ile kapalı çevrim) ──
        heading_err_fwd = normalize_angle(self.seg_ref_heading - self.heading_rad)
        w = max(-self.w_max, min(self.w_max, self.k_heading_fwd * heading_err_fwd))
        pwm_yaw = int(1500 - (w / self.w_max) * 100) if self.w_max > 0 else 1500
        # Heading düzeltme daha yumuşak: max ±100 PWM (dönüşte ±200)

        # ── Derinlik kontrolü ──
        depth_err = self.target_depth - (self.rel_alt if self.rel_alt is not None else 0.0)
        pwm_thr = int(1450 + 300.0 * depth_err)

        rc = OverrideRCIn()
        rc.channels = [65535] * 18
        rc.channels[2] = max(1300, min(1700, pwm_thr))
        rc.channels[3] = max(1100, min(1900, pwm_yaw))
        rc.channels[4] = self.fwd_pwm
        rc.channels[5] = 1500

        self.fwd_out = self.fwd_pwm
        self.yaw_out = rc.channels[3]
        self.rc_pub.publish(rc)

        # İlerleme bilgisi
        remaining_info = ""
        if zed_alive:
            remaining_info = f"Mesafe: {dist_traveled:.2f}/{seg_length:.1f}m"
        else:
            remaining_info = f"Süre: {elapsed:.1f}/{self.target_duration:.1f}s [FALLBACK]"

        self.get_logger().info(
            f"İleri: {remaining_info}, Heading düzeltme: {math.degrees(heading_err_fwd):.1f}°, "
            f"PWM(Fwd,Yaw): ({self.fwd_pwm},{self.yaw_out})",
            throttle_duration_sec=1.0)

    def stop_forward(self):
        """Segment arası kısa duruş (DISARM yapmadan)."""
        rc = OverrideRCIn()
        rc.channels = [65535] * 18
        rc.channels[2] = 1500
        rc.channels[3] = 1500
        rc.channels[4] = 1500
        rc.channels[5] = 1500
        self.fwd_out = 1500
        self.yaw_out = 1500
        self.rc_pub.publish(rc)

    # ═══════════════ LOGGING ═══════════════

    def _log_row(self, now):
        """CSV'ye 20 Hz'de satır yaz."""
        if self.heading_rad is None:
            return

        t_sec = float(now.nanoseconds) / 1e9
        th = 0.0
        ref_h = math.degrees(self.seg_ref_heading) if hasattr(self, 'seg_ref_heading') else 0.0
        seg_len = 0.0
        dist_traveled = 0.0
        dur = 0.0

        if self.wp_idx < len(self.macro_segments) - 1:
            cur = self.macro_segments[self.wp_idx]
            nxt = self.macro_segments[self.wp_idx + 1]
            th = math.degrees(math.atan2(nxt[1] - cur[1], nxt[0] - cur[0]))
            seg_len = math.hypot(nxt[0] - cur[0], nxt[1] - cur[1])

            if self.state == 'FORWARD' and self.fwd_start_time:
                dur = max(0.0, self.target_duration - (now - self.fwd_start_time).nanoseconds * 1e-9)
                dist_traveled = math.hypot(
                    self.zed_x - self.seg_start_zed_x,
                    self.zed_y - self.seg_start_zed_y)

        self.csv_writer.writerow([
            f"{t_sec:.3f}", self.state, self.wp_idx,
            f"{math.degrees(self.heading_rad):.2f}", f"{th:.2f}", f"{ref_h:.2f}",
            f"{self.rel_alt if self.rel_alt is not None else 0.0:.2f}",
            f"{self.vfr_alt if self.vfr_alt is not None else 0.0:.2f}",
            self.fwd_out, self.yaw_out,
            f"{dist_traveled:.3f}", f"{seg_len:.2f}", f"{dur:.2f}",
            f"{self.zed_x:.3f}", f"{self.zed_y:.3f}", f"{math.degrees(self.zed_yaw):.2f}",
            f"{self.cube_ax:.3f}", f"{self.cube_ay:.3f}", f"{self.cube_az:.3f}",
            f"{self.cube_gx:.3f}", f"{self.cube_gy:.3f}", f"{self.cube_gz:.3f}",
            f"{self.zed_ax:.3f}", f"{self.zed_ay:.3f}", f"{self.zed_az:.3f}",
            f"{self.zed_gx:.3f}", f"{self.zed_gy:.3f}", f"{self.zed_gz:.3f}",
            f"{self.ekf_x:.3f}", f"{self.ekf_y:.3f}", f"{self.ekf_z:.3f}",
            f"{math.degrees(self.ekf_yaw):.2f}",
            f"{self.zed_fps:.1f}", self.zed_diag_msg,
            1 if self.using_zed_fallback else 0
        ])
        self.log_file.flush()


def main(args=None):
    rclpy.init(args=args)
    node = HybridPathFollowerNode()

    def sigint_handler(sig, frame):
        node.get_logger().info("CTRL+C Algılandı! Araç acil durduruluyor...")
        node.stop()
        if not node.log_file.closed:
            node.log_file.close()

        import time
        time.sleep(0.2)

        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(node)
    except Exception:
        pass


if __name__ == '__main__':
    main()
