#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import csv
import datetime
import os
import struct
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Path, Odometry
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import VfrHud, OverrideRCIn
from diagnostic_msgs.msg import DiagnosticArray

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

def point_line_distance(point, start, end):
    if start == end:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    num = abs((end[1] - start[1]) * point[0] - (end[0] - start[0]) * point[1] + end[0] * start[1] - end[1] * start[0])
    den = math.hypot(end[1] - start[1], end[0] - start[0])
    return num / den

def rdp(points, epsilon):
    dmax = 0.0
    index = 0
    end = len(points) - 1
    for i in range(1, end):
        d = point_line_distance(points[i], points[0], points[end])
        if d > dmax:
            index = i
            dmax = d
    if dmax > epsilon:
        left = rdp(points[:index+1], epsilon)
        right = rdp(points[index:], epsilon)
        return left[:-1] + right
    else:
        return [points[0], points[end]]

class AdvancedBlindFollower(Node):
    def __init__(self):
        super().__init__('advanced_blind_follower')
        
        # Parametreler
        self.declare_parameter('rc_override_topic', '/mavros/rc/override')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('fwd_pwm', 1680)
        self.declare_parameter('fwd_duration_ref', 25.0)   # 5m = 25s
        self.declare_parameter('k_heading_fwd', 2.5)   
        
        gp = lambda n: self.get_parameter(n).value
        self.fwd_pwm = int(gp('fwd_pwm'))
        self.fwd_duration_ref = float(gp('fwd_duration_ref'))
        self.k_heading_fwd = float(gp('k_heading_fwd'))
        rate = float(gp('control_rate_hz'))
        
        self.vfr_alt = None
        self.ekf_yaw_rad = None  # Ana yönetim açısı (EKF'den gelen temiz yönelim)
        self.shutdown_requested = False
        
        # Segment logic
        self.segments = []  # List of {'distance': D, 'heading_rad': H}
        self.curr_segment_idx = 0
        self.target_dist_log = 0.0
        
        self.state = 'STARTING'
        self.state_start_time = self.get_clock().now()
        self.rotating_settled_start = None
        
        # ZED Logging variables (arka planda sessizce kaydedeceğiz)
        self.zed_odom_x = 0.0
        self.zed_odom_y = 0.0
        self.zed_odom_z = 0.0
        self.zed_odom_yaw = 0.0
        
        # CSV Logging Setup
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = os.path.join(os.path.expanduser('~'), f'advanced_sensor_fusion_log_{stamp}.csv')
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            'timestamp', 'state', 'segment_idx', 'tgt_dist',
            'ekf_yaw_deg', 'vfr_alt', 'thr_pwm', 'fwd_pwm', 'yaw_pwm', 'duration_left',
            'zed_odom_x', 'zed_odom_y', 'zed_odom_z', 'zed_odom_yaw_deg'
        ])
        
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        latched_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
            
        # Abonelikler
        self.vfr_sub = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.on_vfr, sensor_qos)
        self.ekf_odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.on_ekf_odom, sensor_qos)
        self.zed_odom_sub = self.create_subscription(Odometry, '/zed/zed_node/odom', self.on_zed_odom, sensor_qos)
        self.path_sub = self.create_subscription(Path, '/planned_route', self.on_planned_route, latched_qos)
        
        self.rc_pub = self.create_publisher(OverrideRCIn, str(gp('rc_override_topic')), 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        
        self.get_logger().info(f"Yedekli Sensör Füzyonu Sistemi (Advanced Blind Follower) Başlatıldı.")
        self.get_logger().info(f"Kayıt Dosyası: {self.log_filename}")

    def on_vfr(self, msg: VfrHud):
        self.vfr_alt = msg.altitude

    def on_ekf_odom(self, msg: Odometry):
        o = msg.pose.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        # EKF Çıktısı ENU standardındadır (X ileriyi, Y solu gösterir). 
        # Matematiksel olarak NED formatından dönüştürmemize gerek kalmaz, doğrudan Hedef Matrisiyle kullanılabilir.
        # Rota Planlayıcı ENU standardına göre açı (atan2) gönderdiği için birebir eşleşir.
        self.ekf_yaw_rad = yaw

    def on_zed_odom(self, msg: Odometry):
        # Arka planda loglamak için ZED kameranın kendi optik frame uzayındaki değerleri
        self.zed_odom_x = msg.pose.pose.position.x
        self.zed_odom_y = msg.pose.pose.position.y
        self.zed_odom_z = msg.pose.pose.position.z
        o = msg.pose.pose.orientation
        self.zed_odom_yaw = quat_to_yaw(o.x, o.y, o.z, o.w)

    def on_planned_route(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) < 2: return
            
        simp_pts = rdp(pts, 0.5)
        new_segments = []
        for i in range(len(simp_pts)-1):
            p0 = simp_pts[i]
            p1 = simp_pts[i+1]
            dist = math.hypot(p1[1] - p0[1], p1[0] - p0[0])
            hdg = math.atan2(p1[1] - p0[1], p1[0] - p0[0]) # Hedef açısı ENU (Doğu->Kuzey->Batı) formatında.
            if dist > 0.5:
                new_segments.append({'distance': dist, 'heading_rad': hdg})
                
        if len(new_segments) > 0:
            self.segments = new_segments
            self.get_logger().info(f"YENİ ROTA ALINDI. Toplam {len(self.segments)} adet sekans çıkartıldı.")
            for i, s in enumerate(self.segments):
                self.get_logger().info(f"Segment {i+1}: Mesafe {s['distance']:.2f}m, Hedef Açısı: {math.degrees(s['heading_rad']):.1f} rad, ENU")

    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().info(f"--- STATE: {new_state} ---")

    def stop(self):
        try:
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18 
            self.rc_pub.publish(rc_msg)
            if self.arm_client.wait_for_service(timeout_sec=0.5):
                req = CommandBool.Request()
                req.value = False
                self.arm_client.call_async(req)
        except Exception:
            pass

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        
        hdg_deg = math.degrees(self.ekf_yaw_rad) if self.ekf_yaw_rad is not None else 0.0
        dur = 0.0
        if self.state == 'FORWARD' and self.curr_segment_idx < len(self.segments):
            dur = max(0.0, (self.segments[self.curr_segment_idx]['distance'] * (self.fwd_duration_ref / 5.0)) - elapsed)
            
        thr_out, fwd_out, yaw_out = 1500, 1500, 1500
        
        if self.shutdown_requested:
            self.stop()
            return
            
        if self.state == 'DONE':
            return
            
        if self.state == 'SURFACING':
            self.get_logger().info('Motorlar kapatildi. Arac kendi yuzerliligi ile yuzeye cikiyor...')
            self.stop()
            self.change_state('DONE')
            return

        if self.state == 'STARTING':
            has_hdg = self.ekf_yaw_rad is not None
            has_alt = self.vfr_alt is not None
            self.get_logger().info(f'Başlatılıyor... EKF={has_hdg}, vfr_alt={has_alt}', throttle_duration_sec=2.0)
            if elapsed > 2.0 and has_hdg and has_alt:
                self.change_state('WAIT_ROUTE')
            return
            
        if self.state == 'WAIT_ROUTE':
            if len(self.segments) > 0:
                self.get_logger().info(f'Rota doğrulandı. ARM başlıyor.')
                self.change_state('ARMING')
            else:
                self.get_logger().info('Düz rota noktaları /planned_route için bekleniyor...', throttle_duration_sec=2.0)
            return

        if self.state == 'ARMING':
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
                self.get_logger().info(f'ALT_HOLD + ARM komutu gönderildi (deneme, elapsed={elapsed:.1f}s)')
            
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            for i in range(2, 6): rc_msg.channels[i] = 1500
            self.rc_pub.publish(rc_msg)
            
            if elapsed > 4.0:
                self.change_state('DIVING')
            return

        if self.ekf_yaw_rad is None or self.vfr_alt is None:
            self.get_logger().info(f'Filtrelenmiş EKF veya vfr_alt verisi kayboldu...', throttle_duration_sec=2.0)
            return

        if self.state == 'DIVING':
            yaw_out = 1500
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = 1400  # Dalış PWM
            rc_msg.channels[3] = yaw_out  
            rc_msg.channels[4] = 1500  
            rc_msg.channels[5] = 1500  
            self.rc_pub.publish(rc_msg)
            
            thr_out = 1400
            self.get_logger().info(f'Dalış: 1400 PWM, vfr_alt: {self.vfr_alt:.2f}m', throttle_duration_sec=2.0)
            
            # vfr_alt negatif oldugu icin < -1.0 ya da limit asimi kontrol edilir
            if self.vfr_alt < -1.0 or elapsed > 15.0:
                self.get_logger().info(f"Dalis bitti. Ilk rotasyon segmentine geçiliyor.")
                self.curr_segment_idx = 0
                self.rotating_settled_start = None
                self.change_state('ROTATING')
                
        elif self.state == 'ROTATING':
            target_hdg = self.segments[self.curr_segment_idx]['heading_rad']
            yaw_err = normalize_angle(target_hdg - self.ekf_yaw_rad)
            yaw_err_deg = math.degrees(yaw_err)
            
            depth_err = -1.0 - self.vfr_alt
            thr_out = int(1450 + 300.0 * depth_err)
            thr_out = max(1300, min(1900, thr_out))
            
            # Dönüş için EKF sensör füzyonu kaynaklı Yaw Controller 
            # - (Y ekseni soldur, sola artıdır, - olduğu için sola itiş gerektirir yani Yaw PWM düşürülür veya artırılır)
            yaw_out = int(1500 - (yaw_err_deg * self.k_heading_fwd))
            yaw_out = max(1350, min(1650, yaw_out))
            
            if abs(yaw_err_deg) < 8.0:
                if self.rotating_settled_start is None:
                    self.rotating_settled_start = now
                elif (now - self.rotating_settled_start).nanoseconds * 1e-9 > 3.0:
                    self.get_logger().info(f"Rotasyon kararlı tamamlandı. İleri harekete geçiliyor.")
                    self.change_state('FORWARD')
                    return
            else:
                self.rotating_settled_start = None
                
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = thr_out
            rc_msg.channels[3] = yaw_out
            rc_msg.channels[4] = 1500
            rc_msg.channels[5] = 1500 
            self.rc_pub.publish(rc_msg)
            
            self.get_logger().info(
                f"Sadece Dönüş [{self.curr_segment_idx+1}/{len(self.segments)}]: Hdf={math.degrees(target_hdg):.1f} | Frk={yaw_err_deg:.1f} | PWM={yaw_out}",
                throttle_duration_sec=1.0)
                
        elif self.state == 'FORWARD':
            segment = self.segments[self.curr_segment_idx]
            target_hdg = segment['heading_rad']
            self.target_dist_log = segment['distance']
            active_duration = self.target_dist_log * (self.fwd_duration_ref / 5.0)
            
            if elapsed >= active_duration:
                self.get_logger().info(f"Segment {self.curr_segment_idx+1} bitti! {active_duration:.1f} sn tamamlandı.")
                self.curr_segment_idx += 1
                if self.curr_segment_idx >= len(self.segments):
                    self.change_state('SURFACING')
                else:
                    self.rotating_settled_start = None
                    self.change_state('ROTATING')
                return
                
            depth_err = -1.0 - self.vfr_alt
            thr_out = int(1450 + 300.0 * depth_err)
            thr_out = max(1300, min(1900, thr_out))
            
            yaw_err = normalize_angle(target_hdg - self.ekf_yaw_rad)
            yaw_err_deg = math.degrees(yaw_err)
            yaw_out = int(1500 - (yaw_err_deg * self.k_heading_fwd))
            yaw_out = max(1450, min(1550, yaw_out))
            
            fwd_out = self.fwd_pwm
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = thr_out
            rc_msg.channels[3] = yaw_out
            rc_msg.channels[4] = fwd_out
            rc_msg.channels[5] = 1500 
            self.rc_pub.publish(rc_msg)
            
            self.get_logger().info(
                f"Güvenli Sürüş [{self.curr_segment_idx+1}/{len(self.segments)}] Kalan:{active_duration - elapsed:.1f}s | YawErr:{yaw_err_deg:.1f} | Y-PWM:{yaw_out}",
                throttle_duration_sec=1.0)

        # Log yaz
        self.csv_writer.writerow([
            f"{float(now.nanoseconds)/1e9:.3f}", self.state, self.curr_segment_idx, f"{self.target_dist_log:.2f}",
            f"{hdg_deg:.2f}", f"{self.vfr_alt:.2f}", 
            thr_out, fwd_out, yaw_out, f"{dur:.2f}",
            f"{self.zed_odom_x:.3f}", f"{self.zed_odom_y:.3f}", f"{self.zed_odom_z:.3f}", f"{math.degrees(self.zed_odom_yaw):.2f}"
        ])
        self.log_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedBlindFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Kapanıyor...")
    finally:
        node.stop()
        if not node.log_file.closed:
            node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
