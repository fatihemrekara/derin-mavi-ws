#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import csv
import datetime
import signal
import sys
import os
import struct
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, Image
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import VfrHud, OverrideRCIn
from nav_msgs.msg import Path

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
    # The normal distance from point to the line (start, end)
    num = abs((end[1] - start[1]) * point[0] - (end[0] - start[0]) * point[1] + end[0] * start[1] - end[1] * start[0])
    den = math.hypot(end[1] - start[1], end[0] - start[0])
    return num / den

def rdp(points, epsilon):
    """Ramer-Douglas-Peucker algorithm to simplify a path."""
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

class SquareBlindTestNode(Node):
    def __init__(self):
        super().__init__('square_blind_test')
        
        self.declare_parameter('rc_override_topic', '/mavros/rc/override')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('fwd_pwm', 1700)        # Ileri yon throttle'i
        self.declare_parameter('fwd_duration_ref', 25.0)   # 5m = 25s referansi
        self.declare_parameter('k_heading_fwd', 2.0)   # Ileri surus yaw P katsayisi
        self.declare_parameter('target_depth_m', -0.5)  # Hedef derinlik (metre, negatif = su altı)
        
        gp = lambda n: self.get_parameter(n).value
        self.fwd_pwm = int(gp('fwd_pwm'))
        self.fwd_duration_ref = float(gp('fwd_duration_ref'))
        self.k_heading_fwd = float(gp('k_heading_fwd'))
        self.target_depth = float(gp('target_depth_m'))
        rate = float(gp('control_rate_hz'))
        
        self.heading_rad = None
        self.rel_alt = None
        self.vfr_alt = None
        self.shutdown_requested = False
        
        # Segment logic
        self.segments = []  # List of {'distance': D, 'heading_rad': H}
        self.curr_segment_idx = 0
        self.target_dist_log = 0.0
        
        # ZED logging vars
        self.zed_x = 0.0; self.zed_y = 0.0; self.zed_yaw = 0.0
        self.cube_ax = 0.0; self.cube_ay = 0.0; self.cube_az = 0.0
        self.cube_gx = 0.0; self.cube_gy = 0.0; self.cube_gz = 0.0
        self.zed_ax = 0.0; self.zed_ay = 0.0; self.zed_az = 0.0
        self.zed_gx = 0.0; self.zed_gy = 0.0; self.zed_gz = 0.0
        self.zed_ax_raw = 0.0; self.zed_ay_raw = 0.0; self.zed_az_raw = 0.0
        self.zed_gx_raw = 0.0; self.zed_gy_raw = 0.0; self.zed_gz_raw = 0.0
        self.zed_center_depth = 0.0
        self.ekf_x = 0.0; self.ekf_y = 0.0; self.ekf_z = 0.0; self.ekf_yaw = 0.0
        
        self.zed_fps = 0.0
        self.zed_pose_count = 0
        self.last_fps_calc_time = None
        self.zed_diag_msg = "OK"
        
        self.state = 'STARTING'
        self.state_start_time = self.get_clock().now()
        self.rotating_settled_start = None
        
        self.fwd_out = 1500
        self.yaw_out = 1500
        self.thr_out = 1500
        
        # CSV Logging Setup
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = os.path.join(os.path.expanduser('~'), f'square_blind_log_{stamp}.csv')
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            'timestamp', 'state', 'segment_idx', 'tgt_dist',
            'heading_deg', 'rel_alt', 'vfr_alt', 
            'thr_pwm', 'fwd_pwm', 'yaw_pwm', 'duration_left',
            'zed_x', 'zed_y', 'zed_yaw_deg',
            'cube_ax', 'cube_ay', 'cube_az', 'cube_gx', 'cube_gy', 'cube_gz',
            'zed_ax', 'zed_ay', 'zed_az', 'zed_gx', 'zed_gy', 'zed_gz',
            'zed_ax_raw', 'zed_ay_raw', 'zed_az_raw', 'zed_gx_raw', 'zed_gy_raw', 'zed_gz_raw',
            'zed_center_depth_m',
            'ekf_x', 'ekf_y', 'ekf_z', 'ekf_yaw_deg',
            'zed_fps', 'zed_diag'
        ])
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
            
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
            
        self.vfr_sub = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.on_vfr, sensor_qos)
        self.rel_alt_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.on_rel_alt, sensor_qos)
        self.zed_sub = self.create_subscription(PoseStamped, '/zed/zed_node/pose', self.on_zed_pose, sensor_qos)
        self.ekf_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_ekf_pose, sensor_qos)
        
        self.cube_imu_sub = self.create_subscription(Imu, '/mavros/imu/data', self.on_cube_imu, sensor_qos)
        self.zed_imu_sub = self.create_subscription(Imu, '/zed/zed_node/imu/data', self.on_zed_imu, sensor_qos)
        self.zed_imu_raw_sub = self.create_subscription(Imu, '/zed/zed_node/imu/data_raw', self.on_zed_imu_raw, sensor_qos)
        self.zed_depth_sub = self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.on_zed_depth, sensor_qos)
        self.diag_sub = self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, sensor_qos)
        
        self.path_sub = self.create_subscription(Path, '/planned_route', self.on_planned_route, latched_qos)
        
        self.rc_pub = self.create_publisher(OverrideRCIn, str(gp('rc_override_topic')), 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        
        self.get_logger().info(f"Kare Sürüş (Square Blind) Testi Başladı.")
        self.get_logger().info(f"Kayıt Dosyası: {self.log_filename}")

    def on_vfr(self, msg: VfrHud):
        ned_rad = math.radians(msg.heading)
        self.heading_rad = normalize_angle(math.pi/2.0 - ned_rad)
        self.vfr_alt = msg.altitude
        
    def on_rel_alt(self, msg: Float64):
        self.rel_alt = msg.data

    def on_zed_pose(self, msg: PoseStamped):
        self.zed_pose_count += 1
        self.zed_x = msg.pose.position.x
        self.zed_y = msg.pose.position.y
        o = msg.pose.orientation
        self.zed_yaw = quat_to_yaw(o.x, o.y, o.z, o.w)

    def on_ekf_pose(self, msg: PoseStamped):
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

    def on_zed_imu_raw(self, msg: Imu):
        self.zed_ax_raw = msg.linear_acceleration.x
        self.zed_ay_raw = msg.linear_acceleration.y
        self.zed_az_raw = msg.linear_acceleration.z
        self.zed_gx_raw = msg.angular_velocity.x
        self.zed_gy_raw = msg.angular_velocity.y
        self.zed_gz_raw = msg.angular_velocity.z

    def on_zed_depth(self, msg: Image):
        if msg.encoding == '32FC1':
            center_row = msg.height // 2
            center_col = msg.width // 2
            window_size = 10
            valid_depths = []
            
            for r in range(max(0, center_row - window_size), min(msg.height, center_row + window_size)):
                for c in range(max(0, center_col - window_size), min(msg.width, center_col + window_size)):
                    idx = (r * msg.step) + (c * 4)
                    data_bytes = msg.data[idx:idx+4]
                    if len(data_bytes) == 4:
                        (val,) = struct.unpack('f', data_bytes)
                        if not math.isnan(val) and not math.isinf(val) and val > 0.1:
                            valid_depths.append(val)
            
            if valid_depths:
                self.zed_center_depth = sum(valid_depths) / len(valid_depths)

    def on_diagnostics(self, msg: DiagnosticArray):
        errs = []
        for status in msg.status:
            lvl = status.level[0] if isinstance(status.level, bytes) else status.level
            if "zed" in status.name.lower() and lvl > 0:
                clean_msg = status.message.replace(',', ';')
                errs.append(f"LVL{lvl}:{clean_msg}")
        
        if errs:
            self.zed_diag_msg = " | ".join(errs)
        else:
            self.zed_diag_msg = "OK"
            
    def on_planned_route(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) < 2:
            return
            
        # Simplify path with RDP algorithm. Epsilon 0.5 is very lenient and will extract macro straight sections
        simp_pts = rdp(pts, 0.5)
        
        new_segments = []
        for i in range(len(simp_pts)-1):
            p0 = simp_pts[i]
            p1 = simp_pts[i+1]
            dist = math.hypot(p1[1] - p0[1], p1[0] - p0[0])
            hdg = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            if dist > 0.5: # Yalnizca en az yarim metrelik manali cizgileri dikkate aliyoruz
                new_segments.append({'distance': dist, 'heading_rad': hdg})
                
        if len(new_segments) > 0:
            self.segments = new_segments
            self.get_logger().info(f"YENİ ROTA ALINDI. Toplam {len(self.segments)} adet sekans çıkartıldı.")
            for i, s in enumerate(self.segments):
                self.get_logger().info(f"Segment {i+1}: Mesafe {s['distance']:.2f}m, Hedef Açısı (derece): {math.degrees(s['heading_rad']):.1f}")

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
        
        # FPS Calculation
        if self.last_fps_calc_time is None:
            self.last_fps_calc_time = now
        else:
            dt = (now - self.last_fps_calc_time).nanoseconds * 1e-9
            if dt >= 1.0:
                self.zed_fps = self.zed_pose_count / dt
                self.zed_pose_count = 0
                self.last_fps_calc_time = now
        
        # Determine remaining duration for active state logging
        dur = 0.0
        if self.state == 'FORWARD' and self.curr_segment_idx < len(self.segments):
            dur = max(0.0, (self.segments[self.curr_segment_idx]['distance'] * (self.fwd_duration_ref / 5.0)) - elapsed)
            
        hdg_deg = math.degrees(self.heading_rad) if self.heading_rad is not None else 0.0
        
        self.csv_writer.writerow([
            f"{float(now.nanoseconds)/1e9:.3f}", self.state, self.curr_segment_idx, f"{self.target_dist_log:.2f}",
            f"{hdg_deg:.2f}", f"{self.rel_alt if self.rel_alt is not None else 0.0:.2f}", f"{self.vfr_alt if self.vfr_alt is not None else 0.0:.2f}",
            self.thr_out, self.fwd_out, self.yaw_out, f"{dur:.2f}",
            f"{self.zed_x:.3f}", f"{self.zed_y:.3f}", f"{math.degrees(self.zed_yaw):.2f}",
            f"{self.cube_ax:.3f}", f"{self.cube_ay:.3f}", f"{self.cube_az:.3f}",
            f"{self.cube_gx:.3f}", f"{self.cube_gy:.3f}", f"{self.cube_gz:.3f}",
            f"{self.zed_ax:.3f}", f"{self.zed_ay:.3f}", f"{self.zed_az:.3f}",
            f"{self.zed_gx:.3f}", f"{self.zed_gy:.3f}", f"{self.zed_gz:.3f}",
            f"{self.zed_ax_raw:.3f}", f"{self.zed_ay_raw:.3f}", f"{self.zed_az_raw:.3f}",
            f"{self.zed_gx_raw:.3f}", f"{self.zed_gy_raw:.3f}", f"{self.zed_gz_raw:.3f}",
            f"{self.zed_center_depth:.3f}",
            f"{self.ekf_x:.3f}", f"{self.ekf_y:.3f}", f"{self.ekf_z:.3f}", f"{math.degrees(self.ekf_yaw):.2f}",
            f"{self.zed_fps:.1f}", self.zed_diag_msg
        ])
        self.log_file.flush()

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
            has_hdg = self.heading_rad is not None
            has_alt = self.rel_alt is not None
            self.get_logger().info(f'Başlatılıyor... pusula={has_hdg}, rel_alt={has_alt}', throttle_duration_sec=2.0)
            if elapsed > 2.0 and has_hdg:
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
            rc_msg.channels[2] = 1500  
            rc_msg.channels[3] = 1500  
            rc_msg.channels[4] = 1500  
            self.rc_pub.publish(rc_msg)
            self.thr_out = 1500; self.fwd_out = 1500; self.yaw_out = 1500; 
            
            if elapsed > 4.0:
                self.change_state('DIVING')
            return

        if self.heading_rad is None or self.rel_alt is None:
            self.get_logger().info(f'Pusula veya rel_alt verisi kayboldu...', throttle_duration_sec=2.0)
            return

        if self.state == 'DIVING':
            yaw_pwm_calc = 1500

            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = 1400  # Dalış PWM
            rc_msg.channels[3] = yaw_pwm_calc  
            rc_msg.channels[4] = 1500  
            rc_msg.channels[5] = 1500  
            self.rc_pub.publish(rc_msg)
            
            self.thr_out = 1400; self.fwd_out = 1500; self.yaw_out = yaw_pwm_calc
            self.get_logger().info(f'Dalış: 1400 PWM, rel_alt: {self.rel_alt:.2f}m, Yaw: {yaw_pwm_calc}', throttle_duration_sec=2.0)
            
            if self.rel_alt < self.target_depth or elapsed > 15.0:
                self.get_logger().info(f"Dalis bitti. Ilk rotasyon segmentine geçiliyor.")
                self.curr_segment_idx = 0
                self.rotating_settled_start = None
                self.change_state('ROTATING')
                
        elif self.state == 'ROTATING':
            target_hdg = self.segments[self.curr_segment_idx]['heading_rad']
            yaw_err = normalize_angle(target_hdg - self.heading_rad)
            yaw_err_deg = math.degrees(yaw_err)
            
            # Sadece yerimizde dönüyoruz, fwd yollamıyoruz. Sadece ALT_HOLD icin derinlik koruma acik
            depth_err = self.target_depth - (self.rel_alt if self.rel_alt is not None else 0.0)
            pwm_thr = int(1450 + 300.0 * depth_err)
            pwm_thr = max(1300, min(1900, pwm_thr))
            
            # Donus icin Bang-Bang Yaw Controller (sabit hiz)
            if abs(yaw_err_deg) < 8.0:
                yaw_pwm_calc = 1500
            elif yaw_err_deg > 0:
                yaw_pwm_calc = 1400
            else:
                yaw_pwm_calc = 1600
            
            if abs(yaw_err_deg) < 8.0:
                if self.rotating_settled_start is None:
                    self.rotating_settled_start = now
                elif (now - self.rotating_settled_start).nanoseconds * 1e-9 > 3.5:
                    self.get_logger().info(f"Rotasyon tamamlandı, açıya yerleşildi. İleri harekete geçiliyor.")
                    self.change_state('FORWARD')
                    return
            else:
                self.rotating_settled_start = None
                
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = pwm_thr
            rc_msg.channels[3] = yaw_pwm_calc
            rc_msg.channels[4] = 1500
            rc_msg.channels[5] = 1500 
            
            self.thr_out = pwm_thr; self.fwd_out = 1500; self.yaw_out = yaw_pwm_calc
            self.rc_pub.publish(rc_msg)
            
            self.get_logger().info(
                f"Sadece Dönüş [{self.curr_segment_idx+1}/{len(self.segments)}]: Hedef={math.degrees(target_hdg):.1f} | Fark={yaw_err_deg:.1f} | Yaw PWM={yaw_pwm_calc}",
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
                    self.get_logger().info("Bütün segmentler bitirildi! Yüzeye çıkılıyor.")
                    self.change_state('SURFACING')
                else:
                    self.rotating_settled_start = None
                    self.change_state('ROTATING')
                return
                
            depth_err = self.target_depth - (self.rel_alt if self.rel_alt is not None else 0.0)
            pwm_thr = int(1450 + 300.0 * depth_err)
            pwm_thr = max(1300, min(1900, pwm_thr))
            
            # Yaw Bang-Bang (SADECE FORWARD'DA, YAW duzeltme)
            yaw_err = normalize_angle(target_hdg - self.heading_rad)
            yaw_err_deg = math.degrees(yaw_err)
            
            if abs(yaw_err_deg) < 8.0:
                yaw_pwm_calc = 1500
            elif yaw_err_deg > 0:
                yaw_pwm_calc = 1400
            else:
                yaw_pwm_calc = 1600

                
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = pwm_thr
            rc_msg.channels[3] = yaw_pwm_calc
            rc_msg.channels[4] = self.fwd_pwm
            rc_msg.channels[5] = 1500 
            
            self.thr_out = pwm_thr; self.fwd_out = self.fwd_pwm; self.yaw_out = yaw_pwm_calc
            self.rc_pub.publish(rc_msg)
            
            self.get_logger().info(
                f"Kör Gidiş [{self.curr_segment_idx+1}/{len(self.segments)}] Kalan:{active_duration - elapsed:.1f}s | Hdg Err:{yaw_err_deg:.1f} | Thr PWM:{pwm_thr}",
                throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = SquareBlindTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("CTRL+C Algılandı! Kapanıyor...")
        pass
    except Exception as e:
        node.get_logger().error(f"Beklenmeyen hata: {e}")
    finally:
        node.stop()
        if not node.log_file.closed:
            node.log_file.close()
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
