#!/usr/bin/env python3
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

class StraightBlindTestNode(Node):
    def __init__(self):
        super().__init__('straight_blind_test')
        
        self.declare_parameter('rc_override_topic', '/mavros/rc/override')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('fwd_pwm', 1700)        # Ileri yon throttle'i
        self.declare_parameter('fwd_duration', 10.0)   # Kac saniye duz gidecek
        self.declare_parameter('k_heading_fwd', 2.0)   # Ileri surus yaw P katsayisi
        
        gp = lambda n: self.get_parameter(n).value
        self.fwd_pwm = int(gp('fwd_pwm'))
        self.fwd_duration = float(gp('fwd_duration'))
        self.k_heading_fwd = float(gp('k_heading_fwd'))
        rate = float(gp('control_rate_hz'))
        
        self.heading_rad = None
        self.ref_heading_rad = None
        self.shutdown_requested = False
        
        # ZED logging vars
        self.zed_x = 0.0; self.zed_y = 0.0; self.zed_yaw = 0.0
        self.cube_ax = 0.0; self.cube_ay = 0.0; self.cube_az = 0.0
        self.cube_gx = 0.0; self.cube_gy = 0.0; self.cube_gz = 0.0
        self.zed_ax = 0.0; self.zed_ay = 0.0; self.zed_az = 0.0
        self.zed_gx = 0.0; self.zed_gy = 0.0; self.zed_gz = 0.0
        self.ekf_x = 0.0; self.ekf_y = 0.0; self.ekf_z = 0.0; self.ekf_yaw = 0.0
        
        self.zed_fps = 0.0
        self.zed_pose_count = 0
        self.last_fps_calc_time = None
        self.zed_diag_msg = "OK"
        
        self.state = 'STARTING'
        self.state_start_time = self.get_clock().now()
        
        self.fwd_out = 1500
        self.yaw_out = 1500
        self.thr_out = 1500
        
        # CSV Logging Setup
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = os.path.join(os.path.expanduser('~'), f'straight_blind_log_{stamp}.csv')
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            'timestamp', 'state',
            'heading_deg', 'rel_alt', 'vfr_alt', 
            'thr_pwm', 'fwd_pwm', 'yaw_pwm', 'duration_left',
            'zed_x', 'zed_y', 'zed_yaw_deg',
            'cube_ax', 'cube_ay', 'cube_az', 'cube_gx', 'cube_gy', 'cube_gz',
            'zed_ax', 'zed_ay', 'zed_az', 'zed_gx', 'zed_gy', 'zed_gz',
            'ekf_x', 'ekf_y', 'ekf_z', 'ekf_yaw_deg',
            'zed_fps', 'zed_diag'
        ])
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
            
        self.vfr_sub = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.on_vfr, sensor_qos)
        self.rel_alt_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.on_rel_alt, sensor_qos)
        self.zed_sub = self.create_subscription(PoseStamped, '/zed/zed_node/pose', self.on_zed_pose, sensor_qos)
        self.ekf_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_ekf_pose, sensor_qos)
        
        self.cube_imu_sub = self.create_subscription(Imu, '/mavros/imu/data', self.on_cube_imu, sensor_qos)
        self.zed_imu_sub = self.create_subscription(Imu, '/zed/zed_node/imu/data', self.on_zed_imu, sensor_qos)
        self.diag_sub = self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, sensor_qos)
        
        self.rc_pub = self.create_publisher(OverrideRCIn, str(gp('rc_override_topic')), 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        
        self.get_logger().info(f"Düz Sürüş (Straight Blind) Testi Başladı. {self.fwd_duration} sn boyunca Ileri PWM:{self.fwd_pwm}.")
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

    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().info(f"--- STATE: {new_state} ---")

    def stop(self):
        rc_msg = OverrideRCIn()
        rc_msg.channels = [65535] * 18 
        # Butun eksenlerde kontrolu birak
        self.rc_pub.publish(rc_msg)
        
        if self.arm_client.wait_for_service(timeout_sec=0.5):
            req = CommandBool.Request()
            req.value = False
            self.arm_client.call_async(req)

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
        
        # LOGGING (20 hz)
        t_sec = float(now.nanoseconds) / 1e9
        dur = max(0.0, self.fwd_duration - elapsed) if self.state == 'FORWARD' else 0.0
        
        hdg_deg = math.degrees(self.heading_rad) if self.heading_rad is not None else 0.0
        
        self.csv_writer.writerow([
            f"{t_sec:.3f}", self.state,
            f"{hdg_deg:.2f}",
            f"{self.rel_alt if self.rel_alt is not None else 0.0:.2f}",
            f"{self.vfr_alt if self.vfr_alt is not None else 0.0:.2f}",
            self.thr_out, self.fwd_out, self.yaw_out, f"{dur:.2f}",
            f"{self.zed_x:.3f}", f"{self.zed_y:.3f}", f"{math.degrees(self.zed_yaw):.2f}",
            f"{self.cube_ax:.3f}", f"{self.cube_ay:.3f}", f"{self.cube_az:.3f}",
            f"{self.cube_gx:.3f}", f"{self.cube_gy:.3f}", f"{self.cube_gz:.3f}",
            f"{self.zed_ax:.3f}", f"{self.zed_ay:.3f}", f"{self.zed_az:.3f}",
            f"{self.zed_gx:.3f}", f"{self.zed_gy:.3f}", f"{self.zed_gz:.3f}",
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
            # Araç pozitif yüzerliliğe sahip olduğu için motorları kapatıp kendi çıkmasına izin veriyoruz
            self.get_logger().info('Motorlar kapatildi. Arac kendi yuzerliligi ile yuzeye cikiyor...')
            self.stop()
            self.change_state('DONE')
            return

        # STARTING ve ARMING sensör verisine ihtiyaç duymaz — MAVROS topic'leri
        # henüz yayınlanmamış olsa bile ARM süreci başlasın.
        if self.state == 'STARTING':
            has_hdg = self.heading_rad is not None
            has_alt = self.rel_alt is not None
            self.get_logger().info(f'Başlatılıyor... pusula={has_hdg}, rel_alt={has_alt}', throttle_duration_sec=2.0)
            if elapsed > 2.0:
                self.change_state('ARMING')
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
            rc_msg.channels[2] = 1500  # Throttle nötr
            rc_msg.channels[3] = 1500  # Yaw nötr
            rc_msg.channels[4] = 1500  # Forward nötr
            self.rc_pub.publish(rc_msg)
            
            if elapsed > 4.0:
                self.change_state('DIVING')
            return

        # DIVING ve FORWARD state'leri sensör verisine ihtiyaç duyar
        if self.heading_rad is None or self.rel_alt is None:
            self.get_logger().info(
                f'Pusula veya rel_alt verisi bekleniyor... (hdg={self.heading_rad is not None}, alt={self.rel_alt is not None})',
                throttle_duration_sec=2.0)
            return

        if self.state == 'DIVING':
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = 1400  # 1200 cok agresif oldugu icin 1400 olarak revize edildi
            rc_msg.channels[3] = 1500  
            rc_msg.channels[4] = 1500  
            rc_msg.channels[5] = 1500  
            self.rc_pub.publish(rc_msg)
            
            self.thr_out = 1400; self.fwd_out = 1500; self.yaw_out = 1500
            
            self.get_logger().info(f'Dalış: 1400 PWM, rel_alt: {self.rel_alt:.2f}m', throttle_duration_sec=2.0)
            
            if self.rel_alt < -1.0 or elapsed > 15.0:
                self.ref_heading_rad = self.heading_rad
                self.get_logger().info(f"Dalis bitti. Ref heading: {math.degrees(self.ref_heading_rad):.1f}")
                self.change_state('FORWARD')

        elif self.state == 'FORWARD':
            if elapsed >= self.fwd_duration:
                self.get_logger().info(f"Süre doldu! {self.fwd_duration} sn tamamlandı.")
                # self.stop() SURFACING icinde cagirilacak
                self.change_state('SURFACING')
                return
                
            depth_err = -1.0 - (self.rel_alt if self.rel_alt is not None else 0.0)
            pwm_thr = int(1450 + 300.0 * depth_err)
            pwm_thr = max(1300, min(1900, pwm_thr)) # Daha güçlü surface limiti eklendi (Pitch down baskısına karsi)
            
            # Yaw PID (CLOSED LOOP HEADING)
            if self.ref_heading_rad is None:
                self.ref_heading_rad = self.heading_rad
                
            yaw_err = normalize_angle(self.ref_heading_rad - self.heading_rad)
            yaw_err_deg = math.degrees(yaw_err)
            yaw_pwm_calc = int(1500 + (yaw_err_deg * self.k_heading_fwd))
            yaw_pwm_calc = max(1400, min(1600, yaw_pwm_calc))
            
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = pwm_thr
            rc_msg.channels[3] = yaw_pwm_calc
            rc_msg.channels[4] = self.fwd_pwm
            rc_msg.channels[5] = 1500 
            
            self.thr_out = pwm_thr; self.fwd_out = self.fwd_pwm; self.yaw_out = yaw_pwm_calc
            self.rc_pub.publish(rc_msg)
            
            self.get_logger().info(
                f"Düz Gidiliyor {self.fwd_duration - elapsed:.1f}s | Hdg Err: {yaw_err_deg:.1f} Yaw PWM: {yaw_pwm_calc} | Thr PWM: {pwm_thr}",
                throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = StraightBlindTestNode()

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
