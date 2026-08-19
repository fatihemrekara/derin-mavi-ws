#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import csv
import datetime
import signal
import sys
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
    Ramer-Douglas-Peucker vb. kompleks algoritmalar yerine; 
    rotadaki noktalar arasi istikametlerine bakar, keskin dönüs yapilan 
    (>2 derece) yerleri makro kose (waypoint) olarak isaretler.  
    Boylelikle arc seklinde donus varsa bile ufak parcalara (adimlara) 
    bırakacak, ama duz cizgi uzerindeyse (ornegin square_route_planner.py 
    duz cizgide cok nokta atar) bunlari tek buyuk kosede toplayacaktir.
    """
    if len(pts) < 2: return pts
    simplified = [pts[0]]
    current_heading = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
    for i in range(1, len(pts)-1):
        dx = pts[i+1][0] - pts[i][0]
        dy = pts[i+1][1] - pts[i][1]
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            continue
        h = math.atan2(dy, dx)
        if abs(normalize_angle(h - current_heading)) > math.radians(heading_tol_deg):
            simplified.append(pts[i])
            current_heading = h
    simplified.append(pts[-1])
    return simplified


class BlindPathFollowerNode(Node):
    def __init__(self):
        super().__init__('blind_path_follower')
        
        self.declare_parameter('path_topic', '/planned_route')
        self.declare_parameter('rc_override_topic', '/mavros/rc/override')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('speed_mps', 0.4)       # Zaman hesabi icin hedef fwd hizi (yaklasik)
        self.declare_parameter('fwd_pwm', 1600)        # Duz ilerlerken verilecek ileri yon throttle'i
        self.declare_parameter('yaw_tolerance_deg', 15.0)
        self.declare_parameter('k_heading', 3.0)
        self.declare_parameter('w_max', 1.0)
        
        gp = lambda n: self.get_parameter(n).value
        self.speed_mps = float(gp('speed_mps'))
        self.fwd_pwm = int(gp('fwd_pwm'))
        self.yaw_tol = math.radians(float(gp('yaw_tolerance_deg')))
        self.k_heading = float(gp('k_heading'))
        self.w_max = float(gp('w_max'))
        rate = float(gp('control_rate_hz'))
        
        self.path_pts = []
        self.macro_segments = []
        self.wp_idx = 0
        
        self.heading_rad = None
        self.rel_alt = None
        self.vfr_alt = None
        
        # ZED logging vars
        self.zed_x = 0.0
        self.zed_y = 0.0
        self.zed_yaw = 0.0
        
        self.cube_ax = 0.0; self.cube_ay = 0.0; self.cube_az = 0.0
        self.cube_gx = 0.0; self.cube_gy = 0.0; self.cube_gz = 0.0
        
        self.zed_ax = 0.0; self.zed_ay = 0.0; self.zed_az = 0.0
        self.zed_gx = 0.0; self.zed_gy = 0.0; self.zed_gz = 0.0
        
        self.ekf_x = 0.0; self.ekf_y = 0.0; self.ekf_z = 0.0; self.ekf_yaw = 0.0
        
        self.zed_fps = 0.0
        self.zed_pose_count = 0
        self.last_fps_calc_time = None
        self.zed_diag_msg = "OK"
        
        self.state = 'WAITING_FOR_PATH'
        self.dive_start_time = None
        self.fwd_start_time = None
        
        self.fwd_out = 1500
        self.yaw_out = 1500
        
        # CSV Logging Setup
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = f'blind_follower_log_{stamp}.csv'
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            'timestamp', 'state', 'wp_idx', 
            'heading_deg', 'target_heading_deg', 
            'rel_alt', 'vfr_alt', 'fwd_pwm', 'yaw_pwm', 'duration_left',
            'zed_x', 'zed_y', 'zed_yaw_deg',
            'cube_ax', 'cube_ay', 'cube_az', 'cube_gx', 'cube_gy', 'cube_gz',
            'zed_ax', 'zed_ay', 'zed_az', 'zed_gx', 'zed_gy', 'zed_gz',
            'ekf_x', 'ekf_y', 'ekf_z', 'ekf_yaw_deg',
            'zed_fps', 'zed_diag'
        ])
        
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
            
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
            
        self.path_sub = self.create_subscription(Path, str(gp('path_topic')), self.on_path, latched)
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
        
        self.get_logger().info(f"Kör Sürüş (ZED'siz Pusula/IMU) Follower Başladı. Hız Tespiti:{self.speed_mps} m/s, Ileri PWM:{self.fwd_pwm}.")
        self.get_logger().info(f"Kayıt Dosyası: {self.log_filename}")

    def on_path(self, msg: Path):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if len(pts) < 2:
            self.stop()
            self.state = 'WAITING_FOR_PATH'
            return
            
        self.macro_segments = simplify_path(pts, 2.0)
        
        self.get_logger().info(f'Düz gidilecek makro köşeler çıkarıldı. Rota: {len(pts)} küçük wpt -> {len(self.macro_segments)} büyük wp')
        self.path_pts = pts
        self.wp_idx = 0
        if self.state in ['WAITING_FOR_PATH', 'DONE']:
            self.state = 'ARMING'

    def on_vfr(self, msg: VfrHud):
        """VFR HUD pusula verisi (NED -> 0 Kuzey, 90 Doğu). ROS planlayıcıları ENU (0 Doğu, 90 Kuzey) kullanır. NED'den ENU'ya çeviriyoruz."""
        ned_rad = math.radians(msg.heading)
        self.heading_rad = normalize_angle(math.pi/2.0 - ned_rad)
        self.vfr_alt = msg.altitude
        
    def on_rel_alt(self, msg: Float64):
        """Global Position'dan relative_alt degeri gelir. Su altina inis karsi yonlu (-)"""
        self.rel_alt = msg.data

    def on_zed_pose(self, msg: PoseStamped):
        """Sadece LOGLAMAK icin arka planda okunur, kontrole dahil edilmez."""
        self.zed_pose_count += 1
        self.zed_x = msg.pose.position.x
        self.zed_y = msg.pose.position.y
        o = msg.pose.orientation
        self.zed_yaw = quat_to_yaw(o.x, o.y, o.z, o.w)

    def on_ekf_pose(self, msg: PoseStamped):
        """Sadece LOGLAMAK icindir. Arac bu veriyi takip icin KULLANMAZ."""
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

    def stop(self):
        rc_msg = OverrideRCIn()
        # 0 göndermek MAVROS'ta RC override'ı pilot kumandasına (joystick) devretmek demektir!
        rc_msg.channels = [0] * 18 
        self.fwd_out = 1500
        self.yaw_out = 1500
        self.rc_pub.publish(rc_msg)
        
        # Otonom görev bittiğinde veya CTRL+C basıldığında KESİN durması için aracı DISARM (Motor Kapatma) yapıyoruz!
        if self.arm_client.wait_for_service(timeout_sec=0.5):
            req = CommandBool.Request()
            req.value = False
            self.arm_client.call_async(req)

    def control_loop(self):
        now = self.get_clock().now()
        
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
        if self.heading_rad is not None:
            t_sec = float(now.nanoseconds) / 1e9
            th = 0.0
            dur = 0.0
            if self.wp_idx < len(self.macro_segments)-1:
                cur = self.macro_segments[self.wp_idx]
                nxt = self.macro_segments[self.wp_idx+1]
                th = math.degrees(math.atan2(nxt[1]-cur[1], nxt[0]-cur[0]))
                if self.state == 'FORWARD' and self.fwd_start_time:
                    dur = max(0.0, self.target_duration - (now - self.fwd_start_time).nanoseconds * 1e-9)
                    
            self.csv_writer.writerow([
                f"{t_sec:.3f}", self.state, self.wp_idx,
                f"{math.degrees(self.heading_rad):.2f}", f"{th:.2f}",
                f"{self.rel_alt if self.rel_alt is not None else 0.0:.2f}",
                f"{self.vfr_alt if self.vfr_alt is not None else 0.0:.2f}",
                self.fwd_out, self.yaw_out, f"{dur:.2f}",
                f"{self.zed_x:.3f}", f"{self.zed_y:.3f}", f"{math.degrees(self.zed_yaw):.2f}",
                f"{self.cube_ax:.3f}", f"{self.cube_ay:.3f}", f"{self.cube_az:.3f}",
                f"{self.cube_gx:.3f}", f"{self.cube_gy:.3f}", f"{self.cube_gz:.3f}",
                f"{self.zed_ax:.3f}", f"{self.zed_ay:.3f}", f"{self.zed_az:.3f}",
                f"{self.zed_gx:.3f}", f"{self.zed_gy:.3f}", f"{self.zed_gz:.3f}",
                f"{self.ekf_x:.3f}", f"{self.ekf_y:.3f}", f"{self.ekf_z:.3f}", f"{math.degrees(self.ekf_yaw):.2f}",
                f"{self.zed_fps:.1f}", self.zed_diag_msg
            ])
            self.log_file.flush()

        if self.state in ['WAITING_FOR_PATH', 'DONE']:
            return
            
        if self.heading_rad is None or self.rel_alt is None:
            self.get_logger().info('Pusula (vfr_hud) veya rel_alt verisi bekleniyor...', throttle_duration_sec=2.0)
            return

        if self.state == 'ARMING':
            elapsed = (now - self.state_start_time).nanoseconds * 1e-9 if hasattr(self, 'state_start_time') else 0.0
            
            # İlk 0.2 saniye içinde mod ve arm komutlarını gönder (referans: compass_dive_test.py)
            if elapsed < 0.2:
                if not hasattr(self, '_arm_sent'):
                    if self.mode_client.wait_for_service(timeout_sec=0.5):
                        req = SetMode.Request()
                        req.custom_mode = 'ALT_HOLD'
                        self.mode_client.call_async(req)
                    if self.arm_client.wait_for_service(timeout_sec=0.5):
                        req = CommandBool.Request()
                        req.value = True
                        self.arm_client.call_async(req)
                    self._arm_sent = True
                    self.state_start_time = now
                    self.get_logger().info('Araç ALT_HOLD moduna alınıyor ve Arm ediliyor...')
            
            # 2 saniye boyunca güvenlik PWM'i gönder (ArduSub'ın modu kabul etmesini bekle)
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = 1500  # Throttle nötr
            rc_msg.channels[3] = 1500  # Yaw nötr
            rc_msg.channels[4] = 1500  # Forward nötr
            self.rc_pub.publish(rc_msg)
            
            if elapsed > 2.0:
                self.state = 'DIVING'
                self.dive_start_time = now
                self.get_logger().info('Araç hazır! Otonom dalış başlıyor (1400 PWM)...')
            return
            
        if self.state == 'DIVING':
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            
            # Tamamen çalışan referans koddaki (line_controller_node) eşleştirme
            rc_msg.channels[2] = 1400  # Dalış Gücü (Heave)
            rc_msg.channels[3] = 1500  # Yaw
            rc_msg.channels[4] = 1500  # Forward
            rc_msg.channels[5] = 1500  # Sway (Yanal - referans koda eklendiği gibi)
            
            self.fwd_out = 1500; self.yaw_out = 1500
            self.rc_pub.publish(rc_msg)
            
            elapsed = (now - self.dive_start_time).nanoseconds * 1e-9
            # rel_alt negatif yonde artiyorsa (su altina indiysek) -1 yapalim
            if elapsed > 30.0 or self.rel_alt < -1.0:
                if self.mode_client.wait_for_service(timeout_sec=1.0):
                    req = SetMode.Request()
                    req.custom_mode = 'ALT_HOLD'
                    self.mode_client.call_async(req)
                self.state = 'ROTATING'
                self.get_logger().info(f'Dalış tamamlandı (rel_alt: {self.rel_alt:.2f}m). Rota takibine başlanıyor!')
            return

        if self.wp_idx >= len(self.macro_segments) - 1:
            self.get_logger().info('HEDEFE ULAŞILDI. Araç durduruldu.')
            self.stop()
            self.state = 'DONE'
            return

        cur_p = self.macro_segments[self.wp_idx]
        nxt_p = self.macro_segments[self.wp_idx + 1]
        
        target_heading = math.atan2(nxt_p[1] - cur_p[1], nxt_p[0] - cur_p[0])
        dist = math.hypot(nxt_p[0] - cur_p[0], nxt_p[1] - cur_p[1])
        
        heading_err = normalize_angle(target_heading - self.heading_rad)
        
        if self.state == 'ROTATING':
            if abs(heading_err) <= self.yaw_tol:
                self.state = 'FORWARD'
                self.fwd_start_time = now
                self.target_duration = dist / self.speed_mps
                self.get_logger().info(f"Açı hizalandı. {dist:.1f} m için {self.target_duration:.1f} saniye ileri PWM:{self.fwd_pwm} yollanacak.")
                return
                
            w = max(-self.w_max, min(self.w_max, self.k_heading * heading_err))
            pwm_yaw = int(1500 - (w / self.w_max) * 200) if self.w_max > 0 else 1500
            
            # Aktif Derinlik Kontrolü (-1.0 metre hedefini korumak için)
            depth_err = -1.0 - (self.rel_alt if self.rel_alt is not None else 0.0)
            # Sabit pozitif batmazlığı yenmek için (Steady-State error) merkez 1450'ye çekildi!
            pwm_thr = int(1450 + 300.0 * depth_err) 
            
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = max(1300, min(1700, pwm_thr))
            rc_msg.channels[3] = max(1100, min(1900, pwm_yaw))
            rc_msg.channels[4] = 1500  # Stop fwd
            rc_msg.channels[5] = 1500  # Sway (Yanal Hareket Kilitli)
            
            self.fwd_out = 1500
            self.yaw_out = rc_msg.channels[3]
            self.rc_pub.publish(rc_msg)
            self.get_logger().info(f"Dönülüyor... Hata: {math.degrees(heading_err):.1f} deg", throttle_duration_sec=1.0)

        elif self.state == 'FORWARD':
            elapsed = (now - self.fwd_start_time).nanoseconds * 1e-9
            if elapsed >= self.target_duration:
                self.get_logger().info(f"Segment bitti! {self.target_duration:.1f} saniye doldu.")
                self.wp_idx += 1
                self.state = 'ROTATING'
                self.stop()
                return
                
            # İleri giderken baş açısı kontrolü tamamen kapalı (kullanıcı talebi)
            pwm_yaw = 1500
            
            # Aktif Derinlik Kontrolü (-1.0 metre hedefini korumak için)
            depth_err = -1.0 - (self.rel_alt if self.rel_alt is not None else 0.0)
            # Sabit pozitif batmazlığı yenmek için merkez 1450'ye çekildi
            pwm_thr = int(1450 + 300.0 * depth_err)
            
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = max(1300, min(1700, pwm_thr))
            rc_msg.channels[3] = max(1100, min(1900, pwm_yaw))
            rc_msg.channels[4] = self.fwd_pwm
            rc_msg.channels[5] = 1500  # Sway Kilitli
            
            self.fwd_out = self.fwd_pwm
            self.yaw_out = rc_msg.channels[3]
            self.rc_pub.publish(rc_msg)
            self.get_logger().info(f"Düz gidiliyor... {self.target_duration - elapsed:.1f} s kaldı. Pwm(Fwd, Yaw): ({self.fwd_pwm}, {self.yaw_out})", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = BlindPathFollowerNode()
    
    def sigint_handler(sig, frame):
        node.get_logger().info("CTRL+C Algılandı! Araç acil durduruluyor...")
        node.stop()
        if not node.log_file.closed:
            node.log_file.close()
        
        # ROS 2 tamamen kapanmadan önce mesajın gittiğinden emin olmak için kısa süre bekle
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
