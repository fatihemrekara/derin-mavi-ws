#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import csv
import datetime
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import Path
from mavros_msgs.msg import OverrideRCIn, VfrHud
from std_msgs.msg import Float64
from mavros_msgs.srv import CommandBool, SetMode

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
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('k_heading', 1.8)
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
            'rel_alt', 'vfr_alt', 'fwd_pwm', 'yaw_pwm', 'duration_left'
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
        """VFR HUD'dan bodoslama EKF compass header'ı gelir."""
        self.heading_rad = math.radians(msg.heading)
        self.vfr_alt = msg.altitude
        
    def on_rel_alt(self, msg: Float64):
        """Global Position'dan relative_alt degeri gelir. Su altina inis karsi yonlu (-)"""
        self.rel_alt = msg.data

    def stop(self):
        rc_msg = OverrideRCIn()
        rc_msg.channels = [65535] * 18
        rc_msg.channels[2] = 1500  # Depth hold neutral
        rc_msg.channels[3] = 1500
        rc_msg.channels[4] = 1500
        self.fwd_out = 1500
        self.yaw_out = 1500
        self.rc_pub.publish(rc_msg)

    def control_loop(self):
        now = self.get_clock().now()
        
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
                self.fwd_out, self.yaw_out, f"{dur:.2f}"
            ])
            self.log_file.flush()

        if self.state in ['WAITING_FOR_PATH', 'DONE']:
            return
            
        if self.heading_rad is None or self.rel_alt is None:
            self.get_logger().info('Pusula (vfr_hud) veya rel_alt verisi bekleniyor...', throttle_duration_sec=2.0)
            return

        if self.state == 'ARMING':
            if self.arm_client.wait_for_service(timeout_sec=1.0):
                req = CommandBool.Request()
                req.value = True
                self.arm_client.call_async(req)
            self.state = 'DIVING'
            self.dive_start_time = now
            self.get_logger().info('Araç Arm edildi. Dalış moduna geçiliyor...')
            return
            
        if self.state == 'DIVING':
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = 1400  # Dalış Gücü (aşağı bastır)
            self.fwd_out = 1500; self.yaw_out = 1500
            self.rc_pub.publish(rc_msg)
            
            elapsed = (now - self.dive_start_time).nanoseconds * 1e-9
            # rel_alt negatif yonde artiyorsa (su altina indiysek) -1 yapalim
            if elapsed > 15.0 or self.rel_alt < -1.0:
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
            pwm_yaw = int(1500 - (w / self.w_max) * 400) if self.w_max > 0 else 1500
            
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = 1500
            rc_msg.channels[3] = max(1100, min(1900, pwm_yaw))
            rc_msg.channels[4] = 1500  # Stop fwd
            
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
                
            # İleri giderken ayni zamanda heading correction yapmaya devam edelim
            w = max(-self.w_max, min(self.w_max, self.k_heading * heading_err))
            pwm_yaw = int(1500 - (w / self.w_max) * 400) if self.w_max > 0 else 1500
            
            rc_msg = OverrideRCIn()
            rc_msg.channels = [65535] * 18
            rc_msg.channels[2] = 1500
            rc_msg.channels[3] = max(1100, min(1900, pwm_yaw))
            rc_msg.channels[4] = self.fwd_pwm
            
            self.fwd_out = self.fwd_pwm
            self.yaw_out = rc_msg.channels[3]
            self.rc_pub.publish(rc_msg)
            self.get_logger().info(f"Düz gidiliyor... {self.target_duration - elapsed:.1f} s kaldı. Pwm(Fwd, Yaw): ({self.fwd_pwm}, {self.yaw_out})", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = BlindPathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        if not node.log_file.closed:
            node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
