#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nozed_test.py - ZED kamerası olmadan, PWM (Dead Reckoning) ve EKF Hız (Velocity)
verilerinin füzyonu ile çalışan test scripti.
video_gorevi.py'nin sağlamlığını tutar, ancak RViz2'de izlenebilmesi için
tahmini bir /nozed/fused_pose ve /nozed/fused_path yayınlar.
Son bacakta otomatik olarak başlangıç noktasına (0,0) dönmeye çalışır.
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import OverrideRCIn, State
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path

# ---------------- Ayarlar ----------------
STRAIGHT_T   = 17.0      
CIRCLE_T     = 25.0      
TURN_TOL     = 3.0       # derece
FWD_PWM      = 1620
CIRCLE_FWD   = 1650
CIRCLE_YAW   = 1570      
CH_THR, CH_YAW, CH_FWD = 2, 3, 4
NEUTRAL = 1500

MAX_SPEED_EST = 1.0      # 1900 PWM'in yaklaşık kaç m/s'ye denk geldiğine dair varsayım (Ayarlanabilir)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def ang_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0

def quat_from_yaw(yaw_rad):
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)

class NoZedMission(Node):
    def __init__(self):
        super().__init__('nozed_mission')
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
            
        # State değişkenleri
        self.hdg = None       # derece 0-360
        self.depth = 0.0      # metre
        self.state = State()
        
        # Konum Tespiti (Füzyon) Değişkenleri
        self.x = 0.0
        self.y = 0.0
        self.ekf_vel = 0.0
        self.current_fwd_pwm = NEUTRAL
        self.last_time = time.time()
        
        self.fused_path_msg = Path()
        self.fused_path_msg.header.frame_id = 'map'

        # Subscribers
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.cb_hdg, sensor_qos)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.cb_depth, sensor_qos)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.cb_vel, sensor_qos)
        self.create_subscription(State, '/mavros/state', self.cb_state, 10)
        
        # Publishers & Services
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/nozed/fused_pose', 10)
        self.path_pub = self.create_publisher(Path, '/nozed/fused_path', 10)
        
        self.cli_arm  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        
        # Füzyon Döngüsü (20 Hz)
        self.timer = self.create_timer(0.05, self.fusion_loop)
        self.get_logger().info("nozed_test başlatıldı. Yalnızca IMU, Derinlik ve PWM Füzyonu kullanılacak.")

    # ---------------- Callbacks ----------------
    def cb_hdg(self, m):   
        self.hdg = m.data
    def cb_depth(self, m): 
        self.depth = m.data
    def cb_state(self, m): 
        self.state = m
    def cb_vel(self, m):
        # EKF'nin X ve Y hızlarından genlik bul (aracın anlık gidiş hızı)
        vx = m.twist.linear.x
        vy = m.twist.linear.y
        self.ekf_vel = math.hypot(vx, vy)
        
    # ---------------- Füzyon (Position Tracking) ----------------
    def fusion_loop(self):
        if self.hdg is None:
            return
            
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        if dt > 0.1: # Çok büyük atlamaları engelle
            dt = 0.05
            
        # Sürekli PWM'den V hesapla (Dead Reckoning)
        # Sadece ileri gidiyorsak hızı hesaba kat (geri gitme senaryosu varsa eklenebilir)
        v_pwm = 0.0
        if self.current_fwd_pwm > 1550:
            pwm_pct = (self.current_fwd_pwm - 1500) / 400.0 # 1900'de %100
            v_pwm = pwm_pct * MAX_SPEED_EST
            
        # Füzyon: PWM varsayımı %80, EKF tahmini %20 (IMU kaymasını minimize etmek için)
        # Eğer EKF hızı anlamsız yüksekse, sadece pwm'i kullan
        if self.ekf_vel > 2.0:
            v_fused = v_pwm
        else:
            v_fused = (v_pwm * 0.8) + (self.ekf_vel * 0.2)
            
        # Yönelim ve Konum Güncelleme
        yaw_rad = math.radians(self.hdg)
        self.x += v_fused * math.cos(yaw_rad) * dt
        self.y += v_fused * math.sin(yaw_rad) * dt
        
        # ROS Mesajlarını Yayınla
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        qx, qy, qz, qw = quat_from_yaw(yaw_rad)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)
        
        # Path
        self.fused_path_msg.poses.append(pose_msg)
        self.fused_path_msg.header.stamp = pose_msg.header.stamp
        # Ekranda çok fazla nokta birikmemesi için (1000 limit)
        if len(self.fused_path_msg.poses) > 1000:
            self.fused_path_msg.poses.pop(0)
        self.path_pub.publish(self.fused_path_msg)

    # ---------------- Yardımcılar ----------------
    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)
            
    def wait_hdg(self):
        while self.hdg is None:
            self.get_logger().info('Pusula verisi bekleniyor...')
            rclpy.spin_once(self, timeout_sec=0.5)
        return self.hdg
        
    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        fut = self.cli_mode.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        self.get_logger().info(f'Mod: {mode}')
        self.spin(0.5)
        
    def arm(self, val=True):
        req = CommandBool.Request()
        req.value = val
        fut = self.cli_arm.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        self.get_logger().info('ARMED' if val else 'DISARMED')
        self.spin(1.0)
        
    def rc(self, thr=NEUTRAL, yaw=NEUTRAL, fwd=NEUTRAL):
        self.current_fwd_pwm = int(fwd)
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[0] = NEUTRAL  
        msg.channels[1] = NEUTRAL  
        msg.channels[5] = NEUTRAL  
        msg.channels[CH_THR] = int(thr)
        msg.channels[CH_YAW] = int(yaw)
        msg.channels[CH_FWD] = int(fwd)
        self.rc_pub.publish(msg)
        
    def neutral(self, sec=1.0):
        t0 = time.time()
        while time.time() - t0 < sec:
            self.rc()
            rclpy.spin_once(self, timeout_sec=0.02)
            
    def align_to(self, target_hdg, timeout=30.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            e = ang_diff(target_hdg, self.hdg)
            if abs(e) < TURN_TOL:
                break
            mag = clamp(60 + abs(e) * 2.0, 0, 200)
            yaw = NEUTRAL + int(mag if e > 0 else -mag)
            self.rc(yaw=yaw)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.neutral(1.0)
        self.get_logger().info(f'Hizalandı: {self.hdg:.0f}° (hedef {target_hdg:.0f}°)')
        return target_hdg

    # ---------------- Hareket primitifleri ----------------
    def dive(self, sec=8.0):
        self.get_logger().info('Dalış...')
        t0 = time.time()
        while time.time() - t0 < sec:
            self.rc(thr=1400)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.neutral()
        self.get_logger().info(f'Dalış tamamlandı. Derinlik kilitlendi.')
        
    def surface(self, sec=6.0):
        self.get_logger().info('Yüzeye çıkılıyor...')
        t0 = time.time()
        while time.time() - t0 < sec:
            self.rc(thr=1600)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.neutral()
        
    def go_straight(self, sec, hold_hdg):
        self.get_logger().info(f'Düz gidiş {sec:.0f}s @ {hold_hdg:.0f}° | Anlık X:{self.x:.1f} Y:{self.y:.1f}')
        t0 = time.time()
        while time.time() - t0 < sec:
            e = ang_diff(hold_hdg, self.hdg)
            yaw = NEUTRAL + int(clamp(e * 4.0, -120, 120))
            self.rc(yaw=yaw, fwd=FWD_PWM)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.neutral(1.5)
        
    def turn_right_90(self):
        target = (self.hdg + 90.0) % 360.0
        self.get_logger().info(f'Sağa 90°: {self.hdg:.0f}° -> {target:.0f}°')
        self.align_to(target)
        return target
        
    def circle_360(self):
        target = (self.hdg + 90.0) % 360.0
        self.get_logger().info(f'Daire başlıyor. Çıkış hedefi: {target:.0f}°')
        prev = self.hdg
        total = 0.0
        t0 = time.time()
        while time.time() - t0 < CIRCLE_T + 15.0 and total < 355.0:
            total += abs(ang_diff(self.hdg, prev))
            prev = self.hdg
            self.rc(yaw=CIRCLE_YAW, fwd=CIRCLE_FWD)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.neutral(1.5)
        self.get_logger().info(f'Tur bitti. Çıkış hizalanıyor...')
        self.align_to(target)
        return target
        
    def go_home(self):
        self.get_logger().info(f'Başlangıca (0,0) dönülüyor. Mevcut: X:{self.x:.1f}, Y:{self.y:.1f}')
        # Hedef açıyı bul (0,0'a doğru)
        # y ekseni + olursa açı +, - olursa -. atan2(hedef_y - anlik_y, hedef_x - anlik_x)
        # Matematiksel atan2 açısını compass (0-360) formatına çevirecek veya aradaki farkı alıp align to edeceğiz.
        # atan2'den çıkan sonuç radian'dır. Doğrudan yaw_rad = atan2(-y, -x) olarak düşünelim.
        # Pusula açısı genellikle Kuzey 0, Doğu 90'dır. Ama ROS'da genelde Doğu 0, Kuzey 90'dır. 
        # Zaten ang_diff kullanacağız. Matematiksel X,Y hesaplanırken pusula (compass) açısı (derece) math.radians() içine konup sin/cos yapıldı.
        # Yani hdg=0 demek cos(0)=1, x ekseninde ilerliyor (Kuzey=X, Doğu=Y kabul edebiliriz fark etmez).
        # Geri dönüş açısı (Radyan):
        home_yaw = math.atan2(0.0 - self.y, 0.0 - self.x)
        home_hdg = (math.degrees(home_yaw) + 360) % 360
        
        self.get_logger().info(f'Dönüş Rotası (Tahmini Hedef Açı): {home_hdg:.0f}°')
        self.align_to(home_hdg)
        
        # Oraya kadar düz git
        t0 = time.time()
        timeout = 40.0 # max süre
        
        while time.time() - t0 < timeout:
            dist = math.hypot(0.0 - self.x, 0.0 - self.y)
            if dist < 1.5:  # 1.5 metre yaklaştıysa görev bitti
                self.get_logger().info(f'Başlangıca başarıyla varıldı! Hata Payı: {dist:.1f} m')
                break
                
            e = ang_diff(home_hdg, self.hdg)
            yaw = NEUTRAL + int(clamp(e * 4.0, -120, 120))
            self.rc(yaw=yaw, fwd=FWD_PWM)
            rclpy.spin_once(self, timeout_sec=0.02)
            
        self.neutral(1.5)

    # ---------------- Görev ----------------
    def run(self):
        for c in (self.cli_arm, self.cli_mode):
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{c.srv_name} servisi bekleniyor...')
                rclpy.spin_once(self, timeout_sec=0.1)
                
        while not self.state.connected:
            self.get_logger().info('FCU (Cube Orange) bağlantısı bekleniyor...')
            rclpy.spin_once(self, timeout_sec=0.1)
            
        self.wait_hdg()
        self.x = 0.0
        self.y = 0.0  # Başlangıç konumunu (0,0) eşitle
        self.last_time = time.time()
        
        # Görev Modu
        self.set_mode('ALT_HOLD') # Guided Mod SIKINTILI OLDUĞU İÇİN (Sizin Talebiniz)
        self.arm(True)
        self.dive()
        
        h = self.hdg
        # Rota
        self.go_straight(STRAIGHT_T, h)   
        h = self.turn_right_90()          
        
        self.go_straight(STRAIGHT_T, h)   
        h = self.circle_360()             
        
        self.go_straight(STRAIGHT_T, h)   
        h = self.turn_right_90()          
        
        # Artık son düzlükte başlangıca dön
        self.go_home()
        
        self.surface()
        self.arm(False)
        self.get_logger().info('Görev tamamlandı.')

def main():
    rclpy.init()
    node = NoZedMission()
    try:
        node.run()
    except KeyboardInterrupt:
        node.neutral(1.0)
        node.arm(False)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
