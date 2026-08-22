#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
import math
import os
from datetime import datetime

def normalize_angle_deg(ang):
    while ang > 180.0:
        ang -= 360.0
    while ang <= -180.0:
        ang += 360.0
    return ang

class CompassTurnSequence(Node):
    def __init__(self):
        super().__init__('compass_turn_sequence')
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        
        self.hdg_sub = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.hdg_cb, sensor_qos)
        self.rel_alt_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.rel_alt_cb, sensor_qos)
        
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        
        self.state = 'STARTING'
        self.compass_hdg = 0.0
        self.rel_alt = 0.0
        
        self.state_start_time = self.get_clock().now()
        
        # Test parametreleri
        self.target_depth = -0.5
        self.fwd_pwm = 1700
        self.fwd1_time_2m = 6.0  # İlk gidiş 6 saniye
        self.fwd2_time_2m = 7.0  # Dönüşten sonraki gidiş 7 saniye (akıntı/ivme kaybı telafisi)
        self.fwd3_time_2m = 6.0  # Sola dönüşten sonraki son gidiş 6 saniye
        self.yaw_kp = 2.0
        self.turn_tolerance = 3.0 # derece
        
        self.target_hdg = 0.0
        self.target_hdg_set = False
        self.settled_start_time = None
        
        # Log dosyasi - script'in çalıştığı yere (veya ilgili workspace içerisine) yazıyoruz
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'compass_turn_log_{stamp}.csv')
        self.log_file = open(log_path, 'w')
        self.log_file.write("Time(s),State,RelAlt(m),Compass_Hdg(deg)\n")
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Compass Turn Sequence scripti baslatildi.")
        self.get_logger().info(f"Loglar {log_path} dosyasina kaydediliyor...")
        
    def hdg_cb(self, msg):
        self.compass_hdg = msg.data
        
    def rel_alt_cb(self, msg):
        self.rel_alt = msg.data
        
    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.target_hdg_set = False
        self.settled_start_time = None
        self.get_logger().info(f"--- STATE: {new_state} ---")
        
    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        
        t_sec = now.nanoseconds * 1e-9
        self.log_file.write(f"{t_sec:.3f},{self.state},{self.rel_alt:.2f},{self.compass_hdg:.2f}\n")
            
        # ---------- STATE MACHINE ----------
        if self.state == 'STARTING':
            if elapsed > 2.0:
                self.change_state('ARMING')
                
        elif self.state == 'ARMING':
            # Aracı ALT_HOLD moda alıp Arm edelim
            if elapsed < 0.5:
                if self.mode_client.wait_for_service(timeout_sec=0.1):
                    req = SetMode.Request()
                    req.custom_mode = 'ALT_HOLD'
                    self.mode_client.call_async(req)
                if self.arm_client.wait_for_service(timeout_sec=0.1):
                    req = CommandBool.Request()
                    req.value = True
                    self.arm_client.call_async(req)
            
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500
            rc.channels[3] = 1500
            rc.channels[4] = 1500
            self.rc_pub.publish(rc)

            if elapsed > 3.0:
                self.change_state('DIVING')
                
        elif self.state == 'DIVING':
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            # Dalış için throttle 1400 (PWM < 1500)
            rc.channels[2] = 1400
            rc.channels[3] = 1500
            rc.channels[4] = 1500
            self.rc_pub.publish(rc)
            
            if int(elapsed*10) % 10 == 0:
                self.get_logger().info(f"[DIVING] Derinlik: {self.rel_alt:.2f} m, Hedef: {self.target_depth} m")
                
            if self.rel_alt <= self.target_depth or elapsed > 15.0:
                # O anki derinliğe kilitlenmesi için tekrar ALT_HOLD atıyoruz
                if self.mode_client.wait_for_service(timeout_sec=0.1):
                    req = SetMode.Request()
                    req.custom_mode = 'ALT_HOLD'
                    self.mode_client.call_async(req)
                self.change_state('FWD_1')
                
        elif self.state == 'FWD_1':
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500
            rc.channels[3] = 1500
            rc.channels[4] = self.fwd_pwm
            self.rc_pub.publish(rc)
            
            if int(elapsed*10) % 10 == 0:
                self.get_logger().info(f"[FWD_1] Ileri gidiliyor... Kalan Sure: {self.fwd1_time_2m - elapsed:.1f} s")
                
            if elapsed > self.fwd1_time_2m:
                self.change_state('TURN_RIGHT')
                
        elif self.state == 'TURN_RIGHT':
            if not self.target_hdg_set:
                self.target_hdg = (self.compass_hdg + 90.0) % 360.0
                self.target_hdg_set = True
                self.get_logger().info(f"[TURN_RIGHT] Mevcut Yon: {self.compass_hdg:.1f}, Hedef Yon: {self.target_hdg:.1f}")
                
            yaw_err = normalize_angle_deg(self.target_hdg - self.compass_hdg)
            
            if abs(yaw_err) <= self.turn_tolerance:
                yaw_pwm = 1500
                if self.settled_start_time is None:
                    self.settled_start_time = elapsed
                elif (elapsed - self.settled_start_time) > 1.5:
                    self.change_state('FWD_2')
                    return
            else:
                self.settled_start_time = None
                # Sag donus icin pwm 1500 ustunde, prop kontrol:
                val = 1500 + int(yaw_err * self.yaw_kp)
                yaw_pwm = max(1350, min(1650, val)) # max 150 pwm offset
                
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500
            rc.channels[3] = yaw_pwm
            rc.channels[4] = 1500
            self.rc_pub.publish(rc)
            
            if int(elapsed*10) % 10 == 0:
                self.get_logger().info(f"[TURN_RIGHT] Hedef Hdg: {self.target_hdg:.1f}, Fark: {yaw_err:.1f}, PWM: {yaw_pwm}")

        elif self.state == 'FWD_2':
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500
            rc.channels[3] = 1500
            rc.channels[4] = self.fwd_pwm
            self.rc_pub.publish(rc)
            
            if int(elapsed*10) % 10 == 0:
                self.get_logger().info(f"[FWD_2] Ileri gidiliyor... Kalan Sure: {self.fwd2_time_2m - elapsed:.1f} s")
                
            if elapsed > self.fwd2_time_2m:
                self.change_state('TURN_LEFT')
                
        elif self.state == 'TURN_LEFT':
            if not self.target_hdg_set:
                self.target_hdg = (self.compass_hdg - 90.0) % 360.0
                if self.target_hdg < 0:
                    self.target_hdg += 360.0
                self.target_hdg_set = True
                self.get_logger().info(f"[TURN_LEFT] Mevcut Yon: {self.compass_hdg:.1f}, Hedef Yon: {self.target_hdg:.1f}")
                
            yaw_err = normalize_angle_deg(self.target_hdg - self.compass_hdg)
            
            if abs(yaw_err) <= self.turn_tolerance:
                yaw_pwm = 1500
                if self.settled_start_time is None:
                    self.settled_start_time = elapsed
                elif (elapsed - self.settled_start_time) > 1.5:
                    self.change_state('FWD_3')
                    return
            else:
                self.settled_start_time = None
                val = 1500 + int(yaw_err * self.yaw_kp)
                yaw_pwm = max(1350, min(1650, val))
                
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500
            rc.channels[3] = yaw_pwm
            rc.channels[4] = 1500
            self.rc_pub.publish(rc)
            
            if int(elapsed*10) % 10 == 0:
                self.get_logger().info(f"[TURN_LEFT] Hedef Hdg: {self.target_hdg:.1f}, Fark: {yaw_err:.1f}, PWM: {yaw_pwm}")

        elif self.state == 'FWD_3':
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500
            rc.channels[3] = 1500
            rc.channels[4] = self.fwd_pwm
            self.rc_pub.publish(rc)
            
            if int(elapsed*10) % 10 == 0:
                self.get_logger().info(f"[FWD_3] Ileri gidiliyor... Kalan Sure: {self.fwd3_time_2m - elapsed:.1f} s")
                
            if elapsed > self.fwd3_time_2m:
                self.change_state('HOLD_DEPTH')

        elif self.state == 'HOLD_DEPTH':
            if elapsed < 0.2:
                # Modumuzu surekli ALT_HOLD olarak temin et
                if self.mode_client.wait_for_service(timeout_sec=0.1):
                    req = SetMode.Request()
                    req.custom_mode = 'ALT_HOLD'
                    self.mode_client.call_async(req)
                
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500 # Throttle ortada (Derinlik koruma)
            rc.channels[3] = 1500
            rc.channels[4] = 1500
            self.rc_pub.publish(rc)
            
            if int(elapsed*10) % 10 == 0:
                self.get_logger().info("[HOLD_DEPTH] Gorev tamamlandi, ALT HOLD modunda konum korunarak bekleniyor...")

def main(args=None):
    rclpy.init(args=args)
    node = CompassTurnSequence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.get_logger().info("Test durduruluyor, RC kanallari temizleniyor...")
        try:
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            node.rc_pub.publish(rc)
        except Exception:
            pass
        try:
            node.log_file.close()
        except:
            pass
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
