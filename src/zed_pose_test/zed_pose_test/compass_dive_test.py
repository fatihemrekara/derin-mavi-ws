#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import signal
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn, VfrHud
from mavros_msgs.srv import CommandBool, SetMode
import math
import time

class CompassDiveLogger(Node):
    def __init__(self):
        super().__init__('compass_dive_logger')
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        
        self.hdg_sub = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.hdg_cb, sensor_qos)
        self.vfr_sub = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.alt_cb, sensor_qos)
        self.rel_alt_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.rel_alt_cb, sensor_qos)
        
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        
        self.state = 'STARTING'
        self.compass_hdg = 0.0
        self.altitude = 0.0
        self.rel_alt = 0.0
        self.shutting_down = False
        
        self.state_start_time = self.get_clock().now()
        
        # ========== DALIS PARAMETRELERI ==========
        self.DIVE_THROTTLE = 1250     # Dalış thrust değeri (1500=nötr, düşük=aşağı).
        self.TARGET_DEPTH = -1.0      # Hedef derinlik (metre, negatif = su altı)
        self.DIVE_TIMEOUT = 15.0      # Maksimum dalış süresi (saniye)
        self.HOLD_DURATION = 5.0      # Derinlikte bekleme süresi (saniye)
        # =========================================
        
        # Log dosyasina yazmak icin CSV açıyoruz
        import os
        from datetime import datetime
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(os.path.expanduser('~'), f'compass_dive_log_{stamp}.csv')
        self.log_file = open(log_path, 'w')
        self.log_file.write("Time(s),State,RelAlt(m),Compass_Hdg(deg)\n")
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Compass Dive Logger scripti baslatildi.")
        self.get_logger().info(f"Loglar {log_path} dosyasina kaydediliyor...")
        self.get_logger().info(f"Dalış throttle: {self.DIVE_THROTTLE}, Hedef derinlik: {self.TARGET_DEPTH} m")
        
    def hdg_cb(self, msg):
        self.compass_hdg = msg.data
        
    def alt_cb(self, msg):
        self.altitude = msg.altitude
        
    def rel_alt_cb(self, msg):
        self.rel_alt = msg.data
        
    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().info(f"--- STATE: {new_state} ---")
    
    def emergency_stop(self):
        """Acil durum: DISARM + RC release. Kod kapansa bile araç durur."""
        if self.shutting_down:
            return
        self.shutting_down = True
        self.get_logger().warn("!!! ACIL DURDURMA - DISARM ediliyor !!!")
        
        # RC kanallarini release et
        for _ in range(5):
            try:
                rc = OverrideRCIn()
                rc.channels = [65535] * 18
                self.rc_pub.publish(rc)
            except Exception:
                pass
        
        # DISARM komutu gönder
        try:
            if self.arm_client.wait_for_service(timeout_sec=1.0):
                req = CommandBool.Request()
                req.value = False
                future = self.arm_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                self.get_logger().info("DISARM komutu gonderildi.")
        except Exception as e:
            self.get_logger().error(f"DISARM hatasi: {e}")
        
        # Log dosyasini kapat
        try:
            self.log_file.flush()
            self.log_file.close()
        except Exception:
            pass
        
    def control_loop(self):
        if self.shutting_down:
            return
            
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        
        # Terminale 1 saniyede bir log bas
        if int(elapsed * 10) % 10 == 0:
            self.get_logger().info(f"[{self.state}] Derinlik: {self.rel_alt:.2f} m, Pusula: {self.compass_hdg:.2f} deg")
            
        # CSV dosyasina 10 Hz'de kaydet (0.1 sn'de bir)
        t_sec = now.nanoseconds * 1e-9
        self.log_file.write(f"{t_sec:.2f},{self.state},{self.rel_alt:.2f},{self.compass_hdg:.2f}\n")
        
        # ---------- STATE MACHINE ----------
        if self.state == 'STARTING':
            if elapsed > 2.0:
                self.change_state('ARMING')
                
        elif self.state == 'ARMING':
            # Her 2 saniyede bir ARM + mod komutunu tekrar gönder
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
                self.get_logger().info(f"ALT_HOLD + ARM komutu gonderildi (elapsed={elapsed:.1f}s)")
            
            # Güvenlik için boş PWM gönderelim
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500  # Throttle nötr
            rc.channels[3] = 1500  # Yaw nötr
            rc.channels[4] = 1500  # Fwd nötr
            self.rc_pub.publish(rc)

            if elapsed > 4.0:
                self.change_state('DIVING')
                
        elif self.state == 'DIVING':
            # Sabit 1200 PWM ile dalış — basınç sensörü hedef derinliği okuyana kadar
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1200  # Sabit dalış gücü
            rc.channels[3] = 1500  # Yaw neutral
            rc.channels[4] = 1500  # Fwd neutral
            self.rc_pub.publish(rc)
            
            self.get_logger().info(f"  Dalis: 1200 PWM, rel_alt={self.rel_alt:.2f}m", throttle_duration_sec=2.0)
            
            # Basınç sensörü hedef derinliği okuyana kadar veya timeout
            if self.rel_alt < self.TARGET_DEPTH or elapsed > self.DIVE_TIMEOUT:
                reason = "hedef derinlik" if self.rel_alt < self.TARGET_DEPTH else "zaman asimi"
                self.get_logger().info(f"Dalis tamamlandi ({reason}). ALT_HOLD derinligi koruyacak.")
                self.change_state('HOLDING')
                
        elif self.state == 'HOLDING':
            # ALT_HOLD modunda throttle 1500 = Orange Cube derinliği korusun
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500  # Nötr — ALT_HOLD derinliği kendi koruyor
            rc.channels[3] = 1500
            rc.channels[4] = 1500
            self.rc_pub.publish(rc)
            
            self.get_logger().info(f"ALT_HOLD derinlik koruması: rel_alt={self.rel_alt:.2f}m", throttle_duration_sec=2.0)
            
            if elapsed > self.HOLD_DURATION:
                self.change_state('STOPPING_MOTORS')
                
        elif self.state == 'STOPPING_MOTORS':
            # Derinlikte bekledikten sonra DISARM et
            if elapsed < 0.5:
                if self.arm_client.wait_for_service(timeout_sec=0.5):
                    req = CommandBool.Request()
                    req.value = False  # DISARM - Pervaneler tamamen durur
                    self.arm_client.call_async(req)
                    self.get_logger().info("DISARM komutu gonderildi")
                
            # RC sinyallerini de tamamen bosta bırakalım
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            self.rc_pub.publish(rc)
            
            if elapsed > 1.0:
                self.change_state('SURFACING_AND_LOGGING')
            
        elif self.state == 'SURFACING_AND_LOGGING':
            # Araç DISARM edildi, pozitif kaldırma kuvvetiyle yüzeye çıkacak.
            # Ctrl+C yapana kadar log kaydına devam eder.
            pass

# Global referans (signal handler icin)
_node = None

def _signal_handler(sig, frame):
    """SIGINT/SIGTERM yakalandığında aracı durdur."""
    global _node
    if _node is not None:
        _node.emergency_stop()
    sys.exit(0)

def main(args=None):
    global _node
    rclpy.init(args=args)
    node = CompassDiveLogger()
    _node = node
    
    # Signal handler'ları kur - kod her türlü kapatılsa bile araç durur
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Beklenmeyen hata: {e}")
    finally:
        node.emergency_stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

