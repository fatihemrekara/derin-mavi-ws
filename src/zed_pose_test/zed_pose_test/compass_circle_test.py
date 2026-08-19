#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn, VfrHud
from mavros_msgs.srv import CommandBool, SetMode

class CompassCircleLogger(Node):
    def __init__(self):
        super().__init__('compass_circle_logger')
        
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
        
        self.state_start_time = self.get_clock().now()
        
        # Log dosyasina yazmak icin CSV açıyoruz
        import os
        from datetime import datetime
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'compass_circle_log_{stamp}.csv')
        self.log_file = open(log_path, 'w')
        self.log_file.write("Time(s),State,RelAlt(m),Compass_Hdg(deg)\n")
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Compass Circle Logger scripti baslatildi.")
        self.get_logger().info(f"Loglar {log_path} dosyasina kaydediliyor...")
        
    def hdg_cb(self, msg):
        self.compass_hdg = msg.data
        
    def alt_cb(self, msg):
        self.altitude = msg.altitude
        
    def rel_alt_cb(self, msg):
        self.rel_alt = msg.data
        
    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().info(f"--- STATE DEĞİŞTİ: {new_state} ---")
        
    def control_loop(self):
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
            # Aracı MANUAL moda alıp Arm edelim
            if elapsed < 0.2:
                if self.mode_client.wait_for_service(timeout_sec=0.5):
                    req = SetMode.Request()
                    req.custom_mode = 'ALT_HOLD'
                    self.mode_client.call_async(req)
                if self.arm_client.wait_for_service(timeout_sec=0.5):
                    req = CommandBool.Request()
                    req.value = True
                    self.arm_client.call_async(req)
            
            # Guvenlik icin bos PWM gonderelim
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500
            rc.channels[3] = 1500
            rc.channels[4] = 1500
            self.rc_pub.publish(rc)

            if elapsed > 2.0:
                self.change_state('DIVING')
                
        elif self.state == 'DIVING':
            # Batırmak için throttle (Kanal 3) 1400'e çekiliyor (Kanal dizini 2)
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1400  # Dalış (throttle = 1400)
            rc.channels[3] = 1500  # Yaw neutral
            rc.channels[4] = 1500  # Fwd neutral
            self.rc_pub.publish(rc)
            
            if self.rel_alt < -1.0 or self.altitude < -1.0 or elapsed > 15.0:
                if self.mode_client.wait_for_service(timeout_sec=0.1):
                    req = SetMode.Request()
                    req.custom_mode = 'ALT_HOLD'
                    self.mode_client.call_async(req)
                self.change_state('MOVING_FORWARD')
                
        elif self.state == 'MOVING_FORWARD':
            # 5 saniye düz gidiyoruz
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500  # Derinlik koruma (throttle=1500)
            rc.channels[3] = 1500  # Yaw neutral
            rc.channels[4] = 1750  # Ileri hareket (Fwd = 1600)
            self.rc_pub.publish(rc)
            
            if elapsed > 5.0:
                self.change_state('CIRCLING')
                
        elif self.state == 'CIRCLING':
            # Yaklasik 2 metre yaricapinda bir cember icin
            # İleri itki (1600) ve sag-sol donus (Yaw = 1600 - saga donus) 
            # ikisini ayni anda verirsen bir yay/cember cizer.
            # Rota takip sistemi (kapalı çevrim) kapalı olduğu için pwm ile açık çevrim sürüyoruz. Yarıçap yaklaşık olacaktır.
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500 
            rc.channels[3] = 1630  # Yaw (Sağa dönüş itkisi)
            rc.channels[4] = 1670  # Fwd (İleri itkisi)
            self.rc_pub.publish(rc)
            
            # Cember cizmesi icin ~15 saniye bu sekilde donsun (1 tam tura yakin bir egri olusturmasi icin)
            if elapsed > 15.0:
                self.change_state('STOPPING_MOTORS')
                
        elif self.state == 'STOPPING_MOTORS':
            # Disarm et, pervaneler kapansin. Su yuzune cikacak.
            if elapsed < 0.2:
                if self.arm_client.wait_for_service(timeout_sec=0.5):
                    req = CommandBool.Request()
                    req.value = False  # DISARM
                    self.arm_client.call_async(req)
                
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            self.rc_pub.publish(rc)
            
            if elapsed > 1.0:
                self.change_state('SURFACING_AND_LOGGING')
            
        elif self.state == 'SURFACING_AND_LOGGING':
            # Motor durdu, yuzeye cikiyor. Sadece pusula kaydi alinmaya devam ediliyor. 
            # Kod 'Ctrl+C' yapilana dek bu asama surer, kayit kesilmez.
            pass

def main(args=None):
    rclpy.init(args=args)
    node = CompassCircleLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Cember testi kapatiliyor, RC kanallari bosaltiliyor...")
        try:
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            node.rc_pub.publish(rc)
        except Exception:
            pass
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
