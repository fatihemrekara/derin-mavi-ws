#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Görev Yöneticisi (Mission Manager) - İleri Kategori Kare ve Daire Rotası
----------------------------------------------------------------------
Bu node, aracı daldırıp (Depth Hold), parçalı rotaları (Path) yayınlar
ve aralardaki keskin dönüşleri doğrudan RC Override ile yapar.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mavros_msgs.msg import OverrideRCIn, VFR_HUD
from mavros_msgs.srv import CommandBool, SetMode

def quat_to_yaw(ox, oy, oz, ow):
    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    return math.atan2(siny_cosp, cosy_cosp)

def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

def normalize_angle(a):
    while a > math.pi: a -= 2.0 * math.pi
    while a <= -math.pi: a += 2.0 * math.pi
    return a

class VideoTaskNode(Node):
    def __init__(self):
        super().__init__('videotask')
        
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('rc_topic', '/mavros/rc/override')
        self.declare_parameter('path_topic', '/planned_route')
        
        pose_topic = self.get_parameter('pose_topic').value
        rc_topic = self.get_parameter('rc_topic').value
        path_topic = self.get_parameter('path_topic').value
        
        # QoS for Path (Latched)
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
            
        self.path_pub = self.create_publisher(Path, path_topic, latched_qos)
        self.rc_pub = self.create_publisher(OverrideRCIn, rc_topic, 10)
        
        # QoS for Pose (MAVROS local_position is usually RELIABLE)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
            
        self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self.on_pose, sensor_qos)
        self.vfr_sub = self.create_subscription(VFR_HUD, '/mavros/vfr_hud', self.on_vfr, sensor_qos)
        
        # MAVROS Servisleri
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.pose = None # (x, y, yaw)
        self.alt = 0.0 # Depth/Altitude from VFR_HUD
        self.state = 'INIT'
        self.state_start_time = None
        self.target_yaw = 0.0
        self.current_target_pt = None
        
        self.timer = self.create_timer(0.1, self.loop)
        
        self.get_logger().info("VideoTask Node Başlatıldı. Pozisyon bekleniyor...")

    def on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        self.pose = (p.x, p.y, yaw)
        
        if self.state == 'INIT':
            self.state = 'ARMING'
            self.get_logger().info("İlk pozisyon alındı, göreve başlanıyor.")

    def on_vfr(self, msg: VFR_HUD):
        self.alt = msg.altitude

    def set_mode_and_arm(self):
        # ArduSub için DEPTH_HOLD modu ayarlanır
        if self.mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = 'ALT_HOLD' # ArduSub'da DEPTH_HOLD = ALT_HOLD
            self.mode_client.call_async(req)
        
        if self.arm_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = True
            self.arm_client.call_async(req)

    def send_rc(self, forward=1500, yaw=1500, throttle=1500):
        msg = OverrideRCIn()
        msg.channels = [65535] * 18
        msg.channels[2] = throttle  # Z ekseni
        msg.channels[3] = yaw       # Dönüş
        msg.channels[4] = forward   # İleri
        self.rc_pub.publish(msg)

    def publish_path(self, pts):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            path.poses.append(ps)
        self.path_pub.publish(path)
        
        if pts:
            self.current_target_pt = pts[-1]

    def loop(self):
        if self.pose is None:
            return
            
        x, y, yaw = self.pose
        now = self.get_clock().now()
        
        def elapsed():
            if self.state_start_time is None: return 0.0
            return (now - self.state_start_time).nanoseconds * 1e-9
            
        def change_state(new_state):
            self.get_logger().info(f"Durum Değişimi: {self.state} -> {new_state}")
            self.state = new_state
            self.state_start_time = now
            # Rotayı temizle
            self.publish_path([])

        # ================= STATE MACHINE =================
        if self.state == 'ARMING':
            self.set_mode_and_arm()
            change_state('DIVE')

        elif self.state == 'DIVE':
            # Dalış: Kanal 3'e 1400 (Aşağı in) gönder
            self.send_rc(throttle=1400)
            
            # Zaman (8 saniye) veya derinlik mantığı ile inildiğinde dur
            # Note: The alt value could be positive or negative depending on barometer zeroing. 
            # Fallback to elapsed time of 8 seconds if altitude doesn't reach threshold.
            if elapsed() > 8.0 or self.alt < -1.0:
                self.send_rc(throttle=1500) # Dur
                # LEG 1 Rotası (0,0 -> 0,5) Y ekseni (Kuzey)
                self.publish_path([(x, y), (x, y + 5.0)])
                change_state('LEG_1')

        elif self.state == 'LEG_1':
            # Path follower aracı sürüyor, hedefe varılıp varılmadığına bak
            dist = math.hypot(self.current_target_pt[0] - x, self.current_target_pt[1] - y)
            if dist < 0.5:
                self.target_yaw = normalize_angle(yaw - math.pi/2) # 90 derece sağ
                change_state('TURN_1')

        elif self.state == 'TURN_1':
            err = normalize_angle(self.target_yaw - yaw)
            if abs(err) < math.radians(5.0):
                self.send_rc(yaw=1500)
                # LEG 2 Rotası (Doğuya 5m)
                self.publish_path([(x, y), (x + 5.0, y)])
                change_state('LEG_2')
            else:
                # ArduSub RC 4 (Yaw): 1600 is Right turn.
                # If target is to the right, err is negative (yaw > target_yaw).
                # We subtract err from 1500. So 1500 - (-val) = 1500 + val.
                pwm = 1500 - int((err / math.pi) * 400)
                self.send_rc(yaw=max(1100, min(1900, pwm)))

        elif self.state == 'LEG_2':
            dist = math.hypot(self.current_target_pt[0] - x, self.current_target_pt[1] - y)
            if dist < 0.5:
                # Daire Rotası: (x, y) noktasından girip 450 derece dönecek şekilde saat yönünde
                cx, cy = x, y - 0.5 # Merkez noktası, yarıçap 0.5m (Çap 1m)
                pts = []
                for i in range(1, 46): # 45 adım -> 450 derece
                    a = math.pi/2 - (math.radians(10.0) * i)
                    pts.append((cx + 0.5 * math.cos(a), cy + 0.5 * math.sin(a)))
                self.publish_path(pts)
                change_state('CIRCLE')

        elif self.state == 'CIRCLE':
            dist = math.hypot(self.current_target_pt[0] - x, self.current_target_pt[1] - y)
            if dist < 0.5:
                # LEG 3 Rotası (Güneye doğru ilerle - (5.5, 4.5) noktasından vb.)
                # Circle bitiş noktası = cx + 0.5*cos(-4.5 rad), cy + 0.5*sin(-4.5 rad)
                # Basitleştirmek adına x,y yerine rotayı uzatalım
                self.publish_path([(x, y), (x, y - 5.0)])
                change_state('LEG_3')

        elif self.state == 'LEG_3':
            dist = math.hypot(self.current_target_pt[0] - x, self.current_target_pt[1] - y)
            if dist < 0.5:
                self.target_yaw = normalize_angle(yaw - math.pi/2) # 90 derece sağ
                change_state('TURN_2')

        elif self.state == 'TURN_2':
            err = normalize_angle(self.target_yaw - yaw)
            if abs(err) < math.radians(5.0):
                self.send_rc(yaw=1500)
                # LEG 4 Rotası (Batıya, başlangıca doğru)
                self.publish_path([(x, y), (x - 5.5, y)])
                change_state('LEG_4')
            else:
                pwm = 1500 - int((err / math.pi) * 400)
                self.send_rc(yaw=max(1100, min(1900, pwm)))

        elif self.state == 'LEG_4':
            dist = math.hypot(self.current_target_pt[0] - x, self.current_target_pt[1] - y)
            if dist < 0.5:
                change_state('SURFACE')

        elif self.state == 'SURFACE':
            self.send_rc(throttle=1600) # Yukarı
            if elapsed() > 8.0:
                self.send_rc(throttle=1500)
                # Disarm
                if self.arm_client.wait_for_service(timeout_sec=1.0):
                    req = CommandBool.Request()
                    req.value = False
                    self.arm_client.call_async(req)
                change_state('DONE')

        elif self.state == 'DONE':
            pass

def main(args=None):
    rclpy.init(args=args)
    node = VideoTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
