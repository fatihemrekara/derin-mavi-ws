#!/usr/bin/env python3
# mission_ileri.py — ROS 2 + MAVROS (ArduSub / Cube Orange + Jetson)
# Bağımlılıklar: rclpy, mavros_msgs, std_msgs
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import OverrideRCIn, State
from std_msgs.msg import Float64
# ---------------- Ayarlar ----------------
STRAIGHT_T   = 17.0      # >= 15 sn (emniyet payı)
CIRCLE_T     = 25.0      # 1 tam tur için tahmini süre (havuzda kalibre et)
TURN_TOL     = 3.0       # derece
FWD_PWM      = 1620
CIRCLE_FWD   = 1650
CIRCLE_YAW   = 1570      # fwd/yaw oranı -> daire çapı (>= 1 m olacak şekilde ayarla)
# ArduSub RC kanal haritası (0-index): 2=throttle(dikey) 3=yaw 4=forward
CH_THR, CH_YAW, CH_FWD = 2, 3, 4
NEUTRAL = 1500
def clamp(v, lo, hi):
    return max(lo, min(hi, v))
def ang_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0
class Mission(Node):
    def __init__(self):
        super().__init__('mission_ileri')
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.hdg = None       # derece 0-360
        self.depth = 0.0      # metre
        self.state = State()
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg',
                                 self.cb_hdg, sensor_qos)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',
                                 self.cb_depth, sensor_qos)
        self.create_subscription(State, '/mavros/state', self.cb_state, 10)
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.cli_arm  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        # HATA BURADAYDI: Kilitlenen bekleme döngüsü buradan tamamen kaldırıldı!
    # ---------------- Callbacks ----------------
    def cb_hdg(self, m):   self.hdg = m.data
    def cb_depth(self, m): self.depth = m.data
    def cb_state(self, m): self.state = m
    # ---------------- Yardımcılar ----------------
    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)
    def wait_hdg(self):
        while self.hdg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
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
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[0] = NEUTRAL  # Pitch kilitli
        msg.channels[1] = NEUTRAL  # Roll kilitli
        msg.channels[5] = NEUTRAL  # Lateral (Yengeç) kilitli
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
        self.neutral(1.5)
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
    def go_straight(self, sec, hold_hdg):
        self.get_logger().info(f'Düz gidiş {sec:.0f}s @ {hold_hdg:.0f}°')
        t0 = time.time()
        while time.time() - t0 < sec:
            e = ang_diff(hold_hdg, self.hdg)
            yaw = NEUTRAL + int(clamp(e * 4.0, -120, 120))
            self.rc(yaw=yaw, fwd=FWD_PWM)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.neutral(1.5)
    def turn_right_90(self):
        start = self.hdg
        target = (start + 90.0) % 360.0
        self.get_logger().info(f'Sağa 90°: {start:.0f}° -> {target:.0f}°')
        self.align_to(target)
        return target
    def circle_360(self):
        entry = self.hdg
        exit_hdg = (entry + 90.0) % 360.0
        self.get_logger().info(f'Daire başlıyor. Giriş: {entry:.0f}° -> Çıkış hedefi: {exit_hdg:.0f}°')
        prev = self.hdg
        total = 0.0
        t0 = time.time()
        while time.time() - t0 < CIRCLE_T + 15.0 and total < 355.0:
            total += abs(ang_diff(self.hdg, prev))
            prev = self.hdg
            self.rc(yaw=CIRCLE_YAW, fwd=CIRCLE_FWD)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.neutral(1.5)
        self.get_logger().info(f'Tur tamam: {total:.0f}°. Çıkış heading hizalanıyor...')
        self.align_to(exit_hdg)
        return exit_hdg
    # ---------------- Görev ----------------
    def run(self):
        # YENİ DÜZENLEME: Servisleri beklerken ağı dinliyoruz (spin_once)
        for c in (self.cli_arm, self.cli_mode):
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{c.srv_name} servisi bekleniyor...')
                rclpy.spin_once(self, timeout_sec=0.1)
        while not self.state.connected:
            self.get_logger().info('FCU (Cube Orange) bağlantısı bekleniyor...')
            rclpy.spin_once(self, timeout_sec=0.1)
        self.wait_hdg()
        self.set_mode('ALT_HOLD')
        self.arm(True)
        self.dive()
        h = self.hdg
        self.go_straight(STRAIGHT_T, h)   # 1) 15 sn düz
        h = self.turn_right_90()          # 2) sağa 90°
        self.go_straight(STRAIGHT_T, h)   # 3) 15 sn düz
        h = self.circle_360()             # 4) daire + çıkış heading'ine hizalan (giriş + 90)
        self.go_straight(STRAIGHT_T, h)   # 5) 15 sn düz
        h = self.turn_right_90()          # 6) sağa 90°
        self.go_straight(STRAIGHT_T, h)   # 7) 15 sn düz -> başlangıç alanı
        self.neutral(2.0)
        self.arm(False)
        self.get_logger().info('Görev tamamlandı.')
def main():
    rclpy.init()
    node = Mission()
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
