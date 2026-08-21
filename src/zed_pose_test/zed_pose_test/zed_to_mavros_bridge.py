#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ZED 2 → MAVROS (ArduSub EKF3) Akıllı Köprü — Outlier Rejection & Covariance Inflation
========================================================================================

ZED Wrapper'dan gelen görsel odometriyi (nav_msgs/Odometry) MAVROS'un
görsel pozisyon topic'ine (geometry_msgs/PoseWithCovarianceStamped) aktarır.

EKF3'ü koruyan 3 katmanlı savunma:
  1. Jump Detection: Fiziksel olarak imkansız konum sıçramalarını tespit edip drop eder
  2. Confidence Check: ZED'in kendi güven skorunu kontrol eder
  3. Covariance Inflation: Güven düşükse covariance matrisini şişirerek EKF3'e
     "bu veriye az güven" sinyali gönderir

Not: Offset mesafe (X,Y,Z) ve sensör yönelimi (VISO_ORIENT=Down) ayarları
QGroundControl üzerinden EKF3'e yaptırılmaktadır.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32

import math
import time


class ZedToMavrosBridge(Node):
    def __init__(self):
        super().__init__('zed_to_mavros_bridge')

        # ── Parametreler ──
        self.declare_parameter('zed_odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('mavros_vision_topic', '/mavros/vision_pose/pose_cov')
        self.declare_parameter('zed_confidence_topic', '/zed/zed_node/confidence')

        # Outlier rejection parametreleri
        self.declare_parameter('max_speed_ms', 1.5)        # m/s — bunun üstü fiziksel olarak imkansız
        self.declare_parameter('min_confidence', 30)        # ZED confidence: 0-100 arası. 30 altı = kör
        self.declare_parameter('low_confidence_threshold', 60)  # Bunun altında covariance şişir
        self.declare_parameter('covariance_inflation_factor', 50.0)  # Düşük güvende cov × bu kadar
        self.declare_parameter('stats_interval_sec', 10.0)  # İstatistik log aralığı

        # Parametre değerlerini oku
        self.max_speed = self.get_parameter('max_speed_ms').value
        self.min_conf = self.get_parameter('min_confidence').value
        self.low_conf = self.get_parameter('low_confidence_threshold').value
        self.cov_inflation = self.get_parameter('covariance_inflation_factor').value
        self.stats_interval = self.get_parameter('stats_interval_sec').value

        # ── State (önceki ölçüm takibi) ──
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.prev_time = None
        self.zed_confidence = 100  # Başlangıçta maksimum güven varsay

        # ── İstatistikler ──
        self.total_frames = 0
        self.dropped_jump = 0
        self.dropped_confidence = 0
        self.inflated_frames = 0
        self.passed_frames = 0
        self.stats_start_time = time.time()

        # ── QoS profilleri ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Topic'ler ──
        zed_topic = self.get_parameter('zed_odom_topic').value
        mav_topic = self.get_parameter('mavros_vision_topic').value
        conf_topic = self.get_parameter('zed_confidence_topic').value

        self.pub = self.create_publisher(PoseWithCovarianceStamped, mav_topic, pub_qos)

        self.sub = self.create_subscription(
            Odometry, zed_topic, self.odom_callback, sensor_qos)

        self.conf_sub = self.create_subscription(
            Float32, conf_topic, self.confidence_callback, sensor_qos)

        # İstatistik timer
        self.stats_timer = self.create_timer(self.stats_interval, self.log_stats)

        self.get_logger().info("=" * 60)
        self.get_logger().info("ZED → MAVROS Akıllı Köprü (Outlier Rejection) Başlatıldı")
        self.get_logger().info(f"  Dinlenen  : {zed_topic}")
        self.get_logger().info(f"  Yayınlanan: {mav_topic}")
        self.get_logger().info(f"  Confidence: {conf_topic}")
        self.get_logger().info(f"  Max hız eşiği : {self.max_speed} m/s")
        self.get_logger().info(f"  Min confidence: {self.min_conf}")
        self.get_logger().info(f"  Low conf thres: {self.low_conf}")
        self.get_logger().info(f"  Cov inflation : ×{self.cov_inflation}")
        self.get_logger().info("=" * 60)

    # ── Callback: ZED Confidence ──
    def confidence_callback(self, msg: Float32):
        self.zed_confidence = msg.data

    # ── Callback: ZED Odometri ──
    def odom_callback(self, msg: Odometry):
        self.total_frames += 1

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # ━━━ KATMAN 1: Jump Detection (Sıçrama Tespiti) ━━━
        if self.prev_x is not None and self.prev_time is not None:
            dt = now_sec - self.prev_time
            if dt > 1e-6:  # Sıfıra bölmeden kaçın
                dx = x - self.prev_x
                dy = y - self.prev_y
                dz = z - self.prev_z
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                speed = dist / dt

                if speed > self.max_speed:
                    self.dropped_jump += 1
                    self.get_logger().warn(
                        f'JUMP DROP: {speed:.2f} m/s > {self.max_speed} m/s eşiği '
                        f'(dt={dt:.3f}s, dist={dist:.3f}m)',
                        throttle_duration_sec=1.0)
                    # Önceki pozisyonu GÜNCELLEME — sıçrama noktasını referans alma
                    return

        # Önceki pozisyonu güncelle (sadece geçerli verilerle)
        self.prev_x = x
        self.prev_y = y
        self.prev_z = z
        self.prev_time = now_sec

        # ━━━ KATMAN 2: Confidence Check (Güven Kontrolü) ━━━
        if self.zed_confidence < self.min_conf:
            self.dropped_confidence += 1
            self.get_logger().warn(
                f'CONFIDENCE DROP: ZED confidence={self.zed_confidence:.0f} < {self.min_conf} eşiği',
                throttle_duration_sec=2.0)
            return

        # ━━━ Mesajı oluştur ━━━
        out_msg = PoseWithCovarianceStamped()
        out_msg.header.stamp = msg.header.stamp
        out_msg.header.frame_id = 'odom'

        # ━━━ EKSEN DÖNÜŞÜMÜ (AXIS REMAPPING) ━━━
        # Kamera 90 derece aşağı bakıyor. 
        # ZED'in kendi X'i aşağı, Z'si ileri gösteriyor. 
        # MAVROS ENU (X: İleri, Y: Sol, Z: Yukarı) bekliyor.
        out_msg.pose.pose.position.x = msg.pose.pose.position.z   # ZED Z -> MAVROS X (İleri)
        out_msg.pose.pose.position.y = msg.pose.pose.position.y   # ZED Y -> MAVROS Y (Sol)
        out_msg.pose.pose.position.z = -msg.pose.pose.position.x  # ZED X (Aşağı) -> MAVROS Z (Yukarı = -Aşağı)

        # Orientation sabitlenmesi (sadece XYZ pozisyon kullanılıyor)
        # Yaw/Pitch/Roll tamamen Orange Cube IMU'dan çözülüyor
        out_msg.pose.pose.orientation.x = 0.0
        out_msg.pose.pose.orientation.y = 0.0
        out_msg.pose.pose.orientation.z = 0.0
        out_msg.pose.pose.orientation.w = 1.0

        # ━━━ KOVARYANS MATRİSİ DÖNÜŞÜMÜ ━━━
        # Eksenler değiştiği için varyansların da matriste yer değiştirmesi şarttır
        old_cov = list(msg.pose.covariance)
        new_cov = [0.0] * 36
        
        # Mappings: MAVROS[0(X)] = ZED[2(Z)], MAVROS[1(Y)] = ZED[1(Y)], MAVROS[2(Z)] = ZED[0(X)]
        # Roll ve Yaw eksenleri de aynı mantıkla yer değiştirir
        mapping = [2, 1, 0, 5, 4, 3] 
        signs = [1.0, 1.0, -1.0, 1.0, 1.0, -1.0]
        
        for i in range(6):
            for j in range(6):
                orig_i = mapping[i]
                orig_j = mapping[j]
                new_cov[i*6 + j] = old_cov[orig_i*6 + orig_j] * signs[i] * signs[j]

        # ━━━ KATMAN 3: Covariance Inflation (Güven düşükse şişir) ━━━
        if self.zed_confidence < self.low_conf:
            # Düşük güvende covariance'ın köşegen elemanlarını şişir
            # EKF3 bunu görür ve "bu ölçüme az güveneyim, IMU'ya yaslanayım" der
            factor = self.cov_inflation
            # 6x6 covariance matrisinin köşegen indeksleri: 0, 7, 14, 21, 28, 35
            for diag_idx in [0, 7, 14, 21, 28, 35]:
                new_cov[diag_idx] = max(new_cov[diag_idx] * factor, factor * 0.01)
            self.inflated_frames += 1
        else:
            self.passed_frames += 1

        out_msg.pose.covariance = new_cov

        self.pub.publish(out_msg)

    # ── İstatistik loglama ──
    def log_stats(self):
        elapsed = time.time() - self.stats_start_time
        if self.total_frames == 0:
            return

        drop_pct = ((self.dropped_jump + self.dropped_confidence) / self.total_frames) * 100
        inf_pct = (self.inflated_frames / self.total_frames) * 100

        self.get_logger().info(
            f'[STATS {elapsed:.0f}s] '
            f'Toplam:{self.total_frames} | '
            f'Geçen:{self.passed_frames} | '
            f'Şişirilen:{self.inflated_frames}({inf_pct:.1f}%) | '
            f'Jump drop:{self.dropped_jump} | '
            f'Conf drop:{self.dropped_confidence} | '
            f'Toplam drop:{drop_pct:.1f}% | '
            f'ZED conf:{self.zed_confidence:.0f}')


def main(args=None):
    rclpy.init(args=args)
    node = ZedToMavrosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Bridge durduruluyor...")
        node.log_stats()  # Son istatistikleri bas
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
