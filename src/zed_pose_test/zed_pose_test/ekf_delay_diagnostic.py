#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EK3_VIS_DELAY Ölçüm Aracı
===========================
ZED → MAVROS boru hattındaki gecikmeyi ölçer.

ZED odom'un header.stamp'i ile bu mesajın ROS2'de alındığı an arasındaki
farkı 30 saniye boyunca toplar ve EK3_VIS_DELAY parametresi için
önerilen değeri (milisaniye cinsinden) hesaplar.

Kullanım:
  ros2 run zed_pose_test ekf_delay_diagnostic

Çıktı:
  QGroundControl'da EK3_VIS_DELAY parametresine yazılacak ms değeri.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
import statistics


class EkfDelayDiagnostic(Node):
    def __init__(self):
        super().__init__('ekf_delay_diagnostic')

        self.declare_parameter('zed_odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('duration_sec', 30.0)

        topic = self.get_parameter('zed_odom_topic').value
        self.duration = self.get_parameter('duration_sec').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.delays_ms = []
        self.start_time = None

        self.sub = self.create_subscription(
            Odometry, topic, self.odom_cb, sensor_qos)

        self.get_logger().info(f"EK3 Delay ölçümü başladı — {self.duration:.0f} saniye veri toplanacak...")
        self.get_logger().info(f"Dinlenen topic: {topic}")

    def odom_cb(self, msg: Odometry):
        now = self.get_clock().now()

        if self.start_time is None:
            self.start_time = now

        # ZED mesajının üretildiği an (header.stamp)
        msg_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # ROS2'de alındığı an
        rcv_sec = now.nanoseconds * 1e-9

        delay_ms = (rcv_sec - msg_sec) * 1000.0

        # Anlamsız negatif veya aşırı büyük değerleri filtrele
        if 0 < delay_ms < 2000:
            self.delays_ms.append(delay_ms)

        elapsed = (now - self.start_time).nanoseconds * 1e-9
        if elapsed >= self.duration:
            self.report_and_exit()

    def report_and_exit(self):
        n = len(self.delays_ms)
        if n < 10:
            self.get_logger().error(
                f"Yeterli veri toplanamadı ({n} örnek). ZED çalışıyor mu?")
            raise SystemExit(1)

        avg = statistics.mean(self.delays_ms)
        med = statistics.median(self.delays_ms)
        p95 = sorted(self.delays_ms)[int(n * 0.95)]
        mn = min(self.delays_ms)
        mx = max(self.delays_ms)
        std = statistics.stdev(self.delays_ms) if n > 1 else 0

        self.get_logger().info("=" * 60)
        self.get_logger().info("        EK3_VIS_DELAY ÖLÇÜM SONUÇLARI")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  Toplam örnek : {n}")
        self.get_logger().info(f"  Ortalama     : {avg:.1f} ms")
        self.get_logger().info(f"  Medyan       : {med:.1f} ms")
        self.get_logger().info(f"  %95 Yüzdelik : {p95:.1f} ms")
        self.get_logger().info(f"  Min / Max    : {mn:.1f} / {mx:.1f} ms")
        self.get_logger().info(f"  Std sapma    : {std:.1f} ms")
        self.get_logger().info("-" * 60)

        # Önerilen değer: medyan + 1 std sapma (güvenli tarafta kal)
        recommended = int(med + std)
        # 10'un katına yuvarla
        recommended = max(10, ((recommended + 5) // 10) * 10)

        self.get_logger().info(f"  ➤ ÖNERİLEN EK3_VIS_DELAY = {recommended} ms")
        self.get_logger().info(f"    (QGroundControl → Parameters → EK3_VIS_DELAY)")
        self.get_logger().info("=" * 60)

        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = EkfDelayDiagnostic()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info("İptal edildi.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
