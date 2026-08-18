#!/usr/bin/env python3
"""
MINI ROV KAMERA YAYINCISI  (Raspberry Pi 5 + OBSBOT Meet SE)

TEMEL FIKIR — "kasmama" burada saklidir:
OBSBOT Meet SE bir UVC kameradir ve goruntuyu DONANIMDA zaten MJPEG olarak
uretir. Klasik yaklasim (cv2.VideoCapture -> BGR -> cv2.imencode('.jpg'))
bu JPEG'i once cozer, sonra tekrar sikistirir. Pi 5'te bu bosuna %40-70 CPU
demektir.

Burada CAP_PROP_CONVERT_RGB = 0 ile OpenCV'ye "cozme, ham ver" diyoruz.
Gelen buffer dogrudan JPEG baytlaridir ve oldugu gibi CompressedImage
icine konur. Pi tarafinda decode/encode YOK, sadece kopyalama var.

Eger surucu bu modu desteklemezse kod otomatik olarak normal decode+encode
yoluna duser (uyari loglar) — yani her halukarda calisir.

DIKKAT — COZUNURLUK SECIMI:
OBSBOT Meet SE, MJPEG'i YALNIZCA 1920x1080 ve 1280x720'de sunar. 640x480
istersen surucu YUYV'ye duser, passthrough kapanir ve Pi bosuna encode eder.
Bu yuzden varsayilan 1280x720'dir. Kendi kameranda once sunu kontrol et:
    v4l2-ctl -d /dev/video0 --list-formats-ext

KULLANIM:
    python3 mini_cam_publisher.py
    python3 mini_cam_publisher.py --ros-args -p device:=/dev/video0 -p fps:=20

Topic: /mini_rov/camera/image_raw/compressed   (sensor_msgs/CompressedImage)
QoS  : BEST_EFFORT, depth=1  (istasyondaki GUI ile ayni)
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage


class MiniCamPublisher(Node):

    def __init__(self):
        super().__init__('mini_cam_publisher')

        # ---- Parametreler (launch/CLI'dan degistirilebilir) ----
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 1280)   # OBSBOT Meet SE: MJPG sadece 1080p/720p verir
        self.declare_parameter('height', 720)  # 640x480 sadece YUYV'de var -> passthrough'u bozar
        self.declare_parameter('fps', 15)      # surucunun destekledigi tam degerlerden biri
        self.declare_parameter('topic', '/mini_rov/camera/image_raw/compressed')
        self.declare_parameter('jpeg_quality', 70)   # sadece fallback yolunda kullanilir
        self.declare_parameter('frame_id', 'mini_cam')

        p = self.get_parameter
        self.device = p('device').value
        self.width = int(p('width').value)
        self.height = int(p('height').value)
        self.fps = int(p('fps').value)
        self.jpeg_quality = int(p('jpeg_quality').value)
        self.frame_id = p('frame_id').value

        cam_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.pub = self.create_publisher(CompressedImage, p('topic').value, cam_qos)

        self.cap = None
        self.passthrough = False
        self._open_camera()

        # Yakalama dongusu timer ile: rclpy.spin ile birlikte duzgun calisir.
        self.timer = self.create_timer(1.0 / max(self.fps, 1), self.grab_and_publish)

        # Basit FPS/istatistik logu (10 saniyede bir)
        self._n = 0
        self._bytes = 0
        self._t0 = time.monotonic()
        self.stat_timer = self.create_timer(10.0, self._log_stats)

    # ------------------------------------------------------------------
    def _open_camera(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f"Kamera acilamadi: {self.device}")
            raise RuntimeError(f"Kamera acilamadi: {self.device}")

        # 1) Kameradan MJPEG iste (YUYV istersen USB bandini doldurur ve FPS duser)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        # 2) Surucu tarafinda kare birikmesin (gecikme = eski kare demektir)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # 3) ASIL NUMARA: cozme, ham MJPEG baytlarini ver
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

        self.cap = cap

        # Ilk kareye bakarak passthrough gercekten calisiyor mu anlayalim.
        ok, frame = cap.read()
        if ok and frame is not None and frame.ndim <= 2:
            self.passthrough = True
            self.get_logger().info(
                "MJPEG passthrough AKTIF - Pi'de decode/encode yapilmiyor.")
        else:
            self.passthrough = False
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            self.get_logger().warn(
                "MJPEG passthrough desteklenmedi; decode+encode moduna dusuldu "
                "(CPU kullanimi daha yuksek olacak).")

        real_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        real_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(
            f"{self.device} acildi: {int(real_w)}x{int(real_h)} @ hedef {self.fps} FPS")

    # ------------------------------------------------------------------
    def grab_and_publish(self):
        if self.cap is None:
            return
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Kare alinamadi.", throttle_duration_sec=2.0)
            return

        if self.passthrough:
            # frame zaten JPEG baytlari (1 x N uint8)
            payload = np.asarray(frame).tobytes()
        else:
            enc_ok, buf = cv2.imencode(
                '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not enc_ok:
                return
            payload = buf.tobytes()

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = payload
        self.pub.publish(msg)

        self._n += 1
        self._bytes += len(payload)

    def _log_stats(self):
        dt = time.monotonic() - self._t0
        if dt <= 0 or self._n == 0:
            return
        fps = self._n / dt
        mbps = (self._bytes * 8) / dt / 1e6
        self.get_logger().info(
            f"Yayin: {fps:.1f} FPS, {mbps:.1f} Mbit/s, "
            f"ort. kare {self._bytes / self._n / 1024:.0f} KB")
        self._n = 0
        self._bytes = 0
        self._t0 = time.monotonic()

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MiniCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
