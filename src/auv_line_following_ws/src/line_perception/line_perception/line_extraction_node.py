#!/usr/bin/env python3
"""line_extraction_node.py

YOLO11-seg çıkışındaki ikili maskeyi OpenCV ile işleyerek
LineError mesajı (x_error, angle_error, is_line_lost) üretir.

Sorumluluk: SADECE görüntü işleme / geometri. Kontrol mantığı (PID, 6 eksen
komutları) burada YOKTUR; bu, line_control paketinin görevidir.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from line_interfaces.msg import LineError


class LineExtractionNode(Node):
    def __init__(self):
        super().__init__('line_extraction_node')

        self._declare_parameters()
        self._read_parameters()

        self.bridge = CvBridge()
        self._consecutive_lost_frames = 0

        self.mask_sub = self.create_subscription(
            Image, self.mask_topic, self.mask_callback, qos_profile_sensor_data
        )
        self.error_pub = self.create_publisher(LineError, self.error_topic, 10)
        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)

        self.get_logger().info(
            f"Line Extraction Node başladı. Girdi: {self.mask_topic} -> "
            f"Çıktı: {self.error_topic}"
        )

    # ------------------------------------------------------------------
    # Parametreler
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        self.declare_parameter('mask_topic', '/line_perception/segmentation_mask')
        self.declare_parameter('line_error_topic', '/line_follower/line_error')
        self.declare_parameter('debug_image_topic', '/line_follower/debug_image')
        self.declare_parameter('min_contour_area', 150)
        self.declare_parameter('lost_frame_threshold', 10)
        self.declare_parameter('publish_debug_image', True)

    def _read_parameters(self):
        gp = self.get_parameter
        self.mask_topic = gp('mask_topic').value
        self.error_topic = gp('line_error_topic').value
        self.debug_topic = gp('debug_image_topic').value
        self.min_area = float(gp('min_contour_area').value)
        self.lost_frame_threshold = int(gp('lost_frame_threshold').value)
        self.publish_debug = bool(gp('publish_debug_image').value)

    # ------------------------------------------------------------------
    # Ana geri çağırım
    # ------------------------------------------------------------------
    def mask_callback(self, msg: Image) -> None:
        try:
            mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as exc:
            self.get_logger().warn(f"Maske dönüşüm hatası: {exc}")
            return

        h, w = mask.shape[:2]
        center_x = w / 2.0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        line_error = LineError()
        line_error.header = msg.header

        largest = self._largest_valid_contour(contours)

        if largest is None:
            self._consecutive_lost_frames += 1
            line_error.x_error = 0.0
            line_error.angle_error = 0.0
            line_error.is_line_lost = (
                self._consecutive_lost_frames >= self.lost_frame_threshold
            )
            self.error_pub.publish(line_error)
            if self.publish_debug:
                self._publish_debug(mask, None, None, None)
            return

        self._consecutive_lost_frames = 0

        cx, cy, angle_deg, vx, vy = self._analyze_contour(largest)

        # Sağ pozitif, sol negatif (LineError.msg spesifikasyonuna göre)
        line_error.x_error = float(cx - center_x)
        line_error.angle_error = float(angle_deg)
        line_error.is_line_lost = False

        self.error_pub.publish(line_error)

        if self.publish_debug:
            self._publish_debug(mask, largest, (cx, cy), (vx, vy))

    def _largest_valid_contour(self, contours):
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            return None
        return largest

    @staticmethod
    def _analyze_contour(contour):
        """Konturun ağırlık merkezini ve yönelim açısını hesaplar.

        Açı, görüntünün dikey (yukarı) eksenine göre, LineError.msg'de
        tanımlı kurala uygun şekilde hesaplanır: 0 -> çizgi dikey (araç
        hizalı), pozitif -> çizginin üstü sağa yatık.
        """
        moments = cv2.moments(contour)
        cx = moments['m10'] / moments['m00']
        cy = moments['m01'] / moments['m00']

        # En küçük kareler ile çizgi doğrultusunu bul (gürültüye karşı
        # minAreaRect'ten daha kararlı)
        vx, vy, _, _ = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01).flatten()

        # fitLine yönü belirsizdir (180 derece ters olabilir); yönü her
        # zaman "yukarı" (görüntü -y) bakacak şekilde normalleştir.
        if vy > 0:
            vx, vy = -vx, -vy

        angle_rad = np.arctan2(vx, -vy)  # dikeyden sapma
        angle_deg = np.degrees(angle_rad)

        return cx, cy, angle_deg, vx, vy

    def _publish_debug(self, mask, contour, centroid, direction):
        debug_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        h, w = mask.shape[:2]
        center_x = w // 2

        cv2.line(debug_img, (center_x, 0), (center_x, h), (255, 0, 0), 1)

        if contour is not None:
            cv2.drawContours(debug_img, [contour], -1, (0, 255, 0), 2)
        if centroid is not None:
            cx, cy = int(centroid[0]), int(centroid[1])
            cv2.circle(debug_img, (cx, cy), 6, (0, 0, 255), -1)
            if direction is not None:
                vx, vy = direction
                length = 80
                pt2 = (int(cx + vx * length), int(cy + vy * length))
                cv2.arrowedLine(debug_img, (cx, cy), pt2, (0, 165, 255), 2)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineExtractionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
