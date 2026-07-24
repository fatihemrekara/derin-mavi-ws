#!/usr/bin/env python3
"""yolo_segmentation_node.py

YOLO11-seg modelini kullanarak kameradan gelen görüntüde çizgiyi (line)
segmente eder ve ikili (binary) maske olarak yayınlar.

Bu düğüm, algılama (perception) katmanının ilk aşamasıdır:

    Kamera -> [YOLO11-seg] -> binary mask -> [OpenCV: line_extraction_node]
              -> LineError

Sorumluluğu SADECE segmentasyon yapmaktır; piksel hatası hesaplama,
açı hesaplama gibi geometrik işlemler bilinçli olarak line_extraction_node'a
bırakılmıştır (tek sorumluluk ilkesi / modülerlik).
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "ultralytics paketi bulunamadı. Kurmak için: "
        "pip install ultralytics --break-system-packages"
    ) from exc


class YoloSegmentationNode(Node):
    """Kameradan gelen görüntüyü YOLO11-seg ile işleyip çizgi maskesi üretir."""

    def __init__(self):
        super().__init__('yolo_segmentation_node')

        self._declare_parameters()
        self._read_parameters()

        self.get_logger().info(f"YOLO11-seg modeli yükleniyor: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"Model yüklendi. Cihaz: {self.device}")

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, qos_profile_sensor_data
        )
        self.mask_pub = self.create_publisher(Image, self.mask_topic, 10)
        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)

        self.get_logger().info(
            f"YOLO Segmentation Node başladı. Girdi: {self.camera_topic} -> "
            f"Çıktı: {self.mask_topic}"
        )

    # ------------------------------------------------------------------
    # Parametreler
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        self.declare_parameter('model_path', 'models/line_seg_yolo11.pt')
        self.declare_parameter('camera_topic', '/auv/camera/image_raw')
        self.declare_parameter('mask_topic', '/line_perception/segmentation_mask')
        self.declare_parameter('debug_image_topic', '/line_perception/annotated_image')
        self.declare_parameter('target_class_id', 0)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cpu')  # 'cpu' | 'cuda:0'
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('input_image_size', 640)

    def _read_parameters(self):
        gp = self.get_parameter
        self.model_path = gp('model_path').value
        self.camera_topic = gp('camera_topic').value
        self.mask_topic = gp('mask_topic').value
        self.debug_topic = gp('debug_image_topic').value
        self.target_class_id = int(gp('target_class_id').value)
        self.conf_thresh = float(gp('confidence_threshold').value)
        self.device = gp('device').value
        self.publish_debug = bool(gp('publish_debug_image').value)
        self.imgsz = int(gp('input_image_size').value)

    # ------------------------------------------------------------------
    # Ana geri çağırım
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # cv_bridge dönüşüm hatası
            self.get_logger().warn(f"Görüntü dönüşüm hatası: {exc}")
            return

        h, w = frame.shape[:2]

        results = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self.conf_thresh,
            device=self.device,
            verbose=False,
            retina_masks=True,  # maskeleri orijinal çözünürlükte al
        )

        binary_mask = np.zeros((h, w), dtype=np.uint8)
        result = results[0]

        if result.masks is not None and result.boxes is not None:
            classes = result.boxes.cls.cpu().numpy().astype(int)
            mask_data = result.masks.data.cpu().numpy()  # (N, H, W) 0/1

            for cls_id, single_mask in zip(classes, mask_data):
                if cls_id != self.target_class_id:
                    continue
                if single_mask.shape[:2] != (h, w):
                    single_mask = cv2.resize(
                        single_mask.astype(np.float32), (w, h),
                        interpolation=cv2.INTER_NEAREST,
                    )
                binary_mask = np.bitwise_or(
                    binary_mask, (single_mask > 0.5).astype(np.uint8) * 255
                )

        mask_msg = self.bridge.cv2_to_imgmsg(binary_mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        if self.publish_debug:
            annotated = result.plot()
            debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
