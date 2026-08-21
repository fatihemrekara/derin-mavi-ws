#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ImuAlignmentNode(Node):
    def __init__(self):
        super().__init__('imu_alignment_node')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.sub = self.create_subscription(
            Imu, 
            '/zed/zed_node/imu/data_raw', 
            self.imu_cb, 
            qos
        )
        
        self.pub = self.create_publisher(Imu, '/zed/aligned_imu', 10)
        self.get_logger().info("IMU Alignment Node Started. Mapping ZED Downward frame to ENU...")

    def imu_cb(self, msg: Imu):
        aligned = Imu()
        aligned.header = msg.header
        aligned.header.frame_id = 'base_link'  # Artık robotun gövde merkezine hizalandı kabul ediliyor
        
        # LOG analizinde bulduğumuz eksen değişimleri
        # ZED Z (Optik İleri) -> MAVROS X (Gövde İleri)
        aligned.linear_acceleration.x = msg.linear_acceleration.z
        aligned.angular_velocity.x = msg.angular_velocity.z
        
        # ZED Y (Ön Sol) -> MAVROS Y (Gövde Sol)
        aligned.linear_acceleration.y = msg.linear_acceleration.y
        aligned.angular_velocity.y = msg.angular_velocity.y
        
        # ZED X (Optik Aşağı) -> MAVROS Z (Gövde Yukarı, yani ters çeviriyoruz)
        aligned.linear_acceleration.z = -msg.linear_acceleration.x
        aligned.angular_velocity.z = -msg.angular_velocity.x
        
        # Oryantasyon çeyreği bilinmediğinden geçersiz kılıyoruz (EKF kendisi ivme/jirodan çözer)
        aligned.orientation_covariance[0] = -1.0
        
        self.pub.publish(aligned)

def main(args=None):
    rclpy.init(args=args)
    node = ImuAlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
