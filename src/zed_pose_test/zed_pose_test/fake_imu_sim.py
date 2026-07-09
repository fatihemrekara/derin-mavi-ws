#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sahte IMU (Ataletsel Ölçüm Birimi) Simülatörü
--------------------------------------------
/cmd_vel veya gerçek hareket bilgisini alıp üzerine gürültü ve bias ekleyerek
/imu/data (sensor_msgs/Imu) olarak yayınlar.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class FakeImuSim(Node):
    def __init__(self):
        super().__init__('fake_imu_sim')
        
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('rate_hz', 50.0)  # IMU genellikle hızlı veri verir (50-200 Hz)
        
        # IMU Gürültü Parametreleri
        self.declare_parameter('gyro_noise_density', 0.05) # rad/s
        self.declare_parameter('gyro_bias_drift', 0.001)   # rad/s^2 (zamanla biriken bias)
        
        gp = lambda n: self.get_parameter(n).value
        
        self.rate_hz = float(gp('rate_hz'))
        self.dt = 1.0 / self.rate_hz
        
        self.gyro_noise_density = float(gp('gyro_noise_density'))
        self.gyro_bias_drift = float(gp('gyro_bias_drift'))
        
        self.current_w = 0.0
        self.current_bias = 0.0
        
        self.imu_pub = self.create_publisher(Imu, str(gp('imu_topic')), 10)
        self.cmd_sub = self.create_subscription(Twist, str(gp('cmd_topic')), self.on_cmd, 10)
        
        self.timer = self.create_timer(self.dt, self.publish_imu)
        
        self.get_logger().info(f"Sahte IMU {self.rate_hz} Hz ile çalışıyor. Gürültü: {self.gyro_noise_density}, Bias Kayması: {self.gyro_bias_drift}")

    def on_cmd(self, msg: Twist):
        # Kinematik modelde arabanın gerçek dönüş hızı cmd_vel'deki angular.z'dir
        # (Fizik motorumuz şu an gecikmesiz direkt bu hızı kullanıyor)
        self.current_w = msg.angular.z

    def publish_imu(self):
        # 1. Bias (Sabit kayma) zamanla rastgele yürüyüş yapar
        self.current_bias += np.random.normal(0, self.gyro_bias_drift) * self.dt
        
        # 2. Anlık beyaz gürültü
        noise = np.random.normal(0, self.gyro_noise_density)
        
        # 3. Ölçülen açısal hız
        measured_w = self.current_w + self.current_bias + noise
        
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        
        # Sadece Z eksenindeki (yaw) dönüşü dolduruyoruz (2D model)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = measured_w
        
        # İvmeölçer (Linear Acceleration) - Şimdilik sadece yerçekimi veriyoruz
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81
        
        # Quaternion (Orientation) - Gerçek IMU'lar manyetometre ile yön bulur,
        # ancak biz burada sadece raw (işlenmemiş) gyro ve ivme basıyoruz.
        # Kalman filtresi bu raw değerleri alıp yönelim hesaplayacak.
        
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeImuSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
