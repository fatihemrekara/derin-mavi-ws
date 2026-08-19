#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn, VfrHud
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import math
import os

class EkfCircleEvaluation(Node):
    def __init__(self):
        super().__init__('ekf_circle_evaluation')
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
            
        self.hdg_sub = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.hdg_cb, sensor_qos)
        self.vfr_sub = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.vfr_cb, sensor_qos)
        self.rel_alt_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.rel_alt_cb, sensor_qos)
        
        self.ekf_pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.ekf_cb, sensor_qos)
        self.zed_pose_sub = self.create_subscription(PoseStamped, '/zed/zed_node/pose', self.zed_cb, sensor_qos)
        self.imu_sub = self.create_subscription(Imu, '/mavros/imu/data', self.imu_cb, sensor_qos)
        
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        
        self.state = 'STARTING'
        self.compass_hdg = 0.0
        self.altitude = 0.0
        self.rel_alt = 0.0
        
        self.ekf_x, self.ekf_y, self.ekf_z = 0.0, 0.0, 0.0
        self.zed_x, self.zed_y, self.zed_z = 0.0, 0.0, 0.0
        self.imu_acc_x, self.imu_acc_y, self.imu_acc_z = 0.0, 0.0, 0.0
        
        self.state_start_time = self.get_clock().now()
        
        from datetime import datetime
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'ekf_circle_evaluation_log_{stamp}.csv')
        self.log_file = open(log_path, 'w')
        self.log_file.write("Time(s),State,RelAlt(m),Compass_Hdg(deg),EKF_X,EKF_Y,EKF_Z,ZED_X,ZED_Y,ZED_Z,IMU_AccX,IMU_AccY,IMU_AccZ\n")
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("EKF Circle Evaluation scripti baslatildi.")
        self.get_logger().info(f"Loglar {log_path} dosyasina kaydediliyor...")
        
    def hdg_cb(self, msg): self.compass_hdg = msg.data
    def vfr_cb(self, msg): self.altitude = msg.altitude
    def rel_alt_cb(self, msg): self.rel_alt = msg.data
    
    def ekf_cb(self, msg):
        self.ekf_x = msg.pose.position.x
        self.ekf_y = msg.pose.position.y
        self.ekf_z = msg.pose.position.z
        
    def zed_cb(self, msg):
        self.zed_x = msg.pose.position.x
        self.zed_y = msg.pose.position.y
        self.zed_z = msg.pose.position.z
        
    def imu_cb(self, msg):
        self.imu_acc_x = msg.linear_acceleration.x
        self.imu_acc_y = msg.linear_acceleration.y
        self.imu_acc_z = msg.linear_acceleration.z

    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().info(f"--- STATE: {new_state} ---")

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        
        if int(elapsed * 10) % 10 == 0:
            self.get_logger().info(f"[{self.state}] EKF(X:{self.ekf_x:.2f}) ZED(X:{self.zed_x:.2f})")
            
        t_sec = now.nanoseconds * 1e-9
        self.log_file.write(f"{t_sec:.2f},{self.state},{self.rel_alt:.2f},{self.compass_hdg:.2f},{self.ekf_x:.2f},{self.ekf_y:.2f},{self.ekf_z:.2f},{self.zed_x:.2f},{self.zed_y:.2f},{self.zed_z:.2f},{self.imu_acc_x:.2f},{self.imu_acc_y:.2f},{self.imu_acc_z:.2f}\n")
        
        if self.state == 'STARTING':
            if elapsed > 2.0:
                self.change_state('ARMING')
        elif self.state == 'ARMING':
            if elapsed < 0.2:
                if self.mode_client.wait_for_service(timeout_sec=0.5):
                    req = SetMode.Request()
                    req.custom_mode = 'ALT_HOLD'
                    self.mode_client.call_async(req)
                if self.arm_client.wait_for_service(timeout_sec=0.5):
                    req = CommandBool.Request()
                    req.value = True
                    self.arm_client.call_async(req)
            
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[0] = 1500
            rc.channels[1] = 1500
            rc.channels[2] = 1500
            rc.channels[3] = 1500
            rc.channels[4] = 1500
            self.rc_pub.publish(rc)

            if elapsed > 2.0:
                self.change_state('DIVING')

        elif self.state == 'DIVING':
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[0] = 1500
            rc.channels[1] = 1500
            rc.channels[2] = 1400  # Dalış
            rc.channels[3] = 1500
            rc.channels[4] = 1500
            self.rc_pub.publish(rc)
            
            if self.rel_alt < -1.0 or self.altitude < -1.0 or elapsed > 15.0:
                if self.mode_client.wait_for_service(timeout_sec=0.1):
                    req = SetMode.Request()
                    req.custom_mode = 'ALT_HOLD'
                    self.mode_client.call_async(req)
                self.change_state('MOVING_FORWARD')

        elif self.state == 'MOVING_FORWARD':
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[0] = 1500  # Pitch
            rc.channels[1] = 1500  # Roll
            rc.channels[2] = 1500  # Derinlik koruma (Throttle)
            rc.channels[3] = 1500  # Yaw
            rc.channels[4] = 1650  # Ileri (1800'den 1650'ye dusuruldu, pitch-up kalkmasini engellemek icin)
            self.rc_pub.publish(rc)
            
            if elapsed > 10.0:
                self.change_state('CIRCLING')
                
        elif self.state == 'CIRCLING':
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[0] = 1500
            rc.channels[1] = 1500
            rc.channels[2] = 1500  # ALT_HOLD devrede
            rc.channels[3] = 1580  # Hafif sağa dönüş ile çember
            rc.channels[4] = 1620  # Yavaşça ileri gidiş
            self.rc_pub.publish(rc)
            
            if elapsed > 30.0: # Daireyi tamamlaması için yeterli süre
                self.change_state('STOPPING_MOTORS')

        elif self.state == 'STOPPING_MOTORS':
            if elapsed < 0.2:
                if self.arm_client.wait_for_service(timeout_sec=0.5):
                    req = CommandBool.Request()
                    req.value = False
                    self.arm_client.call_async(req)
                
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            self.rc_pub.publish(rc)
            
            if elapsed > 1.0:
                self.change_state('SURFACING_AND_LOGGING')
            
        elif self.state == 'SURFACING_AND_LOGGING':
            pass

def main(args=None):
    rclpy.init(args=args)
    node = EkfCircleEvaluation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Test durduruluyor, RC kanallari temizleniyor...")
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
