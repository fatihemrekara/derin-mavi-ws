#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State as MavrosState

class TestState:
    INIT = 0
    ARMING = 1
    MOVING_FORWARD = 2
    TURN_180 = 3
    RETURN = 4
    DONE = 5

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a

class PoolTestNode(Node):
    def __init__(self):
        super().__init__('pool_test_node')
        
        # Parameters
        self.declare_parameter('distance', 5.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('obstacle_threshold', 1.5)
        self.declare_parameter('auto_arm', True)
        
        self.target_distance = self.get_parameter('distance').value
        self.fwd_speed = self.get_parameter('speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.obs_thresh = self.get_parameter('obstacle_threshold').value
        self.auto_arm = self.get_parameter('auto_arm').value
        
        # State variables
        self.state = TestState.INIT
        self.distance_traveled = 0.0
        self.last_pose = None
        self.front_distance = 10.0 # safe initial value
        self.initial_turn_yaw = 0.0
        
        # MAVROS state
        self.mavros_state = MavrosState()
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/zed/zed_node/odom', self.odom_cb, qos_profile)
        self.depth_sub = self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered', self.depth_cb, qos_profile)
        self.state_sub = self.create_subscription(
            MavrosState, '/mavros/state', self.mavros_state_cb, 10)
            
        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
            
        # Service Clients
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        
        # Timer loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"PoolTestNode started. Target distance: {self.target_distance}m")

    def mavros_state_cb(self, msg):
        self.mavros_state = msg

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        
        if self.last_pose is not None:
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            dist_inc = math.hypot(dx, dy)
            
            if self.state in [TestState.MOVING_FORWARD, TestState.RETURN]:
                self.distance_traveled += dist_inc
                
        self.last_pose = (x, y, yaw)

    def depth_cb(self, msg):
        try:
            # depth_registered is 32FC1
            if msg.encoding != '32FC1':
                return
                
            depth_array = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            
            # 50x50 center crop
            h, w = depth_array.shape
            cy, cx = h//2, w//2
            roi = depth_array[cy-25:cy+25, cx-25:cx+25]
            
            valid_depths = roi[np.isfinite(roi)]
            if len(valid_depths) > 0:
                self.front_distance = float(np.nanmean(valid_depths))
            else:
                self.front_distance = 10.0
                
        except Exception as e:
            self.get_logger().error(f"Depth processing error: {e}")

    def set_mode(self, mode):
        if self.mode_cli.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = mode
            self.mode_cli.call_async(req)

    def arm_vehicle(self, arm):
        if self.arm_cli.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = arm
            self.arm_cli.call_async(req)

    def stop_motors(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        # State Machine Logic
        if self.state == TestState.INIT:
            if not self.auto_arm:
                self.get_logger().info("Auto-arm disabled. Waiting for manual arm and GUIDED mode...")
                if self.mavros_state.armed and self.mavros_state.mode == "GUIDED":
                    self.get_logger().info("Vehicle is ARMED and in GUIDED mode. Starting mission!")
                    self.state = TestState.MOVING_FORWARD
                    self.distance_traveled = 0.0
            else:
                self.get_logger().info("Auto-arming and setting GUIDED mode...")
                self.set_mode("GUIDED")
                self.arm_vehicle(True)
                self.state = TestState.ARMING
                
        elif self.state == TestState.ARMING:
            if self.mavros_state.armed and self.mavros_state.mode == "GUIDED":
                self.get_logger().info("Successfully armed and in GUIDED mode. Starting mission!")
                self.state = TestState.MOVING_FORWARD
                self.distance_traveled = 0.0
            else:
                # Keep trying
                self.set_mode("GUIDED")
                self.arm_vehicle(True)
                
        elif self.state == TestState.MOVING_FORWARD:
            if self.front_distance < self.obs_thresh:
                self.get_logger().warn(f"OBSTACLE DETECTED at {self.front_distance:.2f}m! Stopping forward motion.")
                self.target_distance = self.distance_traveled # Make it return from here
                self.state = TestState.TURN_180
                self.initial_turn_yaw = self.last_pose[2] if self.last_pose else 0.0
                self.stop_motors()
                return
                
            if self.distance_traveled >= self.target_distance:
                self.get_logger().info(f"Reached target distance: {self.distance_traveled:.2f}m. Turning 180 degrees.")
                self.state = TestState.TURN_180
                self.initial_turn_yaw = self.last_pose[2] if self.last_pose else 0.0
                self.stop_motors()
                return
                
            # Move forward
            cmd = Twist()
            cmd.linear.x = self.fwd_speed
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"Forward... Dist: {self.distance_traveled:.2f}/{self.target_distance:.2f}m | Front: {self.front_distance:.2f}m", throttle_duration_sec=0.5)

        elif self.state == TestState.TURN_180:
            if self.last_pose is None:
                return
                
            current_yaw = self.last_pose[2]
            target_yaw = normalize_angle(self.initial_turn_yaw + math.pi)
            yaw_error = normalize_angle(target_yaw - current_yaw)
            
            if abs(yaw_error) < 0.1: # ~5.7 degrees tolerance
                self.get_logger().info("180 degree turn completed. Returning...")
                self.state = TestState.RETURN
                self.distance_traveled = 0.0
                self.stop_motors()
                return
                
            # Rotate
            cmd = Twist()
            # Turn direction based on error
            cmd.angular.z = self.turn_speed if yaw_error > 0 else -self.turn_speed
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"Turning... Error: {math.degrees(yaw_error):.1f} deg", throttle_duration_sec=0.5)

        elif self.state == TestState.RETURN:
            if self.front_distance < self.obs_thresh:
                self.get_logger().warn(f"OBSTACLE DETECTED on return at {self.front_distance:.2f}m! Stopping.")
                self.state = TestState.DONE
                self.stop_motors()
                return
                
            if self.distance_traveled >= self.target_distance:
                self.get_logger().info("Returned to start point. Mission Complete.")
                self.state = TestState.DONE
                self.stop_motors()
                # Optional: disarm vehicle
                # self.arm_vehicle(False)
                return
                
            # Move forward
            cmd = Twist()
            cmd.linear.x = self.fwd_speed
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"Returning... Dist: {self.distance_traveled:.2f}/{self.target_distance:.2f}m | Front: {self.front_distance:.2f}m", throttle_duration_sec=0.5)

        elif self.state == TestState.DONE:
            self.stop_motors()

    def shutdown_sequence(self):
        self.get_logger().info("Ctrl+C algılandı! Motorlar durduruluyor ve MANUEL moda geçiliyor...")
        self.stop_motors()
        
        # Senkron (bekleyerek) mod değiştirme ki node kapanmadan komut gitsin
        if self.mode_cli.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = "MANUAL"
            future = self.mode_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            self.get_logger().info("Araç MANUAL moda alındı. Kontrol kumandada.")

def main(args=None):
    rclpy.init(args=args)
    node = PoolTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_sequence()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
