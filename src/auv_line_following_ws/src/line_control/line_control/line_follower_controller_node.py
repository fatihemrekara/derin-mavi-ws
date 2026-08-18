#!/usr/bin/env python3
"""line_follower_controller_node.py

LineError mesajını (x_error, angle_error, is_line_lost) 6 eksenli
(surge, sway, heave, roll, pitch, yaw) araç hareket komutuna dönüştürür.

Çıkış: geometry_msgs/Twist (/auv/cmd_vel)
    linear.x  -> surge (ileri/geri)
    linear.y  -> sway  (sağ/sol yanal)
    linear.z  -> heave (yukarı/aşağı) -- bu düğümde pass-through / 0,
                 ayrı bir derinlik kontrolcüsüne bırakılmıştır
    angular.x -> roll  -- bu düğümde 0, stabilizasyon ayrı katmanda
    angular.y -> pitch -- bu düğümde 0, stabilizasyon ayrı katmanda
    angular.z -> yaw   (dönme)

Tasarım notu: Bu düğüm SADECE hat takibinden doğan surge/sway/yaw
komutlarını üretir. Roll/pitch/heave için 0 (pass-through) yayınlar ki
sistem, bu eksenleri yöneten ayrı kontrolcülerle (örn. derinlik/IMU
stabilizasyon düğümü) çakışmadan çalışabilsin. Gelecekte bu düğüme
depth_setpoint / attitude_setpoint gibi ek girdiler kolayca eklenebilir.

Çıkış tipi olarak bilinçli olarak standart geometry_msgs/Twist seçildi;
böylece mevcut/gelecek herhangi bir "thruster tahsisi" (thruster
allocation / mixer) düğümüyle uyumlu, jenerik bir arayüz sağlanır.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from line_interfaces.msg import LineError

from line_control.pid_controller import PIDController
from line_control.line_follower_state_machine import (
    LineFollowerStateMachine,
    LineFollowerState,
)


class LineFollowerControllerNode(Node):
    def __init__(self):
        super().__init__('line_follower_controller_node')

        self._declare_parameters()
        self._read_parameters()

        self.yaw_pid = PIDController(
            self.yaw_kp, self.yaw_ki, self.yaw_kd,
            output_limits=(-self.max_yaw_rate, self.max_yaw_rate),
        )
        self.sway_pid = PIDController(
            self.sway_kp, self.sway_ki, self.sway_kd,
            output_limits=(-self.max_sway_speed, self.max_sway_speed),
        )

        self.state_machine = LineFollowerStateMachine(
            hold_timeout_sec=self.hold_timeout_sec
        )

        self._search_direction = 1.0
        self._last_msg_time = None

        self.error_sub = self.create_subscription(
            LineError, self.line_error_topic, self.line_error_callback,
            qos_profile_sensor_data,
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        # Watchdog: LineError akışı tamamen kesilirse (düğüm çöktü vb.)
        # aracı güvenli şekilde durdur.
        self.watchdog_timer = self.create_timer(
            self.watchdog_period_sec, self._watchdog_check
        )

        self.get_logger().info(
            f"Line Follower Controller başladı. Girdi: {self.line_error_topic} "
            f"-> Çıktı: {self.cmd_vel_topic}"
        )

    # ------------------------------------------------------------------
    # Parametreler
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        self.declare_parameter('line_error_topic', '/line_follower/line_error')
        self.declare_parameter('cmd_vel_topic', '/auv/cmd_vel')
        self.declare_parameter('state_topic', '/line_follower/state')

        self.declare_parameter('yaw_kp', 0.02)
        self.declare_parameter('yaw_ki', 0.0)
        self.declare_parameter('yaw_kd', 0.005)
        self.declare_parameter('max_yaw_rate', 0.6)

        self.declare_parameter('sway_kp', 0.004)
        self.declare_parameter('sway_ki', 0.0)
        self.declare_parameter('sway_kd', 0.001)
        self.declare_parameter('max_sway_speed', 0.3)

        self.declare_parameter('nominal_surge_speed', 0.35)
        self.declare_parameter('min_surge_speed', 0.05)

        self.declare_parameter('search_yaw_rate', 0.25)
        self.declare_parameter('hold_timeout_sec', 8.0)
        self.declare_parameter('watchdog_period_sec', 0.5)
        self.declare_parameter('message_timeout_sec', 1.0)

    def _read_parameters(self):
        gp = self.get_parameter
        self.line_error_topic = gp('line_error_topic').value
        self.cmd_vel_topic = gp('cmd_vel_topic').value
        self.state_topic = gp('state_topic').value

        self.yaw_kp = float(gp('yaw_kp').value)
        self.yaw_ki = float(gp('yaw_ki').value)
        self.yaw_kd = float(gp('yaw_kd').value)
        self.max_yaw_rate = float(gp('max_yaw_rate').value)

        self.sway_kp = float(gp('sway_kp').value)
        self.sway_ki = float(gp('sway_ki').value)
        self.sway_kd = float(gp('sway_kd').value)
        self.max_sway_speed = float(gp('max_sway_speed').value)

        self.nominal_surge_speed = float(gp('nominal_surge_speed').value)
        self.min_surge_speed = float(gp('min_surge_speed').value)

        self.search_yaw_rate = float(gp('search_yaw_rate').value)
        self.hold_timeout_sec = float(gp('hold_timeout_sec').value)
        self.watchdog_period_sec = float(gp('watchdog_period_sec').value)
        self.message_timeout_sec = float(gp('message_timeout_sec').value)

    # ------------------------------------------------------------------
    # Ana kontrol döngüsü
    # ------------------------------------------------------------------
    def line_error_callback(self, msg: LineError) -> None:
        now = time.monotonic()
        self._last_msg_time = now

        state = self.state_machine.update(msg.is_line_lost, now)

        if state == LineFollowerState.TRACKING:
            twist = self._compute_tracking_command(msg, now)
        elif state == LineFollowerState.LINE_LOST:
            twist = self._compute_search_command()
        elif state == LineFollowerState.HOLD:
            twist = self._compute_zero_command()
        else:  # SEARCHING (henüz hiç çizgi görülmedi)
            twist = self._compute_zero_command()

        self.cmd_pub.publish(twist)
        self._publish_state(state)

    def _compute_tracking_command(self, msg: LineError, now: float) -> Twist:
        yaw_rate = self.yaw_pid.update(msg.angle_error, now)
        sway_speed = self.sway_pid.update(msg.x_error, now)

        # Açı hatası büyüdükçe ileri hızı azalt (keskin dönüşlerde
        # daha güvenli / kontrollü hareket için)
        angle_rad = math.radians(msg.angle_error)
        speed_factor = max(0.0, math.cos(angle_rad))
        surge_speed = max(
            self.min_surge_speed, self.nominal_surge_speed * speed_factor
        )

        self._search_direction = 1.0 if yaw_rate >= 0.0 else -1.0

        twist = Twist()
        twist.linear.x = surge_speed
        twist.linear.y = sway_speed
        twist.linear.z = 0.0    # heave: ayrı derinlik kontrolcüsüne bırakıldı
        twist.angular.x = 0.0   # roll: ayrı stabilizasyon katmanına bırakıldı
        twist.angular.y = 0.0   # pitch: ayrı stabilizasyon katmanına bırakıldı
        twist.angular.z = yaw_rate
        return twist

    def _compute_search_command(self) -> Twist:
        """Çizgi kaybedildiğinde son bilinen dönüş yönünde yavaşça dönerek
        çizgiyi yeniden bulmaya çalışır."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.search_yaw_rate * self._search_direction
        return twist

    @staticmethod
    def _compute_zero_command() -> Twist:
        return Twist()

    def _publish_state(self, state: LineFollowerState) -> None:
        msg = String()
        msg.data = state.name
        self.state_pub.publish(msg)

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------
    def _watchdog_check(self) -> None:
        if self._last_msg_time is None:
            return
        elapsed = time.monotonic() - self._last_msg_time
        if elapsed > self.message_timeout_sec:
            self.get_logger().warn(
                f"LineError mesajı {elapsed:.2f}s boyunca alınamadı. "
                "Güvenlik için araç durduruluyor."
            )
            self.yaw_pid.reset()
            self.sway_pid.reset()
            self.cmd_pub.publish(self._compute_zero_command())
            self.state_machine.state = LineFollowerState.HOLD
            self._publish_state(LineFollowerState.HOLD)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
