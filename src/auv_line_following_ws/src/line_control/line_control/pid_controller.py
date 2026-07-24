"""Basit, yeniden kullanılabilir PID kontrolcüsü.

Bu sınıf, line_control paketindeki tüm eksenler (yaw, sway, vb.) için ortak
olarak kullanılır; böylece kontrol mantığı tek bir yerde test edilir ve
bakımı kolaylaşır. ROS'a bağımlılığı yoktur, saf Python'dur -> birim testi
kolaydır (bkz. test/test_pid_controller.py).
"""

import time


class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-1.0, 1.0), integral_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output, self.max_output = output_limits
        self.integral_limits = integral_limits or output_limits

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def reset(self):
        """İç durumu sıfırlar (ör. çizgi yeniden bulunduğunda integral
        windup'ı önlemek için çağrılır)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def update(self, error: float, current_time: float = None) -> float:
        current_time = current_time if current_time is not None else time.monotonic()

        if self._prev_time is None:
            dt = 0.0
        else:
            dt = max(current_time - self._prev_time, 1e-6)

        self._integral += error * dt
        self._integral = self._clamp(self._integral, *self.integral_limits)

        derivative = 0.0 if dt == 0.0 else (error - self._prev_error) / dt

        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)
        output = self._clamp(output, self.min_output, self.max_output)

        self._prev_error = error
        self._prev_time = current_time

        return output

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))
