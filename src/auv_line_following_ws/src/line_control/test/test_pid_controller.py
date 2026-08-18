from line_control.pid_controller import PIDController


def test_proportional_only():
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, output_limits=(-10, 10))
    output = pid.update(2.0, current_time=0.0)
    assert output == 2.0


def test_output_clamped_to_limits():
    pid = PIDController(kp=10.0, ki=0.0, kd=0.0, output_limits=(-1.0, 1.0))
    output = pid.update(5.0, current_time=0.0)
    assert output == 1.0


def test_reset_clears_internal_state():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, output_limits=(-10, 10))
    pid.update(1.0, current_time=0.0)
    pid.update(1.0, current_time=1.0)  # integral birikir
    pid.reset()
    assert pid._integral == 0.0
    assert pid._prev_time is None
