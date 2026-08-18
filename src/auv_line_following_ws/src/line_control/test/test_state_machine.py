from line_control.line_follower_state_machine import (
    LineFollowerStateMachine,
    LineFollowerState,
)


def test_transitions_to_tracking_when_line_visible():
    sm = LineFollowerStateMachine(hold_timeout_sec=5.0)
    state = sm.update(is_line_lost=False, now=0.0)
    assert state == LineFollowerState.TRACKING


def test_transitions_to_line_lost_immediately_after_loss():
    sm = LineFollowerStateMachine(hold_timeout_sec=5.0)
    sm.update(is_line_lost=False, now=0.0)
    state = sm.update(is_line_lost=True, now=1.0)
    assert state == LineFollowerState.LINE_LOST


def test_transitions_to_hold_after_timeout():
    sm = LineFollowerStateMachine(hold_timeout_sec=2.0)
    sm.update(is_line_lost=False, now=0.0)
    sm.update(is_line_lost=True, now=1.0)
    state = sm.update(is_line_lost=True, now=5.0)
    assert state == LineFollowerState.HOLD


def test_recovers_to_tracking_from_hold():
    sm = LineFollowerStateMachine(hold_timeout_sec=1.0)
    sm.update(is_line_lost=False, now=0.0)
    sm.update(is_line_lost=True, now=1.0)
    sm.update(is_line_lost=True, now=5.0)  # HOLD durumuna geçer
    state = sm.update(is_line_lost=False, now=6.0)
    assert state == LineFollowerState.TRACKING
