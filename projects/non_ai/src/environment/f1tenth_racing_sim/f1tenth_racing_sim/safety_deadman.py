"""Simulator-local last-resort brake target; independent of ROS controller processes.

This is an actuator-input safety net, not obstacle avoidance or certified Guard.
The integrator has no rolling drag; neutral acceleration is NOT a stop.
"""
import math


def stale_brake_target(speed_mps, deceleration_limit_mps2):
    if not math.isfinite(speed_mps) or not math.isfinite(deceleration_limit_mps2):
        raise ValueError("deadman non-finite state/limit")
    if deceleration_limit_mps2 <= 0.0:
        raise ValueError("deadman deceleration limit must be positive")
    return (-math.copysign(deceleration_limit_mps2, speed_mps)
            if abs(speed_mps) > 1e-9 else 0.0)


def stop_crossed_zero(previous_speed_mps, new_speed_mps):
    """Do not let sustained stale braking cause a new reverse-motion episode."""
    return (abs(previous_speed_mps) <= 1e-9 or
            previous_speed_mps * new_speed_mps <= 0.0)


def hold_stationary(state):
    """Model a static parking hold once the braking trajectory reaches zero."""
    state.speed = 0.0
    state.vx = state.vy = state.yaw_rate = 0.0
    state.acceleration = state.requested_acceleration = 0.0
    state.jerk = 0.0
