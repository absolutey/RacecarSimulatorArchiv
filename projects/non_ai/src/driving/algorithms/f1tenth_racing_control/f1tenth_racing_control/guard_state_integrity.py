"""Independent, ROS-free guard sensor numeric contract.

A corrupted physics state must NEVER produce a new positive or braking
actuation lease. On invalid state, the independently latched simulator
watchdog brakes by lease expiry, rather than using poisoned kinematics.
"""
import math

_FIELDS = ('px','py','yaw','v','vx','vy','omega','a','ax','ay',
           'slip_angle','accel','jerk','steer','steer_vel')


def finite_physics_state(state):
    if state is None:
        return False
    try:
        return all(type(getattr(state,key)) is not bool and
                   math.isfinite(float(getattr(state,key)))
                   for key in _FIELDS)
    except (AttributeError,ValueError,TypeError,OverflowError):
        return False
