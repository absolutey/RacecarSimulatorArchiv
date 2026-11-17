import math
from dataclasses import dataclass

from ..common.geometry import clamp, wrap
from .pacejka import lateral_force


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    acceleration: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    slip: float = 0.0
    requested_acceleration: float = 0.0
    jerk: float = 0.0
    steering: float = 0.0
    steering_rate: float = 0.0


def _blend_fraction(value):
    """C1-continuous mixing fraction; zero slope at both endpoints."""
    t = max(0.0, min(1.0, float(value)))
    return t * t * (3.0 - 2.0 * t)


class Dynamics:
    def __init__(self, config):
        self.c = config
        low = config.get("low_speed_transition", {})
        self.low_speed_enter = float(low.get("enter_mps", 0.5))
        self.low_speed_exit = float(low.get("exit_mps", 1.5))
        self.low_speed_tau = float(low.get("response_time_s", 0.12))
        if not (0.0 < self.low_speed_enter < self.low_speed_exit and
                self.low_speed_tau > 0.0 and all(math.isfinite(v) for v in (
                    self.low_speed_enter, self.low_speed_exit, self.low_speed_tau))):
            raise ValueError("INVALID_LOW_SPEED_TRANSITION")

    def step(self, state, requested_steering, requested_acceleration, dt):
        limits = self.c["limits"]
        old_acceleration = state.requested_acceleration
        steering_delta = clamp(
            requested_steering - state.steering,
            -limits["steering_rate_abs_max_rad_s"] * dt,
            limits["steering_rate_abs_max_rad_s"] * dt,
        )
        state.steering_rate = steering_delta / dt
        state.steering = clamp(
            state.steering + steering_delta,
            -limits["steering_angle_abs_max_rad"],
            limits["steering_angle_abs_max_rad"],
        )
        acceleration_delta = clamp(
            requested_acceleration - old_acceleration,
            -limits["jerk_abs_max_m_s3"] * dt,
            limits["jerk_abs_max_m_s3"] * dt,
        )
        state.jerk = acceleration_delta / dt
        state.requested_acceleration = clamp(
            old_acceleration + acceleration_delta,
            -limits["deceleration_max_m_s2"],
            limits["acceleration_max_m_s2"],
        )
        state.acceleration = state.requested_acceleration

        # A single dynamics authority for both simulator and MPC rollout.
        # At near-zero speed, beta is not a well-conditioned tire-slip state.
        # Use a finite kinematic bicycle target, then blend *state derivatives*
        # (not two independently integrated states) with the Pacejka model.
        v = state.speed
        # At an exact standstill residual yaw-rate is not a valid turning
        # motion for this steering-driven vehicle.  Preserve yaw and discard
        # only the unphysical residual, as the pre-existing ROS regression
        # contract requires.  Moving-speed blended dynamics remain unchanged.
        if abs(v) < 1.0e-8:
            state.yaw_rate = 0.0
        lr = float(self.c["rear_axle_from_cg_m"])
        lf = float(self.c["front_axle_from_cg_m"])
        wheelbase = lf + lr
        beta_target = math.atan2(lr * math.tan(state.steering), wheelbase)
        # At a physical standstill beta has no directional meaning. Approach 0
        # continuously as the velocity approaches zero.
        beta_target *= _blend_fraction(abs(v) / self.low_speed_enter)
        yaw_target = v * math.cos(beta_target) * math.tan(state.steering) / wheelbase
        low_beta_dot = wrap(beta_target - state.slip) / self.low_speed_tau
        low_yaw_dot = (yaw_target - state.yaw_rate) / self.low_speed_tau

        alpha = _blend_fraction(
            (abs(v) - self.low_speed_enter) /
            (self.low_speed_exit - self.low_speed_enter)
        )
        if alpha > 0.0:
            # Never evaluate the singular tire expression outside its valid
            # domain; the blend coefficient and its first derivative vanish
            # at the entry threshold.
            front_slip = -math.atan2(
                state.vy + lf * state.yaw_rate, state.vx,
            ) + state.steering
            rear_slip = -math.atan2(
                state.vy - lr * state.yaw_rate, state.vx,
            )
            front_force = lateral_force(self.c["front_tire"], front_slip)
            rear_force = lateral_force(self.c["rear_tire"], rear_slip)
            tire_beta_dot = (
                (front_force + rear_force) / (self.c["mass_kg"] * v)
                - state.yaw_rate
            )
            tire_yaw_dot = (
                lf * front_force * math.cos(state.steering)
                - lr * rear_force
            ) / self.c["yaw_inertia_kg_m2"]
        else:
            tire_beta_dot = tire_yaw_dot = 0.0
        slip_dot = (1.0 - alpha) * low_beta_dot + alpha * tire_beta_dot
        yaw_rate_dot = (1.0 - alpha) * low_yaw_dot + alpha * tire_yaw_dot
        if not all(math.isfinite(v) for v in (slip_dot, yaw_rate_dot)):
            raise ArithmeticError("DYNAMICS_NONFINITE_DERIVATIVE")

        state.x += state.speed * math.cos(state.yaw + state.slip) * dt
        state.y += state.speed * math.sin(state.yaw + state.slip) * dt
        state.yaw = wrap(state.yaw + state.yaw_rate * dt)
        state.slip = wrap(state.slip + slip_dot * dt)
        state.speed = clamp(
            state.speed + state.acceleration * dt,
            -limits["speed_abs_max_m_s"],
            limits["speed_abs_max_m_s"],
        )
        state.vx = state.speed * math.cos(state.slip)
        state.vy = state.speed * math.sin(state.slip)
        state.yaw_rate += yaw_rate_dot * dt
        state.ax = (
            state.acceleration * math.cos(state.slip)
            - state.speed * state.yaw_rate * math.sin(state.slip)
        )
        state.ay = (
            state.acceleration * math.sin(state.slip)
            + state.speed * state.yaw_rate * math.cos(state.slip)
        )
        return state
