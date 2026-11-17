from bisect import bisect_right
from dataclasses import dataclass
import math

from f1tenth_racing_geometry import (
    ClosedTrackGeometry,
    interpolate_periodic_angle,
    interpolate_periodic_scalar,
    wrap_angle,
)


def clamp(value, low, high):
    return max(low, min(high, value))


@dataclass(frozen=True)
class ControlConfig:
    wheelbase_m: float
    lookahead_base_m: float
    lookahead_speed_gain_s: float
    lookahead_min_m: float
    lookahead_max_m: float
    speed_feedback_gain_per_s: float
    qualified_speed_cap_mps: float
    steering_max_rad: float
    steering_rate_max_rad_s: float
    acceleration_max_mps2: float
    braking_max_mps2: float
    jerk_max_mps3: float
    degraded_deceleration_mps2: float

    def validate(self):
        values = tuple(self.__dict__.values())
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("controller configuration values must be finite and positive")
        if not self.lookahead_min_m <= self.lookahead_base_m <= self.lookahead_max_m:
            raise ValueError("lookahead bounds do not contain base lookahead")
        if self.degraded_deceleration_mps2 > self.braking_max_mps2:
            raise ValueError("degraded deceleration exceeds Foundation braking limit")
        return self


@dataclass(frozen=True)
class EgoState:
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float

    @property
    def finite(self):
        return all(math.isfinite(value) for value in self.__dict__.values())


@dataclass(frozen=True)
class ControlDecision:
    steering_rad: float
    acceleration_mps2: float
    mode: str
    reason: str
    target_progress_m: float = 0.0
    target_speed_mps: float = 0.0
    lateral_error_m: float = 0.0
    heading_error_rad: float = 0.0
    qualification_cap_active: bool = False
    degradation_active: bool = False


class PursuitReference:
    def __init__(self, samples, station_period_m):
        self.samples = tuple(samples)
        self.station_period_m = float(station_period_m)
        if len(self.samples) < 3 or not math.isfinite(self.station_period_m):
            raise ValueError("reference requires finite closed-loop samples")
        self.stations = tuple(float(sample.s_m) for sample in self.samples)
        if self.stations[0] != 0.0 or any(
            a >= b for a, b in zip(self.stations, self.stations[1:])
        ):
            raise ValueError("reference stations must be strictly increasing from zero")
        self.geometry = ClosedTrackGeometry(
            tuple((sample.x_m, sample.y_m) for sample in self.samples)
        )
        # Active-reference revisions construct new objects. The sample fields are
        # immutable for one adopted revision, so validate and snapshot once.
        self._interpolation_values = {}
        for field in ("x_m", "y_m", "heading_rad", "curvature_per_m",
                      "target_speed_mps", "target_accel_mps2"):
            values = tuple(float(getattr(sample, field)) for sample in self.samples)
            if any(not math.isfinite(value) for value in values):
                raise ValueError("nonfinite reference interpolation field")
            self._interpolation_values[field] = values
        if (not self.station_period_m > 0.0 or
                any(not 0.0 <= station < self.station_period_m for station in self.stations)):
            raise ValueError("reference stations outside station period")

    def _interval(self, station):
        # Algebraic equivalent of geometry._periodic_interval after the
        # immutable station array has been validated at reference adoption.
        if not math.isfinite(station):
            raise ValueError("nonfinite reference station")
        query = station % self.station_period_m
        upper = bisect_right(self.stations, query)
        if upper == 0 or upper == len(self.stations):
            low, high = len(self.stations) - 1, 0
            span = self.stations[0] + self.station_period_m - self.stations[-1]
            offset = (query - self.stations[-1]) % self.station_period_m
        else:
            low, high = upper - 1, upper
            span = self.stations[high] - self.stations[low]
            offset = query - self.stations[low]
        return low, high, offset / span

    def station_at_physical(self, physical_sample):
        index = physical_sample.segment_index
        source = self.samples[index]
        following = self.samples[(index + 1) % len(self.samples)]
        along = math.hypot(physical_sample.x - source.x_m, physical_sample.y - source.y_m)
        fraction = clamp(along / source.ds_to_next_m, 0.0, 1.0)
        station_delta = (following.s_m - source.s_m) % self.station_period_m
        return (source.s_m + fraction * station_delta) % self.station_period_m

    def scalar(self, field, station):
        values = self._interpolation_values.get(field)
        if values is None:
            # Preserve behavior for infrequently requested custom fields.
            return interpolate_periodic_scalar(
                self.stations, tuple(getattr(sample, field) for sample in self.samples),
                station, self.station_period_m,
            )
        low, high, ratio = self._interval(station)
        return values[low] + ratio * (values[high] - values[low])

    def angle(self, field, station):
        values = self._interpolation_values.get(field)
        if values is None:
            return interpolate_periodic_angle(
                self.stations, tuple(getattr(sample, field) for sample in self.samples),
                station, self.station_period_m,
            )
        low, high, ratio = self._interval(station)
        return wrap_angle(values[low] + ratio * wrap_angle(values[high] - values[low]))


class ModelAidedPursuit:
    def __init__(self, config):
        self.config = config.validate()

    def degraded(self, previous_steering_rad, reason, speed_mps=None, dt_s=None):
        c = self.config
        deceleration = c.degraded_deceleration_mps2
        if (speed_mps is not None and dt_s is not None and
                math.isfinite(speed_mps) and math.isfinite(dt_s) and dt_s > 0.0):
            deceleration = min(deceleration, max(speed_mps, 0.0) / dt_s)
        return ControlDecision(
            steering_rad=clamp(previous_steering_rad, -c.steering_max_rad, c.steering_max_rad),
            acceleration_mps2=-deceleration,
            mode=reason,
            reason=reason,
            degradation_active=True,
        )

    def compute(self, reference, state, previous_steering_rad, previous_acceleration_mps2, dt_s):
        if not state.finite:
            return self.degraded(previous_steering_rad, "INVALID_STATE")
        if reference is None:
            return self.degraded(previous_steering_rad, "INVALID_REFERENCE", state.speed_mps, dt_s)
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            return self.degraded(previous_steering_rad, "INVALID_TIMING")
        c = self.config
        projection = reference.geometry.project(state.x_m, state.y_m)
        current_track = reference.geometry.interpolate(projection.s)
        current_station = reference.station_at_physical(current_track)
        lookahead = clamp(
            c.lookahead_base_m + c.lookahead_speed_gain_s * abs(state.speed_mps),
            c.lookahead_min_m,
            c.lookahead_max_m,
        )
        target_track = reference.geometry.interpolate(projection.s + lookahead)
        target_station = reference.station_at_physical(target_track)
        dx, dy = target_track.x - state.x_m, target_track.y - state.y_m
        target_distance = max(math.hypot(dx, dy), c.lookahead_min_m)
        alpha = wrap_angle(math.atan2(dy, dx) - state.yaw_rad)
        curvature = reference.scalar("curvature_per_m", target_station)
        nominal_alpha = math.asin(clamp(0.5 * curvature * lookahead, -0.95, 0.95))
        feedforward = math.atan(c.wheelbase_m * curvature)
        correction_curvature = 2.0 * math.sin(wrap_angle(alpha - nominal_alpha)) / target_distance
        steering_requested = feedforward + math.atan(c.wheelbase_m * correction_curvature)
        steering_requested = clamp(steering_requested, -c.steering_max_rad, c.steering_max_rad)
        steering = clamp(
            steering_requested,
            previous_steering_rad - c.steering_rate_max_rad_s * dt_s,
            previous_steering_rad + c.steering_rate_max_rad_s * dt_s,
        )
        # Longitudinal reference is indexed by the *current* projected
        # station. Steering still uses the lookahead station above.
        # R-A outgoing-segment acceleration is NOT an immediate actuator
        # command from an arbitrary lookahead segment.
        current_speed = reference.scalar('target_speed_mps', current_station)
        upstream_speed = current_speed
        # Respect all upcoming spatial speed constraints before reaching
        # them, using the available braking magnitude. This is a proposal
        # envelope only; independent Guard still performs the full stop proof.
        # Match the independent StoppingEnvelope's certified nominal
        # braking magnitude (2.0 m/s²), not the simulator's emergency
        # absolute dynamics limit (40 m/s²). Guard still re-proves the stop.
        certified_profile_brake = min(c.braking_max_mps2, 2.0)
        forward_limit = current_speed
        for sample in reference.samples:
            distance = (sample.s_m - current_station) % reference.station_period_m
            if distance <= 1e-9:
                continue
            v_next = float(sample.target_speed_mps)
            forward_limit = min(forward_limit, math.sqrt(max(
                0.0, v_next*v_next + 2.0*certified_profile_brake*distance
            )))
        desired_speed = min(current_speed, forward_limit)
        target_speed = min(desired_speed, c.qualified_speed_cap_mps)
        capped = desired_speed > c.qualified_speed_cap_mps
        # Feedforward is meaningful only near the authored velocity profile;
        # when far behind, feedback first synchronizes the physical state.
        # It must never cancel an urgent positive speed error with an
        # outgoing-segment braking value from a future curve.
        speed_error = target_speed - state.speed_mps
        tracking_band = max(0.5, 0.10 * max(1.0, target_speed))
        acceleration_feedforward = (
            clamp(reference.scalar('target_accel_mps2', current_station),
                  -c.braking_max_mps2, c.acceleration_max_mps2)
            if not capped and abs(speed_error) <= tracking_band else 0.0
        )
        acceleration_requested = acceleration_feedforward + c.speed_feedback_gain_per_s * speed_error
        acceleration_requested = clamp(
            acceleration_requested,
            -c.braking_max_mps2,
            c.acceleration_max_mps2,
        )
        acceleration = clamp(
            acceleration_requested,
            previous_acceleration_mps2 - c.jerk_max_mps3 * dt_s,
            previous_acceleration_mps2 + c.jerk_max_mps3 * dt_s,
        )
        return ControlDecision(
            steering_rad=steering,
            acceleration_mps2=acceleration,
            mode="LOCAL_SPEED_CAPPED" if capped else "NORMAL_TRACKING",
            reason="QUALIFICATION_CAP_ACTIVE" if capped else "TRACKING",
            target_progress_m=target_station,
            target_speed_mps=target_speed,
            lateral_error_m=projection.d,
            heading_error_rad=wrap_angle(reference.angle("heading_rad", current_station) - state.yaw_rad),
            qualification_cap_active=capped,
        )
