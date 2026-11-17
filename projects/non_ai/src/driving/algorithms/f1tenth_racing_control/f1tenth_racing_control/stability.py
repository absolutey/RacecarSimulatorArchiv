import csv
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def clamp(value, low, high):
    return min(high, max(low, value))


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def rate_limited(previous, target, rate_per_s, dt_s):
    return clamp(
        target,
        previous - rate_per_s * dt_s,
        previous + rate_per_s * dt_s,
    )


@dataclass(frozen=True)
class RecoveryCommand:
    active: bool
    mode: str
    reason: str
    steering_rad: float = 0.0
    acceleration_mps2: float = 0.0
    target_speed_mps: float = 0.0
    slip_margin: float = math.nan
    yaw_rate_margin: float = math.nan
    physical_clearance_m: float = math.nan


class CausalEntryDetector:
    def __init__(
        self,
        entry_margin_threshold=0.0,
        trend_margin_ceiling=0.6,
        time_to_zero_threshold_s=0.15,
        trend_window_s=0.10,
    ):
        self.entry_margin_threshold = float(entry_margin_threshold)
        self.trend_margin_ceiling = float(trend_margin_ceiling)
        self.time_to_zero_threshold_s = float(time_to_zero_threshold_s)
        self.trend_window_s = float(trend_window_s)
        self.history = deque(maxlen=16)

    def reset(self):
        self.history.clear()

    def update(self, now_s, slip_margin, yaw_rate_margin):
        now_s = float(now_s)
        slip_margin = float(slip_margin)
        yaw_rate_margin = float(yaw_rate_margin)

        immediate = min(slip_margin, yaw_rate_margin) <= self.entry_margin_threshold
        target = now_s - self.trend_window_s
        prior = min(self.history, key=lambda item: abs(item[0] - target)) if self.history else None

        predicted = False
        if prior is not None:
            dt_s = now_s - prior[0]
            if dt_s >= 0.05:
                for margin, old_margin in (
                    (slip_margin, prior[1]),
                    (yaw_rate_margin, prior[2]),
                ):
                    slope = (margin - old_margin) / dt_s
                    if (
                        margin <= self.trend_margin_ceiling
                        and slope < -1e-9
                        and margin / (-slope) <= self.time_to_zero_threshold_s
                    ):
                        predicted = True
                        break

        self.history.append((now_s, slip_margin, yaw_rate_margin))
        return immediate or predicted


class PhysicalTrackCorridor:
    def __init__(self, centerline_csv, footprint):
        with Path(centerline_csv).open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        if len(rows) < 3:
            raise ValueError("track centerline requires at least three samples")

        self.x = np.asarray([float(row["x_m"]) for row in rows], dtype=float)
        self.y = np.asarray([float(row["y_m"]) for row in rows], dtype=float)
        self.left = np.asarray([float(row["w_tr_left_m"]) for row in rows], dtype=float)
        self.right = np.asarray([float(row["w_tr_right_m"]) for row in rows], dtype=float)
        self.x2 = np.roll(self.x, -1)
        self.y2 = np.roll(self.y, -1)
        self.left2 = np.roll(self.left, -1)
        self.right2 = np.roll(self.right, -1)
        self.vx = self.x2 - self.x
        self.vy = self.y2 - self.y
        self.length2 = self.vx * self.vx + self.vy * self.vy
        self.length = np.sqrt(np.maximum(self.length2, 1e-12))
        self.corners = tuple(
            (float(footprint[x_key]), float(footprint[y_key]))
            for x_key in ("min_x_m", "max_x_m")
            for y_key in ("min_y_m", "max_y_m")
        )

    def _point_clearance(self, px, py):
        wx = px - self.x
        wy = py - self.y
        t = np.divide(
            wx * self.vx + wy * self.vy,
            self.length2,
            out=np.zeros_like(self.length2),
            where=self.length2 > 1e-12,
        )
        t = np.clip(t, 0.0, 1.0)
        qx = self.x + t * self.vx
        qy = self.y + t * self.vy
        dx = px - qx
        dy = py - qy
        index = int(np.argmin(dx * dx + dy * dy))
        signed = (self.vx[index] * dy[index] - self.vy[index] * dx[index]) / self.length[index]
        left = (1.0 - t[index]) * self.left[index] + t[index] * self.left2[index]
        right = (1.0 - t[index]) * self.right[index] + t[index] * self.right2[index]
        return min(left - signed, right + signed)

    def clearance(self, px, py, yaw):
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        values = []
        for bx, by in self.corners:
            wx = px + cosine * bx - sine * by
            wy = py + sine * bx + cosine * by
            values.append(self._point_clearance(wx, wy))
        return min(values)


class StabilityRecoverySupervisor:
    STABILIZE = {
        "kd": 1.0683980825844133,
        "kh": -0.7670196187092215,
        "ks": -3.0,
        "ky": -1.036771205453209,
        "a0": -1.444000837011402,
        "ad": 1.094461061060822,
        "ah": 2.397887591373083,
        "aslip": 0.0,
        "ayaw": 1.8417661659206526,
    }
    REJOIN = {
        "kd": -2.545913648612398,
        "kh": -1.633808322127725,
        "ks": 0.1878039038640078,
        "ky": -0.4884291452201821,
        "speed_cap_mps": 4.079008912173433,
        "kv": 2.6070174439757268,
        "accel_bias": -1.622815797456,
        "brake_d": 5.322969754879472,
        "brake_heading": 4.572165000953501,
        "brake_yaw": 2.9535016364511884,
    }

    def __init__(self, mpc, governor, dynamics, limits, environment, sim_share):
        self.mpc = mpc
        self.governor = governor
        self.dynamics = dynamics
        self.limits = limits
        self.wheelbase_m = float(dynamics["dynamic_wheelbase_m"])
        centerline = Path(sim_share) / environment["map"]["centerline_file"]
        footprint = environment["collision"]["footprint_relative_to_base_link"]
        self.corridor = PhysicalTrackCorridor(centerline, footprint)

        slip_fields = []
        for name in dir(mpc.config):
            if "slip" in name.lower() and "limit" in name.lower():
                value = getattr(mpc.config, name)
                if isinstance(value, (int, float)):
                    slip_fields.append(name)
        if len(slip_fields) != 1:
            raise ValueError(f"expected exactly one MPC slip limit field, got {slip_fields}")
        self.slip_limit_rad = float(getattr(mpc.config, slip_fields[0]))
        if self.slip_limit_rad <= 0.0:
            raise ValueError("slip limit must be positive")

        self.entry = CausalEntryDetector()
        self.mode = "NORMAL"
        self.mode_entered_s = None

    def reset(self):
        self.entry.reset()
        self.mode = "NORMAL"
        self.mode_entered_s = None

    def _dynamic_margins(self, message):
        slip_margin = (
            self.slip_limit_rad - abs(float(message.slip_angle))
        ) / self.slip_limit_rad
        speed = max(float(message.v), 0.5)
        yaw_limit = float(self.mpc.config.lateral_acceleration_limit_mps2) / speed
        yaw_margin = (yaw_limit - abs(float(message.omega))) / yaw_limit
        return slip_margin, yaw_margin

    def _projection(self, reference, message):
        projection = reference.geometry.project(float(message.px), float(message.py))
        physical = reference.geometry.interpolate(projection.s)
        station = float(reference.station_at_physical(physical))
        heading_error = wrap_angle(
            float(message.yaw) - float(reference.angle("heading_rad", station))
        )
        return projection, station, heading_error

    def _nominal_steering(self, reference, station):
        curvature = float(reference.scalar("curvature_per_m", station))
        return math.atan(self.wheelbase_m * curvature)

    def _stabilize_control(self, reference, message, station, previous, dt_s):
        projection, _, heading = self._projection(reference, message)
        p = self.STABILIZE
        steering_target = (
            self._nominal_steering(reference, station)
            + p["kd"] * float(projection.d)
            + p["kh"] * heading
            + p["ks"] * float(message.slip_angle)
            + p["ky"] * float(message.omega)
        )
        steering_target = clamp(
            steering_target,
            -float(self.limits["steering_angle_abs_max_rad"]),
            float(self.limits["steering_angle_abs_max_rad"]),
        )
        steering = rate_limited(
            previous[0],
            steering_target,
            float(self.limits["steering_rate_abs_max_rad_s"]),
            dt_s,
        )
        acceleration_target = (
            p["a0"]
            - p["ad"] * abs(float(projection.d))
            - p["ah"] * abs(heading)
            - p["aslip"] * abs(float(message.slip_angle))
            - p["ayaw"] * abs(float(message.omega))
        )
        acceleration_target = clamp(
            acceleration_target,
            -float(self.limits["deceleration_max_m_s2"]),
            float(self.limits["acceleration_max_m_s2"]),
        )
        acceleration = rate_limited(
            previous[1],
            acceleration_target,
            float(self.limits["jerk_abs_max_m_s3"]),
            dt_s,
        )
        return steering, acceleration

    def _rejoin_control(self, reference, envelope, message, station, previous, dt_s):
        projection, _, heading = self._projection(reference, message)
        p = self.REJOIN
        steering_target = (
            self._nominal_steering(reference, station)
            + p["kd"] * float(projection.d)
            + p["kh"] * heading
            + p["ks"] * float(message.slip_angle)
            + p["ky"] * float(message.omega)
        )
        steering_target = clamp(
            steering_target,
            -float(self.limits["steering_angle_abs_max_rad"]),
            float(self.limits["steering_angle_abs_max_rad"]),
        )
        steering = rate_limited(
            previous[0],
            steering_target,
            float(self.limits["steering_rate_abs_max_rad_s"]),
            dt_s,
        )
        target_speed = min(
            float(envelope.at(station).execution_envelope_mps),
            p["speed_cap_mps"],
        )
        acceleration_target = (
            p["accel_bias"]
            + p["kv"] * (target_speed - float(message.v))
            - p["brake_d"] * abs(float(projection.d))
            - p["brake_heading"] * abs(heading)
            - p["brake_yaw"] * abs(float(message.omega))
        )
        acceleration_target = clamp(
            acceleration_target,
            -float(self.limits["deceleration_max_m_s2"]),
            float(self.limits["acceleration_max_m_s2"]),
        )
        acceleration = rate_limited(
            previous[1],
            acceleration_target,
            float(self.limits["jerk_abs_max_m_s3"]),
            dt_s,
        )
        return steering, acceleration, target_speed

    def _overall_margin(self, reference, envelope, message):
        vector = np.asarray([
            float(message.px),
            float(message.py),
            float(message.yaw),
            float(message.v),
            float(message.slip_angle),
            float(message.omega),
            float(message.steer),
            float(message.accel),
        ], dtype=float)
        return self.governor._margin(reference, envelope, vector)

    def update(self, reference, envelope, message, previous_control, now_s, dt_s):
        if reference is None or envelope is None:
            self.reset()
            return RecoveryCommand(False, "NORMAL", "NO_REFERENCE")

        dt_s = clamp(float(dt_s), 1e-4, 0.05)
        now_s = float(now_s)
        slip_margin, yaw_margin = self._dynamic_margins(message)
        physical_clearance = self.corridor.clearance(
            float(message.px), float(message.py), float(message.yaw)
        )
        _, station, _ = self._projection(reference, message)
        trigger = self.entry.update(now_s, slip_margin, yaw_margin)

        if physical_clearance < 0.0:
            # This is already outside the verified track corridor. Do not
            # label it dynamically safe or attempt a speculative rejoin.
            # This braking request is NOT a stopping certificate; the
            # independent arbiter must treat the emergency status accordingly.
            self.mode = "BOUNDARY_EMERGENCY"
            self.mode_entered_s = now_s
            decel = -float(self.limits["deceleration_max_m_s2"])
            return RecoveryCommand(
                True, "BOUNDARY_EMERGENCY", "NEGATIVE_PHYSICAL_CLEARANCE",
                float(previous_control[0]), decel, 0.0,
                slip_margin, yaw_margin, physical_clearance,
            )
        if self.mode == "BOUNDARY_EMERGENCY":
            # Do not silently resume regular driving: a new independent
            # qualification/reset is required after a boundary violation.
            return RecoveryCommand(
                True, "BOUNDARY_EMERGENCY", "REQUIRES_EXPLICIT_RESET",
                float(previous_control[0]), -float(self.limits["deceleration_max_m_s2"]),
                0.0, slip_margin, yaw_margin, physical_clearance,
            )

        if self.mode == "NORMAL":
            if not trigger:
                return RecoveryCommand(
                    False, "NORMAL", "DYNAMIC_MARGIN_OK",
                    slip_margin=slip_margin,
                    yaw_rate_margin=yaw_margin,
                    physical_clearance_m=physical_clearance,
                )
            self.mode = "STABILIZE"
            self.mode_entered_s = now_s

        if self.mode == "STABILIZE":
            elapsed = now_s - self.mode_entered_s
            if (
                elapsed >= 0.10
                and slip_margin >= 0.0
                and yaw_margin >= 0.0
                and physical_clearance >= 0.0
            ):
                self.mode = "REJOIN"
                self.mode_entered_s = now_s
            else:
                steering, acceleration = self._stabilize_control(
                    reference, message, station, previous_control, dt_s
                )
                target_speed = min(
                    float(message.v),
                    float(envelope.at(station).execution_envelope_mps),
                )
                reason = "STABILIZE_TIMEOUT_CONTINUE" if elapsed > 0.40 else "CAUSAL_DYNAMIC_RISK"
                return RecoveryCommand(
                    True, "STABILIZE", reason,
                    steering, acceleration, target_speed,
                    slip_margin, yaw_margin, physical_clearance,
                )

        if self.mode == "REJOIN":
            if slip_margin < 0.0 or yaw_margin < 0.0:
                self.mode = "STABILIZE"
                self.mode_entered_s = now_s
                steering, acceleration = self._stabilize_control(
                    reference, message, station, previous_control, dt_s
                )
                target_speed = min(
                    float(message.v),
                    float(envelope.at(station).execution_envelope_mps),
                )
                return RecoveryCommand(
                    True, "STABILIZE", "REJOIN_DYNAMIC_RELAPSE",
                    steering, acceleration, target_speed,
                    slip_margin, yaw_margin, physical_clearance,
                )

            overall_margin, overall_reason = self._overall_margin(reference, envelope, message)
            if overall_margin >= 0.0 and physical_clearance >= 0.0:
                self.mode = "NORMAL"
                self.mode_entered_s = None
                return RecoveryCommand(
                    False, "NORMAL", "REJOIN_COMPLETE",
                    slip_margin=slip_margin,
                    yaw_rate_margin=yaw_margin,
                    physical_clearance_m=physical_clearance,
                )

            steering, acceleration, target_speed = self._rejoin_control(
                reference, envelope, message, station, previous_control, dt_s
            )
            return RecoveryCommand(
                True, "REJOIN", f"REJOIN_PENDING:{overall_reason}",
                steering, acceleration, target_speed,
                slip_margin, yaw_margin, physical_clearance,
            )

        raise RuntimeError(f"unknown recovery mode: {self.mode}")
