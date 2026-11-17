from dataclasses import dataclass, replace
import hashlib
import json
import math

from f1tenth_racing_geometry import (
    ClosedTrackGeometry,
    interpolate_periodic_scalar,
    interpolate_periodic_vector,
    wrap_angle,
)


@dataclass(frozen=True)
class OvertakingConfig:
    prediction_max_age_s: float
    minimum_track_confidence: float
    minimum_confirmed_hits: int
    maximum_source_misses: int
    maximum_covariance_eigenvalue_m2: float
    eligibility_ahead_min_m: float
    eligibility_ahead_max_m: float
    minimum_closing_speed_mps: float
    candidate_lateral_offset_m: float
    prepare_length_m: float
    pass_length_m: float
    merge_length_m: float
    footprint_half_length_m: float
    footprint_half_width_m: float
    corridor_clearance_m: float
    blue_tracking_uncertainty_m: float
    qualification_margin_m: float
    pass_completion_ahead_m: float
    wheelbase_m: float
    steering_angle_max_rad: float
    steering_rate_max_rad_s: float
    candidate_speed_cap_mps: float
    racing_speed_cap_mps: float
    abort_merge_speed_mps: float
    abort_stop_speed_mps: float

    def __post_init__(self):
        values = tuple(vars(self).values())
        if any(not math.isfinite(float(value)) or float(value) < 0.0 for value in values):
            raise ValueError("overtaking configuration must be finite and nonnegative")


@dataclass(frozen=True)
class PredictionSample:
    dt_s: float
    x_m: float
    y_m: float
    heading_rad: float
    s_m: float
    d_m: float
    covariance_xy: tuple
    occupancy_half_length_m: float
    occupancy_half_width_m: float


@dataclass(frozen=True)
class Prediction:
    identity: str
    source_revision: int
    source_time_s: float
    source_time_ns: int
    receipt_time_s: float
    valid: bool
    lifecycle: str
    confidence: float
    hit_count: int
    miss_count: int
    track_id: str
    frame_id: str
    samples: tuple


@dataclass(frozen=True)
class TrajectorySample:
    sample_index: int
    s_m: float
    x_m: float
    y_m: float
    heading_rad: float
    curvature_per_m: float
    lateral_offset_m: float
    target_speed_mps: float
    target_accel_mps2: float
    ds_to_next_m: float


@dataclass(frozen=True)
class Plan:
    plan_id: str
    revision: int
    state: str
    side: str
    valid: bool
    reason: str
    source_prediction_id: str
    source_revision: int
    source_time_s: float
    source_time_ns: int
    start_s: float
    commit_end_s: float
    pass_end_s: float
    merge_end_s: float
    samples: tuple
    minimum_predicted_separation_m: float
    minimum_corridor_clearance_m: float
    committed_at_s: float


def _smoothstep(value):
    value = min(1.0, max(0.0, value))
    return value * value * value * (10.0 + value * (-15.0 + 6.0 * value))


def _quintic_connection(value, start_value, start_slope, end_value, end_slope, length):
    """C2 connector with zero endpoint second derivatives."""
    u = min(1.0, max(0.0, value / length))
    u2, u3 = u * u, u * u * u
    u4, u5 = u3 * u, u3 * u2
    return (
        (1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5) * start_value
        + (u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5) * length * start_slope
        + (10.0 * u3 - 15.0 * u4 + 6.0 * u5) * end_value
        + (-4.0 * u3 + 7.0 * u4 - 3.0 * u5) * length * end_slope
    )


def continuation_deadline_s(state, reason, committed_at_s, now_s, valid_until_s, timeout_s):
    if not committed_at_s:
        return 0.0
    moving_containment = (
        state == "BLOCKING_HOLD" or
        (state == "ABORT" and reason in (
            "COMMITTED_ABORT:TEMPORAL_CLEARANCE_LOST",
            "COMMITTED_ABORT:TRACKING_ERROR_EXCEEDED",
        ))
    )
    return (now_s if moving_containment else committed_at_s) + timeout_s


def _largest_covariance(covariance):
    a, b, c, d = covariance
    trace = a + d
    return 0.5 * (trace + math.sqrt(max(0.0, (a - d) ** 2 + 4.0 * b * c)))


class OvertakingSupervisor:
    """Small deterministic Frenet supervisor; it never owns actuator commands."""

    def __init__(self, config, centerline, left_widths, right_widths):
        self.config = config
        self.geometry = ClosedTrackGeometry(centerline)
        if len(left_widths) != self.geometry.segment_count or len(right_widths) != self.geometry.segment_count:
            raise ValueError("corridor widths must align with centerline")
        self.left_widths, self.right_widths = tuple(left_widths), tuple(right_widths)
        self.plan = None
        self.revision = 0
        self.degraded_since_s = None
        self.last_opponent_speed_mps = 0.0
        self.last_qualified_prediction = None
        self.parity_reached = False
        self.safe_ahead = False
        self.opponent_relation = "BEHIND"
        self.speed_constraint_owner = "NONE"
        self.geometry_authority = "NOMINAL"
        self.candidate_results = {}
        self.follow_started_s = None
        self.same_strategy_follow_duration_s = 0.0

    def _remember_relation(self, blue_s, prediction):
        """Advance maneuver-relative knowledge; never regress it on a sensor miss."""
        ahead = self.geometry.signed_delta_s(prediction.samples[0].s_m, blue_s)
        relation = ("CLEAR" if ahead >= self.config.pass_completion_ahead_m else
                    "AHEAD" if ahead > 0.0 else
                    "PARITY" if ahead >= 0.0 else "BEHIND")
        order = {"BEHIND": 0, "PARITY": 1, "AHEAD": 2, "CLEAR": 3}
        if order[relation] > order[self.opponent_relation]:
            self.opponent_relation = relation
        self.parity_reached = order[self.opponent_relation] >= order["PARITY"]
        self.safe_ahead = self.opponent_relation == "CLEAR"

    def _changed_payload(self, state, reason, samples, prediction, now_s, **changes):
        """Publish changed geometry/speed only with current qualified provenance."""
        self.revision += 1
        identity = json.dumps({
            "prediction": prediction.identity,
            "revision": self.revision,
            "state": state,
            "start_s": round(changes.get("start_s", self.plan.start_s), 9),
        }, sort_keys=True).encode()
        return replace(
            self.plan,
            plan_id="sha256:" + hashlib.sha256(identity).hexdigest(),
            revision=self.revision,
            state=state,
            reason=reason,
            samples=samples,
            source_prediction_id=prediction.identity,
            source_revision=prediction.source_revision,
            source_time_s=prediction.source_time_s,
            source_time_ns=prediction.source_time_ns,
            committed_at_s=now_s,
            **changes,
        )

    def _prediction_status(self, prediction, now_s, committed=False):
        c = self.config
        if prediction is None or not prediction.valid or not prediction.samples:
            return "PREDICTION_INVALID"
        if now_s - prediction.source_time_s > c.prediction_max_age_s:
            return "PREDICTION_STALE"
        if prediction.lifecycle not in ("TRACKED", "COASTING"):
            return "TRACK_NOT_CONFIRMED"
        if prediction.confidence < c.minimum_track_confidence or prediction.hit_count < c.minimum_confirmed_hits:
            return "TRACK_CONFIDENCE_LOW"
        if prediction.miss_count > c.maximum_source_misses:
            return "TRACK_MISSES_EXCESSIVE"
        first = prediction.samples[0]
        if _largest_covariance(first.covariance_xy) > c.maximum_covariance_eigenvalue_m2:
            return "PREDICTION_UNCERTAINTY_HIGH"
        return "PREDICTION_QUALIFIED"

    def _eligible(self, blue_s, blue_speed, prediction, now_s):
        c = self.config
        status = self._prediction_status(prediction, now_s)
        if status != "PREDICTION_QUALIFIED":
            return status
        first = prediction.samples[0]
        ahead = self.geometry.forward_delta_s(blue_s, first.s_m)
        if not c.eligibility_ahead_min_m <= ahead <= c.eligibility_ahead_max_m:
            return "OPPONENT_NOT_BLOCKING"
        red_speed = math.hypot(
            prediction.samples[-1].x_m - first.x_m,
            prediction.samples[-1].y_m - first.y_m,
        ) / max(1e-6, prediction.samples[-1].dt_s - first.dt_s)
        if blue_speed - red_speed < c.minimum_closing_speed_mps:
            return "NO_CLOSING_SPEED"
        return "ELIGIBLE"

    def _entry_slope(self, start_s, current_yaw):
        return math.tan(wrap_angle(current_yaw - self.geometry.interpolate(start_s).heading))

    def _placed(self, source, d):
        heading = self.geometry.interpolate(source.s_m).heading
        shift = d - source.lateral_offset_m
        return replace(
            source,
            x_m=source.x_m - shift * math.sin(heading),
            y_m=source.y_m + shift * math.cos(heading),
            lateral_offset_m=d,
        )

    def _pass_entry_d(self, source, start_s, current_d, current_slope, target_d, length):
        signed = self.geometry.signed_delta_s(start_s, source.s_m)
        if -length < signed < 0.0:
            return _quintic_connection(
                signed + length, target_d, 0.0, current_d, current_slope, length,
            )
        if 0.0 <= signed < length:
            return _quintic_connection(
                signed, current_d, current_slope, target_d, 0.0, length,
            )
        return target_d

    def _candidate(self, nominal_samples, start_s, side, entry_speed_mps=0.0,
                   current_d=None, current_yaw=None):
        total = self.geometry.length
        stations = tuple(sample.s_m for sample in nominal_samples)
        current_d = (interpolate_periodic_scalar(
            stations, tuple(sample.lateral_offset_m for sample in nominal_samples),
            start_s, total,
        ) if current_d is None else current_d)
        current_yaw = (self.geometry.interpolate(start_s).heading
                       if current_yaw is None else current_yaw)
        current_slope = self._entry_slope(start_s, current_yaw)
        target_d = (self.config.candidate_lateral_offset_m if side == "LEFT" else
                    -self.config.candidate_lateral_offset_m)
        reason = "TRACKING_ENTRY_INFEASIBLE"
        for length in (
                self.config.prepare_length_m,
                1.5 * self.config.prepare_length_m,
                2.0 * self.config.prepare_length_m,
                self.config.pass_length_m):
            generated = []
            for source in nominal_samples:
                d = self._pass_entry_d(
                    source, start_s, current_d, current_slope, target_d, length,
                )
                generated.append(replace(
                    self._placed(source, d),
                    target_speed_mps=min(
                        source.target_speed_mps, self.config.candidate_speed_cap_mps,
                    ),
                    target_accel_mps2=(
                        0.0 if source.target_speed_mps > self.config.candidate_speed_cap_mps
                        else source.target_accel_mps2
                    ),
                ))
            rebuilt, reason = self._rebuild(generated, entry_speed_mps)
            if rebuilt is None:
                continue
            clearance = self._corridor_clearance(rebuilt)
            if clearance >= self.config.corridor_clearance_m:
                return rebuilt, clearance, reason
            reason = "CORRIDOR_INFEASIBLE"
        return None, -math.inf, reason

    def _rebuild(self, generated, speed_floor_mps=0.0, speed_ceiling_mps=math.inf):
        points = [(sample.x_m, sample.y_m) for sample in generated]
        headings = tuple(math.atan2(
            points[(index + 1) % len(points)][1] - point[1],
            points[(index + 1) % len(points)][0] - point[0],
        ) for index, point in enumerate(points))
        rebuilt = []
        for index, sample in enumerate(generated):
            previous = (index - 1) % len(points)
            ds = 0.5 * (math.dist(points[previous], points[index]) +
                        math.dist(points[index], points[(index + 1) % len(points)]))
            curvature = wrap_angle(headings[index] - headings[previous]) / ds
            rebuilt.append(replace(
                sample,
                heading_rad=headings[index],
                curvature_per_m=curvature,
                ds_to_next_m=math.dist(points[index], points[(index + 1) % len(points)]),
            ))
        rebuilt = tuple(rebuilt)
        feasible, reason = self._steering_feasible(
            rebuilt, speed_floor_mps, speed_ceiling_mps,
        )
        return (rebuilt if feasible else None), reason

    def _nominal_connection_d(self, nominal_samples, source, start_s, current_d,
                              current_slope, length):
        stations = tuple(sample.s_m for sample in nominal_samples)
        nominal_start = interpolate_periodic_scalar(
            stations, tuple(sample.lateral_offset_m for sample in nominal_samples),
            start_s, self.geometry.length,
        )
        shift = current_d - nominal_start
        signed = self.geometry.signed_delta_s(start_s, source.s_m)
        if -length < signed < 0.0:
            relative = _quintic_connection(
                signed + length, 0.0, 0.0, shift, current_slope, length,
            )
            return source.lateral_offset_m + relative
        if 0.0 <= signed < length:
            relative = _quintic_connection(
                signed, shift, current_slope, 0.0, 0.0, length,
            )
            return source.lateral_offset_m + relative
        return source.lateral_offset_m

    def _abort_candidate(self, nominal_samples, start_s, current_d, speed_cap_mps,
                         current_yaw=None):
        current_yaw = (self.geometry.interpolate(start_s).heading
                       if current_yaw is None else current_yaw)
        current_slope = self._entry_slope(start_s, current_yaw)
        generated = []
        for source in nominal_samples:
            d = self._nominal_connection_d(
                nominal_samples, source, start_s, current_d, current_slope,
                self.config.merge_length_m,
            )
            generated.append(replace(
                self._placed(source, d),
                target_speed_mps=min(source.target_speed_mps, speed_cap_mps),
                target_accel_mps2=0.0,
            ))
        rebuilt, reason = self._rebuild(generated)
        if rebuilt is None:
            return None, -math.inf, reason
        clearance = self._corridor_clearance(rebuilt)
        if clearance < self.config.corridor_clearance_m:
            return None, clearance, "ABORT_CORRIDOR_INFEASIBLE"
        return rebuilt, clearance, reason

    def _rejoin_candidate(self, nominal_samples, start_s, current_d, length_m,
                          current_yaw=None):
        current_yaw = (self.geometry.interpolate(start_s).heading
                       if current_yaw is None else current_yaw)
        current_slope = self._entry_slope(start_s, current_yaw)
        generated = []
        for source in nominal_samples:
            d = self._nominal_connection_d(
                nominal_samples, source, start_s, current_d, current_slope, length_m,
            )
            generated.append(replace(
                self._placed(source, d),
            ))
        rebuilt, reason = self._rebuild(
            generated, speed_ceiling_mps=self.config.racing_speed_cap_mps,
        )
        if rebuilt is None:
            return None, -math.inf, reason
        clearance = self._corridor_clearance(rebuilt)
        if clearance < self.config.corridor_clearance_m:
            return None, clearance, "REJOIN_CORRIDOR_INFEASIBLE"
        return rebuilt, clearance, reason

    def _post_parity_candidate(self, nominal_samples, start_s, current_d):
        generated = []
        for source in nominal_samples:
            delta = self.geometry.forward_delta_s(start_s, source.s_m)
            if delta < self.config.pass_length_m:
                blend = 1.0
            elif delta < self.config.pass_length_m + self.config.merge_length_m:
                blend = 1.0 - _smoothstep(
                    (delta - self.config.pass_length_m) / self.config.merge_length_m,
                )
            else:
                blend = 0.0
            shift = (current_d - source.lateral_offset_m) * blend
            generated.append(replace(
                source,
                x_m=source.x_m - shift * math.sin(source.heading_rad),
                y_m=source.y_m + shift * math.cos(source.heading_rad),
                lateral_offset_m=source.lateral_offset_m + shift,
            ))
        rebuilt, reason = self._rebuild(
            generated, speed_ceiling_mps=self.config.racing_speed_cap_mps,
        )
        if rebuilt is None:
            return None, -math.inf, reason
        clearance = self._corridor_clearance(rebuilt)
        if clearance < self.config.corridor_clearance_m:
            return None, clearance, "CORRIDOR_INFEASIBLE"
        return rebuilt, clearance, reason

    def _temporal_hold_candidate(self, nominal_samples, start_s, current_d, target_speed_mps):
        generated = []
        for source in nominal_samples:
            shift = current_d - source.lateral_offset_m
            d = source.lateral_offset_m + shift
            generated.append(replace(
                source,
                x_m=source.x_m - shift * math.sin(source.heading_rad),
                y_m=source.y_m + shift * math.cos(source.heading_rad),
                lateral_offset_m=d,
                target_speed_mps=min(source.target_speed_mps, target_speed_mps),
                target_accel_mps2=0.0,
            ))
        rebuilt, reason = self._rebuild(generated)
        if rebuilt is None:
            return None, -math.inf, reason
        clearance = self._corridor_clearance(rebuilt)
        if clearance < self.config.corridor_clearance_m:
            return None, clearance, "HOLD_CORRIDOR_INFEASIBLE"
        return rebuilt, clearance, reason

    def _footprint_projection(self, heading, axis_heading):
        difference = heading - axis_heading
        return (self.config.footprint_half_length_m * abs(math.cos(difference)) +
                self.config.footprint_half_width_m * abs(math.sin(difference)))

    def _corridor_clearance(self, samples):
        clearance = math.inf
        for sample in samples:
            track_heading = self.geometry.interpolate(sample.s_m).heading
            half_width = self._footprint_projection(
                sample.heading_rad, track_heading + math.pi / 2.0,
            )
            available = (self.left_widths[sample.sample_index] - sample.lateral_offset_m
                         if sample.lateral_offset_m >= 0.0 else
                         self.right_widths[sample.sample_index] + sample.lateral_offset_m)
            clearance = min(clearance, available - half_width)
        return clearance

    def _steering_feasible(self, samples, speed_floor_mps=0.0, speed_ceiling_mps=math.inf):
        c = self.config
        if any(not math.isfinite(sample.curvature_per_m) for sample in samples):
            return False, "STEERING_ANGLE_INFEASIBLE"
        steering = tuple(math.atan(c.wheelbase_m * sample.curvature_per_m) for sample in samples)
        if any(not math.isfinite(value) or abs(value) > c.steering_angle_max_rad + 1e-12
               for value in steering):
            return False, "STEERING_ANGLE_INFEASIBLE"
        for index, current in enumerate(samples):
            speed = min(max(current.target_speed_mps, speed_floor_mps), speed_ceiling_mps)
            if speed <= 1e-9:
                continue
            dt = current.ds_to_next_m / speed
            if (not math.isfinite(dt) or dt <= 0.0 or
                    abs(steering[(index + 1) % len(samples)] - steering[index]) / dt
                    > c.steering_rate_max_rad_s + 1e-12):
                return False, "STEERING_RATE_INFEASIBLE"
        return True, "VEHICLE_FEASIBLE"

    def _safe(self, samples, blue_s, blue_speed, prediction, qualification_margin_m=None):
        c = self.config
        margin = c.qualification_margin_m if qualification_margin_m is None else qualification_margin_m
        stations = tuple(sample.s_m for sample in samples)
        positions = tuple((sample.x_m, sample.y_m) for sample in samples)
        heading_vectors = tuple((math.cos(sample.heading_rad), math.sin(sample.heading_rad))
                                for sample in samples)
        minimum = math.inf
        for red in prediction.samples:
            station = self.geometry.wrap_s(blue_s + blue_speed * red.dt_s)
            blue_x, blue_y = interpolate_periodic_vector(
                stations, positions, station, self.geometry.length,
            )
            heading_x, heading_y = interpolate_periodic_vector(
                stations, heading_vectors, station, self.geometry.length,
            )
            blue_heading = math.atan2(heading_y, heading_x)
            dx, dy = blue_x - red.x_m, blue_y - red.y_m
            along = abs(dx * math.cos(red.heading_rad) + dy * math.sin(red.heading_rad))
            across = abs(-dx * math.sin(red.heading_rad) + dy * math.cos(red.heading_rad))
            longitudinal = along - red.occupancy_half_length_m - self._footprint_projection(
                blue_heading, red.heading_rad,
            )
            lateral = across - red.occupancy_half_width_m - self._footprint_projection(
                blue_heading, red.heading_rad + math.pi / 2.0,
            )
            separation = max(longitudinal, lateral)
            minimum = min(minimum, separation)
            if separation < c.blue_tracking_uncertainty_m + margin:
                return False, minimum
        return True, minimum

    def _passing_lateral_clearance(self, samples, start_s, prediction):
        c = self.config
        passing = tuple(
            sample for sample in samples
            if c.prepare_length_m <= self.geometry.forward_delta_s(start_s, sample.s_m)
            < c.prepare_length_m + c.pass_length_m
        )
        return min(
            abs(sample.lateral_offset_m - red.d_m)
            - red.occupancy_half_width_m
            - self._footprint_projection(
                sample.heading_rad, red.heading_rad + math.pi / 2.0,
            )
            for sample in passing for red in prediction.samples
        )

    def _blocking_containment(self, nominal_samples, blue_s, prediction, now_s, reason):
        if self.follow_started_s is None:
            self.follow_started_s = now_s
        self.same_strategy_follow_duration_s = max(0.0, now_s - self.follow_started_s)
        if self.same_strategy_follow_duration_s > 5.0:
            reason = "PASS_PLANNING_STALLED:" + reason
        self.speed_constraint_owner = "OPPONENT"
        self.geometry_authority = "OVERTAKE_OVERRIDE"
        first, last = prediction.samples[0], prediction.samples[-1]
        red_speed = math.hypot(last.x_m - first.x_m, last.y_m - first.y_m) / max(
            1e-6, last.dt_s - first.dt_s,
        )
        containment_speed = min(red_speed, self.config.candidate_speed_cap_mps)
        hold = tuple(replace(
            sample, target_speed_mps=min(sample.target_speed_mps, containment_speed),
            target_accel_mps2=0.0,
        ) for sample in nominal_samples)
        self.revision += 1
        identity = json.dumps({
            "prediction": prediction.identity, "revision": self.revision,
            "state": "BLOCKING_HOLD", "start_s": round(blue_s, 9),
        }, sort_keys=True).encode()
        self.plan = Plan(
            "sha256:" + hashlib.sha256(identity).hexdigest(), self.revision,
            "BLOCKING_HOLD", "NONE", True, f"BLOCKING_HOLD:{reason}",
            prediction.identity, prediction.source_revision,
            prediction.source_time_s, prediction.source_time_ns,
            blue_s, blue_s, blue_s, blue_s, hold, -math.inf,
            min(self.left_widths), now_s,
        )
        return self.plan

    def update(self, nominal_samples, blue_x, blue_y, blue_speed, prediction, now_s,
               blue_yaw=None):
        projection = self.geometry.project(blue_x, blue_y)
        blue_s = projection.s
        if blue_yaw is None:
            blue_yaw = interpolate_periodic_vector(
                tuple(sample.s_m for sample in nominal_samples),
                tuple((math.cos(sample.heading_rad), math.sin(sample.heading_rad))
                      for sample in nominal_samples),
                blue_s, self.geometry.length,
            )
            blue_yaw = math.atan2(blue_yaw[1], blue_yaw[0])
        if self._prediction_status(prediction, now_s) == "PREDICTION_QUALIFIED":
            self.last_qualified_prediction = prediction
        previous_containment = None
        eligibility_speed = blue_speed
        if self.plan and self.plan.state == "BLOCKING_HOLD":
            if self.follow_started_s is not None:
                self.same_strategy_follow_duration_s = max(
                    0.0, now_s - self.follow_started_s,
                )
                if (self.same_strategy_follow_duration_s > 5.0 and
                        not self.plan.reason.startswith(
                            "BLOCKING_HOLD:PASS_PLANNING_STALLED:",
                        )):
                    self.plan = replace(
                        self.plan,
                        reason=self.plan.reason.replace(
                            "BLOCKING_HOLD:",
                            "BLOCKING_HOLD:PASS_PLANNING_STALLED:", 1,
                        ),
                    )
            if self._prediction_status(prediction, now_s) == "PREDICTION_QUALIFIED":
                first, last = prediction.samples[0], prediction.samples[-1]
                red_speed = math.hypot(last.x_m - first.x_m, last.y_m - first.y_m) / max(
                    1e-6, last.dt_s - first.dt_s,
                )
                signed_ahead = self.geometry.signed_delta_s(blue_s, first.s_m)
                ahead = self.geometry.forward_delta_s(blue_s, first.s_m)
                nominal_safe = self._safe(
                    nominal_samples, blue_s, blue_speed, prediction,
                )[0]
                rear_closing = signed_ahead < 0.0 and blue_speed < red_speed
                if (not self.config.eligibility_ahead_min_m <= ahead <=
                        self.config.eligibility_ahead_max_m and nominal_safe and
                        not rear_closing):
                    self.plan = replace(
                        self.plan, state="COMPLETE", valid=False, reason="BLOCKING_HOLD_CLEAR",
                    )
                    self.last_qualified_prediction = None
                    self.follow_started_s = None
                    self.same_strategy_follow_duration_s = 0.0
                    self.speed_constraint_owner = "NONE"
                    self.geometry_authority = "NOMINAL"
                    return self.plan
                previous_containment = self.plan
                self.plan = None
                eligibility_speed = max(blue_speed, self.config.candidate_speed_cap_mps)
            else:
                return self.plan
        if self.plan and self.plan.state not in ("COMPLETE", "ABORT"):
            tracking_lost = False
            if self.plan.state in ("PASSING", "MERGE"):
                stations = tuple(sample.s_m for sample in self.plan.samples)
                target_d = interpolate_periodic_scalar(
                    stations,
                    tuple(sample.lateral_offset_m for sample in self.plan.samples),
                    blue_s,
                    self.geometry.length,
                )
                tracking_lost = abs(projection.d - target_d) > self.config.blue_tracking_uncertainty_m
            status = self._prediction_status(prediction, now_s, committed=True)
            was_parity = self.parity_reached
            if status == "PREDICTION_QUALIFIED":
                self._remember_relation(blue_s, prediction)
            separation = self.plan.minimum_predicted_separation_m
            safe = False
            if status == "PREDICTION_QUALIFIED":
                first, last = prediction.samples[0], prediction.samples[-1]
                self.last_opponent_speed_mps = math.hypot(
                    last.x_m - first.x_m, last.y_m - first.y_m,
                ) / max(1e-6, last.dt_s - first.dt_s)
                safe, separation = self._safe(
                    self.plan.samples, blue_s, blue_speed, prediction, 0.0,
                )
            if self.parity_reached:
                self.speed_constraint_owner = "NONE"
                self.geometry_authority = "OVERTAKE_OVERRIDE"

                def currently_safe(samples):
                    return self._safe(
                        samples, blue_s,
                        max(blue_speed, self.config.racing_speed_cap_mps),
                        prediction, 0.0,
                    )[0]

                stations = tuple(sample.s_m for sample in nominal_samples)
                nominal_d = interpolate_periodic_scalar(
                    stations,
                    tuple(sample.lateral_offset_m for sample in nominal_samples),
                    blue_s,
                    self.geometry.length,
                )
                if self.plan.state == "MERGE" and not tracking_lost:
                    if (self.geometry.forward_delta_s(self.plan.start_s, blue_s)
                            >= self.geometry.forward_delta_s(
                                self.plan.start_s, self.plan.merge_end_s,
                            ) and
                            abs(projection.d - nominal_d)
                            <= self.config.blue_tracking_uncertainty_m):
                        self.plan = replace(
                            self.plan, state="COMPLETE", valid=False,
                            reason="MERGE_COMPLETE",
                        )
                        self.last_qualified_prediction = None
                        self.parity_reached = self.safe_ahead = False
                        self.opponent_relation = "BEHIND"
                        self.speed_constraint_owner = "NONE"
                        self.geometry_authority = "NOMINAL"
                        return self.plan
                    if status != "PREDICTION_QUALIFIED" or currently_safe(self.plan.samples):
                        return self.plan

                # A sensor miss can preserve an accepted post-parity payload, but can
                # never authorize a changed pass/rejoin payload.
                if status != "PREDICTION_QUALIFIED":
                    return self.plan

                if self.safe_ahead or tracking_lost:
                    merge = None
                    merge_reason = "REJOIN_INFEASIBLE"
                    merge_length = self.config.merge_length_m
                    for merge_length in (
                            self.config.merge_length_m,
                            1.5 * self.config.merge_length_m,
                            2.0 * self.config.merge_length_m):
                        candidate, corridor, merge_reason = self._rejoin_candidate(
                            nominal_samples, blue_s, projection.d, merge_length, blue_yaw,
                        )
                        if candidate is not None and currently_safe(candidate):
                            merge = candidate
                            break
                    if merge is not None:
                        merge_end = self.geometry.wrap_s(
                            blue_s + merge_length,
                        )
                        self.plan = self._changed_payload(
                            "MERGE", "PASS_CONFIRMED", merge, prediction, now_s,
                            valid=True, start_s=blue_s,
                            commit_end_s=merge_end, pass_end_s=merge_end,
                            merge_end_s=merge_end,
                            minimum_corridor_clearance_m=corridor,
                        )
                        self.degraded_since_s = None
                        return self.plan
                    if was_parity and not tracking_lost and safe:
                        return replace(self.plan, reason="REJOIN_PENDING:" + merge_reason)

                speed_released = any(
                    sample.target_speed_mps > self.config.candidate_speed_cap_mps
                    for sample in self.plan.samples
                )
                if not speed_released or not was_parity or tracking_lost:
                    continuation, corridor, continuation_reason = self._post_parity_candidate(
                        nominal_samples, blue_s, projection.d,
                    )
                else:
                    continuation, corridor, continuation_reason = (
                        self.plan.samples, self.plan.minimum_corridor_clearance_m,
                        "VEHICLE_FEASIBLE",
                    )
                if continuation is not None and currently_safe(continuation):
                    continuation_end = self.geometry.wrap_s(
                        blue_s + self.config.pass_length_m,
                    )
                    if continuation is self.plan.samples:
                        return self.plan
                    self.plan = self._changed_payload(
                        "PASSING", (
                            "REJOIN_PENDING:" + merge_reason
                            if self.safe_ahead else "POST_PARITY_PASS_CORRIDOR"
                        ), continuation,
                        prediction, now_s, valid=True,
                        start_s=blue_s, commit_end_s=blue_s,
                        pass_end_s=continuation_end,
                        merge_end_s=self.geometry.wrap_s(
                            blue_s + self.config.pass_length_m
                            + self.config.merge_length_m,
                        ),
                        minimum_corridor_clearance_m=corridor,
                    )
                    self.degraded_since_s = None
                    return self.plan
                self.speed_constraint_owner = "CURRENT_COLLISION_THREAT"
                self.degraded_since_s = now_s - self.config.prediction_max_age_s
            if tracking_lost:
                safe = False
                self.degraded_since_s = now_s - self.config.prediction_max_age_s
            elif self.speed_constraint_owner == "CURRENT_COLLISION_THREAT":
                safe = False
            elif safe:
                self.degraded_since_s = None
            else:
                if (self.plan.state == "MERGE" and not tracking_lost and
                        self.speed_constraint_owner != "CURRENT_COLLISION_THREAT"):
                    return self.plan
                if self.degraded_since_s is None:
                    self.degraded_since_s = now_s
                safe = now_s - self.degraded_since_s <= self.config.prediction_max_age_s
            if not safe:
                reason = ("TRACKING_ERROR_EXCEEDED" if tracking_lost else
                          status if status != "PREDICTION_QUALIFIED" else
                          "TEMPORAL_CLEARANCE_LOST")
                speed_released = any(
                    sample.target_speed_mps > self.config.candidate_speed_cap_mps
                    for sample in self.plan.samples
                )
                merge_speed = (
                    max(self.config.abort_merge_speed_mps, self.last_opponent_speed_mps)
                    if self.speed_constraint_owner == "CURRENT_COLLISION_THREAT" else
                    self.config.racing_speed_cap_mps if tracking_lost and speed_released else
                    max(self.config.abort_merge_speed_mps, self.last_opponent_speed_mps)
                )
                abort, corridor, abort_reason = self._abort_candidate(
                    nominal_samples, blue_s, projection.d, merge_speed, blue_yaw,
                )
                check_abort_interaction = (
                    reason == "TEMPORAL_CLEARANCE_LOST" or
                    tracking_lost and status == "PREDICTION_QUALIFIED"
                )
                abort_safe = abort is not None and (
                    not check_abort_interaction or
                    self._safe(abort, blue_s, max(blue_speed, merge_speed), prediction)[0]
                )
                if abort_safe:
                    merge_end = self.geometry.wrap_s(blue_s + self.config.merge_length_m)
                    self.plan = replace(
                        self.plan, state="ABORT", valid=True, samples=abort,
                        reason=f"COMMITTED_ABORT:{reason}:MERGING",
                        start_s=blue_s, commit_end_s=merge_end, pass_end_s=merge_end,
                        merge_end_s=merge_end, minimum_predicted_separation_m=separation,
                        minimum_corridor_clearance_m=corridor, committed_at_s=now_s,
                    )
                else:
                    hold_speed = min(
                        self.config.abort_merge_speed_mps, self.last_opponent_speed_mps,
                    ) if reason == "TEMPORAL_CLEARANCE_LOST" else merge_speed
                    if reason in ("TEMPORAL_CLEARANCE_LOST", "TRACKING_ERROR_EXCEEDED"):
                        hold, hold_corridor, hold_reason = self._temporal_hold_candidate(
                            nominal_samples, blue_s, projection.d, hold_speed,
                        )
                    else:
                        hold, hold_corridor, hold_reason = None, corridor, abort_reason
                    if hold is None:
                        hold = tuple(replace(
                            sample, target_speed_mps=min(sample.target_speed_mps, hold_speed),
                            target_accel_mps2=0.0,
                        ) for sample in self.plan.samples)
                    else:
                        corridor, abort_reason = hold_corridor, hold_reason
                    self.plan = replace(
                        self.plan, state="ABORT", valid=True, samples=hold,
                        reason=f"COMMITTED_ABORT:{reason if abort is not None else abort_reason}",
                        minimum_predicted_separation_m=separation, committed_at_s=now_s,
                    )
                self.degraded_since_s = None
                return self.plan
            if (self.plan.state == "COMMITTED_PASS" and
                    self.geometry.forward_delta_s(self.plan.start_s, blue_s) >= self.config.prepare_length_m):
                self.plan = replace(self.plan, state="PASSING", reason="PASS_CORRIDOR_REACHED")
            return self.plan
        if self.plan and self.plan.state == "ABORT":
            status = self._prediction_status(prediction, now_s)
            if status == "PREDICTION_QUALIFIED":
                self._remember_relation(blue_s, prediction)
            if self.parity_reached:
                self.speed_constraint_owner = "NONE"
                self.geometry_authority = "OVERTAKE_OVERRIDE"
                if status != "PREDICTION_QUALIFIED":
                    return self.plan
                candidates = []
                if self.safe_ahead:
                    for length in (
                            self.config.merge_length_m,
                            1.5 * self.config.merge_length_m,
                            2.0 * self.config.merge_length_m):
                        samples, corridor, reason = self._rejoin_candidate(
                            nominal_samples, blue_s, projection.d, length, blue_yaw,
                        )
                        if (samples is not None and self._safe(
                                samples, blue_s,
                                max(blue_speed, self.config.racing_speed_cap_mps),
                                prediction, 0.0,
                        )[0]):
                            candidates.append(("MERGE", samples, corridor, length, reason))
                            break
                if not candidates:
                    samples, corridor, reason = self._post_parity_candidate(
                        nominal_samples, blue_s, projection.d,
                    )
                    if (samples is not None and self._safe(
                            samples, blue_s,
                            max(blue_speed, self.config.racing_speed_cap_mps),
                            prediction, 0.0,
                    )[0]):
                        candidates.append((
                            "PASSING", samples, corridor,
                            self.config.pass_length_m, reason,
                        ))
                if candidates:
                    state, samples, corridor, length, _ = candidates[0]
                    end = self.geometry.wrap_s(blue_s + length)
                    self.plan = self._changed_payload(
                        state,
                        "POST_PARITY_RECOVERY" if state == "PASSING" else "PASS_CONFIRMED",
                        samples, prediction, now_s, valid=True, start_s=blue_s,
                        commit_end_s=(blue_s if state == "PASSING" else end),
                        pass_end_s=end, merge_end_s=end,
                        minimum_corridor_clearance_m=corridor,
                    )
                return self.plan
            merging = self.plan.reason.endswith(":MERGING")
            if not merging:
                if status != "PREDICTION_QUALIFIED":
                    return self.plan
                merge_speed = max(
                    self.config.abort_merge_speed_mps, self.last_opponent_speed_mps,
                )
                abort, corridor, reason = self._abort_candidate(
                    nominal_samples, blue_s, projection.d, merge_speed, blue_yaw,
                )
                if abort is None or not self._safe(
                        abort, blue_s, merge_speed, prediction)[0]:
                    return self.plan
                merge_end = self.geometry.wrap_s(blue_s + self.config.merge_length_m)
                self.plan = replace(
                    self.plan, samples=abort, reason=self.plan.reason + ":MERGING",
                    start_s=blue_s, commit_end_s=merge_end, pass_end_s=merge_end,
                    merge_end_s=merge_end, minimum_corridor_clearance_m=corridor,
                    committed_at_s=now_s,
                )
                return self.plan
            status = self._prediction_status(prediction, now_s)
            if status == "PREDICTION_QUALIFIED":
                safe, separation = self._safe(
                    self.plan.samples, blue_s, blue_speed, prediction,
                )
                if not safe:
                    first, last = prediction.samples[0], prediction.samples[-1]
                    red_speed = math.hypot(last.x_m - first.x_m, last.y_m - first.y_m) / max(
                        1e-6, last.dt_s - first.dt_s,
                    )
                    hold, corridor, _ = self._temporal_hold_candidate(
                        nominal_samples, blue_s, projection.d,
                        min(self.config.abort_merge_speed_mps, red_speed),
                    )
                    if hold is not None:
                        self.plan = replace(
                            self.plan, samples=hold,
                            reason=self.plan.reason.removesuffix(":MERGING"),
                            minimum_predicted_separation_m=separation,
                            minimum_corridor_clearance_m=corridor,
                            committed_at_s=now_s,
                        )
                        return self.plan
            stations = tuple(sample.s_m for sample in nominal_samples)
            nominal_d = interpolate_periodic_scalar(
                stations, tuple(sample.lateral_offset_m for sample in nominal_samples),
                blue_s, self.geometry.length,
            )
            if (self.geometry.forward_delta_s(self.plan.start_s, blue_s) >= self.config.merge_length_m and
                    abs(projection.d - nominal_d) <= self.config.blue_tracking_uncertainty_m):
                if self._prediction_status(prediction, now_s) == "PREDICTION_QUALIFIED":
                    signed_ahead = self.geometry.signed_delta_s(
                        blue_s, prediction.samples[0].s_m,
                    )
                    nominal_safe = self._safe(
                        nominal_samples, blue_s, blue_speed, prediction,
                    )[0]
                    if (0.0 <= signed_ahead < self.config.eligibility_ahead_min_m or
                            not nominal_safe):
                        return self._blocking_containment(
                            nominal_samples, blue_s, prediction, now_s,
                            "ABORT_NOMINAL_UNSAFE",
                        )
                self.plan = replace(
                    self.plan, state="COMPLETE", valid=False,
                    reason="ABORT_CONTINUATION_COMPLETE",
                )
                self.last_qualified_prediction = None
            return self.plan
        reason = self._eligible(blue_s, eligibility_speed, prediction, now_s)
        if reason != "ELIGIBLE":
            if previous_containment is not None:
                self.plan = previous_containment
                return self.plan
            previous = self.last_qualified_prediction
            if (reason in ("PREDICTION_INVALID", "PREDICTION_STALE") and previous and
                    self.config.eligibility_ahead_min_m <= self.geometry.forward_delta_s(
                        blue_s, previous.samples[0].s_m,
                    ) <= self.config.eligibility_ahead_max_m):
                return self._blocking_containment(
                    nominal_samples, blue_s, previous, now_s, reason,
                )
            return Plan("", self.revision, "NOMINAL", "NONE", False, reason, "", 0, 0.0, 0,
                        blue_s, blue_s, blue_s, blue_s, (), math.inf, math.inf, 0.0)
        candidates, rejected = [], []
        self.candidate_results = {}
        for side in ("LEFT", "RIGHT"):
            samples, corridor, reason = self._candidate(
                nominal_samples, blue_s, side, blue_speed, projection.d, blue_yaw,
            )
            if samples is None:
                self.candidate_results[side] = reason
                rejected.append(reason)
                continue
            passing_clearance = self._passing_lateral_clearance(
                samples, blue_s, prediction,
            )
            if passing_clearance < (self.config.blue_tracking_uncertainty_m
                                    + self.config.qualification_margin_m):
                self.candidate_results[side] = "PASSING_LATERAL_CLEARANCE_INFEASIBLE"
                rejected.append("PASSING_LATERAL_CLEARANCE_INFEASIBLE")
                continue
            safe, separation = self._safe(
                samples, blue_s, max(blue_speed, self.config.candidate_speed_cap_mps), prediction,
            )
            if safe:
                self.candidate_results[side] = "SAFE"
                candidates.append((passing_clearance, corridor, separation, side, samples))
            else:
                self.candidate_results[side] = "TEMPORAL_CLEARANCE_INFEASIBLE"
                rejected.append("TEMPORAL_CLEARANCE_INFEASIBLE")
        if not candidates:
            reason = "NO_SAFE_CANDIDATE" if not rejected else "NO_FEASIBLE_CANDIDATE:" + ",".join(sorted(set(rejected)))
            reason += ":EVAL_LEFT={}:EVAL_RIGHT={}".format(
                self.candidate_results.get("LEFT", "NOT_RUN"),
                self.candidate_results.get("RIGHT", "NOT_RUN"),
            )
            return self._blocking_containment(
                nominal_samples, blue_s, prediction, now_s, reason,
            )
        _, corridor, separation, side, samples = max(
            candidates, key=lambda item: (item[0], item[1], item[2], item[3] == "LEFT"),
        )
        self.revision += 1
        ends = [self.geometry.wrap_s(blue_s + value) for value in (
            self.config.prepare_length_m,
            self.geometry.length - self.config.merge_length_m,
            self.geometry.length,
        )]
        identity = json.dumps({"prediction": prediction.identity, "revision": self.revision,
                               "side": side, "start_s": round(blue_s, 9)}, sort_keys=True).encode()
        first, last = prediction.samples[0], prediction.samples[-1]
        self.last_opponent_speed_mps = math.hypot(
            last.x_m - first.x_m, last.y_m - first.y_m,
        ) / max(1e-6, last.dt_s - first.dt_s)
        self.parity_reached = self.safe_ahead = False
        self.opponent_relation = "BEHIND"
        self.speed_constraint_owner = "OPPONENT"
        self.geometry_authority = "OVERTAKE_OVERRIDE"
        self.follow_started_s = None
        reason = "SAFE_CANDIDATE_COMMITTED:EVAL_LEFT={}:EVAL_RIGHT={}".format(
            self.candidate_results["LEFT"], self.candidate_results["RIGHT"],
        )
        self.plan = Plan("sha256:" + hashlib.sha256(identity).hexdigest(), self.revision, "COMMITTED_PASS", side,
                         True, reason, prediction.identity, prediction.source_revision,
                         prediction.source_time_s, prediction.source_time_ns, blue_s, *ends,
                         samples, separation, corridor, now_s)
        return self.plan
