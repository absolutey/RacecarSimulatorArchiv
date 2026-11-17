"""Successor R-E v0 Q0 core.

Committed Spatiotemporal Frenet Planner.

Q0 scope:
- pure deterministic planning core;
- finite topology generation;
- current-state entry continuity;
- physical/corridor trackability gates;
- candidate-speed-based path/time safety against R-D occupancy;
- executable longitudinal speed-advantage gate;
- frozen pass payload;
- one-time fresh-provenance rejoin creation;
- frozen rejoin payload;
- lifecycle-only fail-closed ABORT latch on fresh predicted collision;
- COMPLETE with unchanged identity-bearing payload.

Not in Q0:
- ROS runtime adapter;
- launch wiring;
- product qualification;
- replacement of R-D, S0-R, or R-B;
- Legacy R-E modification.
"""

from bisect import bisect_right
from dataclasses import dataclass, replace
from enum import Enum
import hashlib
import json
import math

from f1tenth_racing_geometry import (
    ClosedTrackGeometry,
    interpolate_periodic_scalar,
    interpolate_periodic_vector,
    wrap_angle,
)


class InternalState(str, Enum):
    TRAIL = "TRAIL"
    PASS_COMMIT = "PASS_COMMIT"
    FROZEN_PASS = "FROZEN_PASS"
    PASS_CONFIRMED = "PASS_CONFIRMED"
    FROZEN_REJOIN = "FROZEN_REJOIN"
    NOMINAL = "NOMINAL"


class WireState(str, Enum):
    COMMITTED_PASS = "COMMITTED_PASS"
    PASSING = "PASSING"
    MERGE = "MERGE"
    ABORT = "ABORT"
    BLOCKING_HOLD = "BLOCKING_HOLD"
    COMPLETE = "COMPLETE"


class Topology(str, Enum):
    LEFT_EARLY = "LEFT_EARLY"
    LEFT_LATE = "LEFT_LATE"
    RIGHT_EARLY = "RIGHT_EARLY"
    RIGHT_LATE = "RIGHT_LATE"
    FOLLOW = "FOLLOW"


@dataclass(frozen=True)
class SuccessorContract:
    name: str = "Committed Spatiotemporal Frenet Planner"
    schema_version: int = 0
    runtime_enabled: bool = False
    prediction_owner: str = "R-D"
    active_reference_owner: str = "S0-R"
    controller_owner: str = "R-B"
    nominal_reference_owner: str = "R-A2"
    candidate_topologies: tuple = (
        Topology.LEFT_EARLY,
        Topology.LEFT_LATE,
        Topology.RIGHT_EARLY,
        Topology.RIGHT_LATE,
        Topology.FOLLOW,
    )


CONTRACT = SuccessorContract()


@dataclass(frozen=True)
class SuccessorConfig:
    prediction_max_age_s: float
    minimum_track_confidence: float
    minimum_confirmed_hits: int
    maximum_source_misses: int
    maximum_covariance_eigenvalue_m2: float
    eligibility_ahead_min_m: float
    eligibility_ahead_max_m: float
    candidate_lateral_offset_m: float
    entry_length_early_m: float
    entry_length_late_m: float
    rejoin_lengths_m: tuple
    corridor_clearance_m: float
    blue_tracking_uncertainty_m: float
    qualification_margin_m: float
    pass_completion_ahead_m: float
    pass_timeout_s: float
    delta_v_pass_mps: float
    footprint_half_length_m: float
    footprint_half_width_m: float
    wheelbase_m: float
    steering_angle_max_rad: float
    steering_rate_max_rad_s: float
    lateral_acceleration_max_mps2: float
    execution_speed_cap_mps: float
    # Zero means the legacy test-only target-speed clock; the ROS adapter must
    # inject both positive physical limits from Foundation before live planning.
    longitudinal_acceleration_max_mps2: float = 0.0
    longitudinal_deceleration_max_mps2: float = 0.0
    # Zero disables optional occupancy-gated FOLLOW approach (legacy behavior).
    safe_approach_max_closing_speed_mps: float = 0.0
    # Moving ABORT avoidance: independently checked moving rejoin, otherwise ABORT.
    emergency_rejoin_enabled: bool = False

    def __post_init__(self):
        nonnegative = (
            self.prediction_max_age_s,
            self.minimum_track_confidence,
            self.maximum_covariance_eigenvalue_m2,
            self.eligibility_ahead_min_m,
            self.eligibility_ahead_max_m,
            self.candidate_lateral_offset_m,
            self.entry_length_early_m,
            self.entry_length_late_m,
            self.corridor_clearance_m,
            self.blue_tracking_uncertainty_m,
            self.qualification_margin_m,
            self.pass_completion_ahead_m,
            self.pass_timeout_s,
            self.delta_v_pass_mps,
            self.footprint_half_length_m,
            self.footprint_half_width_m,
            self.wheelbase_m,
            self.steering_angle_max_rad,
            self.steering_rate_max_rad_s,
            self.lateral_acceleration_max_mps2,
            self.execution_speed_cap_mps,
            self.longitudinal_acceleration_max_mps2,
            self.longitudinal_deceleration_max_mps2,
            self.safe_approach_max_closing_speed_mps,
        )
        if any(not math.isfinite(float(value)) or float(value) < 0.0
               for value in nonnegative):
            raise ValueError("successor configuration must be finite and nonnegative")
        if type(self.emergency_rejoin_enabled) is not bool:
            raise ValueError('emergency rejoin flag must be boolean')
        if self.minimum_confirmed_hits < 0 or self.maximum_source_misses < 0:
            raise ValueError("hit/miss bounds must be nonnegative")
        if self.entry_length_early_m <= 0.0 or self.entry_length_late_m <= 0.0:
            raise ValueError("entry lengths must be positive")
        if self.entry_length_early_m > self.entry_length_late_m:
            raise ValueError("EARLY entry must not be longer than LATE entry")
        if not self.rejoin_lengths_m:
            raise ValueError("at least one rejoin length is required")
        if any(not math.isfinite(float(value)) or float(value) <= 0.0
               for value in self.rejoin_lengths_m):
            raise ValueError("rejoin lengths must be finite and positive")
        if self.pass_timeout_s <= 0.0:
            raise ValueError("pass timeout must be positive")
        if self.footprint_half_length_m <= 0.0 or self.footprint_half_width_m <= 0.0:
            raise ValueError("vehicle footprint must be positive")
        if self.wheelbase_m <= 0.0:
            raise ValueError("wheelbase must be positive")
        if self.steering_angle_max_rad <= 0.0 or self.steering_rate_max_rad_s <= 0.0:
            raise ValueError("steering limits must be positive")
        if self.lateral_acceleration_max_mps2 <= 0.0:
            raise ValueError("lateral acceleration authority must be positive")
        if self.execution_speed_cap_mps <= 0.0:
            raise ValueError("execution speed cap must be positive")
        if ((self.longitudinal_acceleration_max_mps2 == 0.0)
                != (self.longitudinal_deceleration_max_mps2 == 0.0)):
            raise ValueError("longitudinal bounds must both be positive or both disabled")


@dataclass(frozen=True)
class PredictionSample:
    dt_s: float
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
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
    valid: bool
    lifecycle: str
    confidence: float
    hit_count: int
    miss_count: int
    track_id: str
    frame_id: str
    horizon_s: float
    timestep_s: float
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
class Trackability:
    maximum_abs_steering_rad: float
    maximum_steering_rate_rad_s: float
    maximum_lateral_acceleration_mps2: float
    entry_lateral_error_m: float
    entry_heading_error_rad: float

    def normalized_burden(self, config):
        return max(
            self.maximum_abs_steering_rad / config.steering_angle_max_rad,
            self.maximum_steering_rate_rad_s / config.steering_rate_max_rad_s,
            self.maximum_lateral_acceleration_mps2 / config.lateral_acceleration_max_mps2,
        )


@dataclass(frozen=True)
class Candidate:
    topology: Topology
    side: str
    entry_length_m: float
    samples: tuple
    pass_time_s: float
    pass_end_s: float
    minimum_predicted_separation_m: float
    minimum_corridor_clearance_m: float
    minimum_executable_speed_mps: float
    maximum_predicted_opponent_speed_mps: float
    trackability: Trackability


@dataclass(frozen=True)
class Plan:
    revision: int
    state: str
    side: str
    valid: bool
    reason: str
    source_prediction_id: str
    source_revision: int
    source_time_s: float
    source_time_ns: int
    track_id: str
    frame_id: str
    map_image_sha256: str
    start_s: float
    commit_end_s: float
    pass_end_s: float
    merge_end_s: float
    samples: tuple
    minimum_predicted_separation_m: float
    minimum_corridor_clearance_m: float
    committed_at_s: float
    topology: str

    def identity_bearing_payload(self):
        return {
            "source_prediction_id": self.source_prediction_id,
            "source_track_revision": self.source_revision,
            "source_observation_time_ns": self.source_time_ns,
            "plan_revision": self.revision,
            "passing_side": self.side,
            "track_id": self.track_id,
            "frame_id": self.frame_id,
            "map_image_sha256": self.map_image_sha256,
            "start_progress_m": self.start_s,
            "commit_end_progress_m": self.commit_end_s,
            "pass_end_progress_m": self.pass_end_s,
            "merge_end_progress_m": self.merge_end_s,
            "samples": [[
                sample.sample_index,
                sample.s_m,
                sample.x_m,
                sample.y_m,
                sample.heading_rad,
                sample.curvature_per_m,
                sample.lateral_offset_m,
                sample.target_speed_mps,
                sample.target_accel_mps2,
                sample.ds_to_next_m,
            ] for sample in self.samples],
        }

    @property
    def identity_digest(self):
        payload = json.dumps(
            self.identity_bearing_payload(),
            sort_keys=True,
            separators=(",", ":"),
        ).encode()
        return "sha256:" + hashlib.sha256(payload).hexdigest()


@dataclass(frozen=True)
class PlannerOutput:
    internal_state: InternalState
    plan: Plan | None
    action: str
    reason: str


def _largest_covariance(covariance):
    a, b, c, d = covariance
    trace = a + d
    discriminant = max(0.0, (a - d) ** 2 + 4.0 * b * c)
    return 0.5 * (trace + math.sqrt(discriminant))


def _quintic_connection(value, start_value, start_slope, end_value, end_slope, length):
    u = min(1.0, max(0.0, value / length))
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    u5 = u4 * u
    return (
        (1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5) * start_value
        + (u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5) * length * start_slope
        + (10.0 * u3 - 15.0 * u4 + 6.0 * u5) * end_value
        + (-4.0 * u3 + 7.0 * u4 - 3.0 * u5) * length * end_slope
    )


class CommittedSpatiotemporalFrenetPlanner:
    """Deterministic successor R-E Q0 core. It never owns prediction or actuation."""

    PASS_TOPOLOGIES = (
        Topology.LEFT_EARLY,
        Topology.LEFT_LATE,
        Topology.RIGHT_EARLY,
        Topology.RIGHT_LATE,
    )

    def __init__(
        self,
        config,
        centerline,
        left_widths,
        right_widths,
        track_id,
        frame_id,
        map_image_sha256,
    ):
        self.config = config
        self.geometry = ClosedTrackGeometry(centerline)
        if (
            len(left_widths) != self.geometry.segment_count
            or len(right_widths) != self.geometry.segment_count
        ):
            raise ValueError("corridor widths must align with centerline")
        self.left_widths = tuple(float(value) for value in left_widths)
        self.right_widths = tuple(float(value) for value in right_widths)
        self.track_id = str(track_id)
        self.frame_id = str(frame_id)
        self.map_image_sha256 = str(map_image_sha256)
        self.revision = 0
        self.plan = None
        self.internal_state = InternalState.TRAIL
        self.candidate_results = {}
        # Fresh R-D observation heartbeat for frozen maneuver safety.
        self.last_qualified_prediction_at_s = None

    def acknowledge_complete(self):
        if self.plan is None or self.plan.state != WireState.COMPLETE.value:
            raise ValueError("no COMPLETE plan is awaiting release")
        self.plan = None
        self.internal_state = InternalState.NOMINAL

    def acknowledge_abort_recovery(self):
        """Require an S0-R-confirmed parked nominal handoff after ABORT.

        This does NOT revalidate or reuse the old plan. A new opportunity
        must use a new fresh R-D prediction and a new plan identity/revision.
        """
        if self.plan is None or self.plan.state != WireState.ABORT.value:
            raise ValueError('ABORT_NOT_LATCHED_FOR_RECOVERY')
        self.plan = None
        self.internal_state = InternalState.TRAIL
        self.last_qualified_prediction_at_s = None

    def _prediction_status(self, prediction, now_s):
        c = self.config
        if prediction is None or not prediction.valid or not prediction.samples:
            return "PREDICTION_INVALID"
        if prediction.track_id != self.track_id or prediction.frame_id != self.frame_id:
            return "PREDICTION_AUTHORITY_MISMATCH"
        # Red is a safety witness.  NaN and future source timestamps must
        # never pass comparison-based age gates (NaN comparisons are False).
        if (not math.isfinite(now_s) or
                not math.isfinite(prediction.source_time_s) or
                prediction.source_time_ns <= 0 or
                not math.isfinite(prediction.confidence) or
                not 0.0 <= prediction.confidence <= 1.0 or
                prediction.hit_count < 0 or prediction.miss_count < 0 or
                prediction.source_revision < 0):
            return "PREDICTION_SOURCE_NUMERICS_INVALID"
        age = now_s - prediction.source_time_s
        if age < 0.0:
            return "PREDICTION_FROM_FUTURE"
        if age > c.prediction_max_age_s:
            return "PREDICTION_STALE"
        if prediction.lifecycle not in ("TRACKED", "COASTING"):
            return "TRACK_NOT_CONFIRMED"
        if (
            prediction.confidence < c.minimum_track_confidence
            or prediction.hit_count < c.minimum_confirmed_hits
        ):
            return "TRACK_CONFIDENCE_LOW"
        if prediction.miss_count > c.maximum_source_misses:
            return "TRACK_MISSES_EXCESSIVE"
        if _largest_covariance(
            prediction.samples[0].covariance_xy
        ) > c.maximum_covariance_eigenvalue_m2:
            return "PREDICTION_UNCERTAINTY_HIGH"
        if (not math.isfinite(prediction.horizon_s)
                or not math.isfinite(prediction.timestep_s)
                or prediction.horizon_s <= 0.0
                or prediction.timestep_s <= 0.0
                or prediction.timestep_s > prediction.horizon_s + 1e-12):
            return "PREDICTION_TIME_CONTRACT_INVALID"
        if prediction.horizon_s + 1e-12 < c.pass_timeout_s:
            return "PREDICTION_HORIZON_SHORTER_THAN_PASS_TIMEOUT"
        if any(
            sample.dt_s > prediction.horizon_s + 1e-9
            for sample in prediction.samples
        ):
            return "PREDICTION_SAMPLE_EXCEEDS_DECLARED_HORIZON"
        if any(
            len(sample.covariance_xy) != 4
            or not all(math.isfinite(value) for value in sample.covariance_xy)
            or not math.isfinite(sample.occupancy_half_length_m)
            or not math.isfinite(sample.occupancy_half_width_m)
            or sample.occupancy_half_length_m <= 0.0
            or sample.occupancy_half_width_m <= 0.0
            or _largest_covariance(sample.covariance_xy) < -1e-12
            or (sample.covariance_xy[0] < 0.0 or sample.covariance_xy[3] < 0.0)
            or abs(sample.covariance_xy[1] - sample.covariance_xy[2]) > 1e-8
            or sample.covariance_xy[0] * sample.covariance_xy[3]
               - sample.covariance_xy[1] * sample.covariance_xy[2] < -1e-9
            for sample in prediction.samples
        ):
            return "PREDICTION_OCCUPANCY_OR_COVARIANCE_INVALID"
        if any(
            not math.isfinite(sample.dt_s)
            or sample.dt_s <= 0.0
            or not all(math.isfinite(value) for value in (
                sample.x_m,
                sample.y_m,
                sample.vx_mps,
                sample.vy_mps,
                sample.heading_rad,
                sample.s_m,
                sample.d_m,
                sample.occupancy_half_length_m,
                sample.occupancy_half_width_m,
            ))
            for sample in prediction.samples
        ):
            return "PREDICTION_NUMERICS_INVALID"
        if any(
            a.dt_s >= b.dt_s
            for a, b in zip(prediction.samples, prediction.samples[1:])
        ):
            return "PREDICTION_TIME_ORDER_INVALID"
        if prediction.samples[-1].dt_s + 1e-12 < c.pass_timeout_s:
            return "PREDICTION_HORIZON_SHORTER_THAN_PASS_TIMEOUT"
        return "PREDICTION_QUALIFIED"

    def _eligible(self, blue_s, prediction, now_s):
        status = self._prediction_status(prediction, now_s)
        if status != "PREDICTION_QUALIFIED":
            return status
        ahead = self.geometry.forward_delta_s(blue_s, prediction.samples[0].s_m)
        if not (
            self.config.eligibility_ahead_min_m
            <= ahead
            <= self.config.eligibility_ahead_max_m
        ):
            return "OPPONENT_NOT_BLOCKING"
        return "ELIGIBLE"

    def _entry_slope(self, start_s, current_yaw):
        return math.tan(
            wrap_angle(current_yaw - self.geometry.interpolate(start_s).heading)
        )

    def _placed(self, source, lateral_offset_m):
        heading = self.geometry.interpolate(source.s_m).heading
        shift = lateral_offset_m - source.lateral_offset_m
        return replace(
            source,
            x_m=source.x_m - shift * math.sin(heading),
            y_m=source.y_m + shift * math.cos(heading),
            lateral_offset_m=lateral_offset_m,
        )

    def _pass_entry_d(
        self,
        source,
        start_s,
        current_d,
        current_slope,
        target_d,
        length_m,
    ):
        signed = self.geometry.signed_delta_s(start_s, source.s_m)
        if -length_m < signed < 0.0:
            return _quintic_connection(
                signed + length_m,
                target_d,
                0.0,
                current_d,
                current_slope,
                length_m,
            )
        if 0.0 <= signed < length_m:
            return _quintic_connection(
                signed,
                current_d,
                current_slope,
                target_d,
                0.0,
                length_m,
            )
        return target_d

    def _finite_pass_window(self, entry_length_m):
        """Finite PASS prefix; never claim a merge is confirmed from this shape.

        Hold the candidate lateral offset *beyond* the maximum speed-cap
        travel within the existing PASS timeout. A fresh, separately qualified
        rejoin plan replaces this geometry only after measured pass confirmation.
        A track too short for a nonoverlapping window fails closed.
        """
        c = self.config
        pre_length = entry_length_m
        hold_end = (
            entry_length_m + c.execution_speed_cap_mps * c.pass_timeout_s
            + c.pass_completion_ahead_m + 2.0 * c.footprint_half_length_m
            + c.blue_tracking_uncertainty_m
        )
        merge_length = max(c.rejoin_lengths_m)
        return pre_length, hold_end, merge_length

    def _side_target_d(self, nominal_samples, start_s, hold_end, side):
        """Clip requested offset to measured side width in the active window.

        This is an *upper bound*, never evidence of temporal/physical safety.
        The rebuilt footprint-aware corridor test remains mandatory everywhere.
        """
        c = self.config
        widths = self.left_widths if side == "LEFT" else self.right_widths
        indices = [sample.sample_index for sample in nominal_samples
                   if self.geometry.forward_delta_s(start_s, sample.s_m)
                   <= hold_end + 1e-12]
        if not indices:
            return None
        minimum_width = min(widths[index] for index in indices)
        maximum_offset = (minimum_width - c.footprint_half_width_m
                          - c.corridor_clearance_m - c.blue_tracking_uncertainty_m)
        offset = min(c.candidate_lateral_offset_m, maximum_offset)
        # An offset smaller than the state uncertainty plus qualification
        # margin cannot reliably establish a distinct passing corridor.
        if offset <= c.blue_tracking_uncertainty_m + c.qualification_margin_m:
            return None
        return offset if side == "LEFT" else -offset

    def _finite_pass_d(
        self, source, start_s, current_d, current_slope, target_d,
        entry_length_m, hold_end, merge_length, nominal_start_d,
        nominal_merge_d,
    ):
        """Pre-entry conditioning -> entry -> hold -> finite nominal connection.

        Use forward progress relative to the present station to handle tracks
        whose s=0 seam falls within the maneuver. The pre-entry bridge takes
        only the final entry_length metres *before* the start station; unlike
        the old implementation the rest of the lap stays nominal.
        """
        progress = self.geometry.forward_delta_s(start_s, source.s_m)
        lap = self.geometry.length
        if progress >= lap - entry_length_m:
            before = progress - (lap - entry_length_m)
            return source.lateral_offset_m + _quintic_connection(
                before, 0.0, 0.0, current_d - nominal_start_d,
                current_slope, entry_length_m)
        if progress < entry_length_m:
            return _quintic_connection(
                progress, current_d, current_slope, target_d, 0.0,
                entry_length_m)
        if progress < hold_end:
            return target_d
        if progress < hold_end + merge_length:
            return source.lateral_offset_m + _quintic_connection(
                progress - hold_end, target_d - nominal_merge_d, 0.0,
                0.0, 0.0, merge_length)
        return source.lateral_offset_m

    def _nominal_connection_d(
        self,
        nominal_samples,
        source,
        start_s,
        current_d,
        current_slope,
        length_m,
    ):
        stations = tuple(sample.s_m for sample in nominal_samples)
        nominal_start_d = interpolate_periodic_scalar(
            stations,
            tuple(sample.lateral_offset_m for sample in nominal_samples),
            start_s,
            self.geometry.length,
        )
        shift = current_d - nominal_start_d
        signed = self.geometry.signed_delta_s(start_s, source.s_m)
        if -length_m < signed < 0.0:
            relative = _quintic_connection(
                signed + length_m,
                0.0,
                0.0,
                shift,
                current_slope,
                length_m,
            )
            return source.lateral_offset_m + relative
        if 0.0 <= signed < length_m:
            relative = _quintic_connection(
                signed,
                shift,
                current_slope,
                0.0,
                0.0,
                length_m,
            )
            return source.lateral_offset_m + relative
        return source.lateral_offset_m

    def _footprint_projection(self, heading, axis_heading):
        difference = heading - axis_heading
        c = self.config
        return (
            c.footprint_half_length_m * abs(math.cos(difference))
            + c.footprint_half_width_m * abs(math.sin(difference))
        )

    def _corridor_clearance(self, samples):
        clearance = math.inf
        for sample in samples:
            track_heading = self.geometry.interpolate(sample.s_m).heading
            half_width = self._footprint_projection(
                sample.heading_rad,
                track_heading + math.pi / 2.0,
            )
            available = (
                self.left_widths[sample.sample_index] - sample.lateral_offset_m
                if sample.lateral_offset_m >= 0.0
                else self.right_widths[sample.sample_index] + sample.lateral_offset_m
            )
            clearance = min(clearance, available - half_width)
        return clearance

    def _rebuild(self, generated):
        if len(generated) < 3:
            return None, "GEOMETRY_TOO_SHORT"
        points = [(sample.x_m, sample.y_m) for sample in generated]
        headings = tuple(
            math.atan2(
                points[(index + 1) % len(points)][1] - point[1],
                points[(index + 1) % len(points)][0] - point[0],
            )
            for index, point in enumerate(points)
        )
        rebuilt = []
        c = self.config
        for index, sample in enumerate(generated):
            previous = (index - 1) % len(points)
            ds_previous = math.dist(points[previous], points[index])
            ds_next = math.dist(points[index], points[(index + 1) % len(points)])
            ds_curvature = 0.5 * (ds_previous + ds_next)
            if ds_curvature <= 1e-9 or ds_next <= 1e-9:
                return None, "GEOMETRY_DEGENERATE"
            curvature = wrap_angle(
                headings[index] - headings[previous]
            ) / ds_curvature
            if not math.isfinite(curvature):
                return None, "CURVATURE_NONFINITE"
            curvature_speed = (
                math.sqrt(c.lateral_acceleration_max_mps2 / abs(curvature))
                if abs(curvature) > 1e-12
                else math.inf
            )
            speed = min(
                sample.target_speed_mps,
                c.execution_speed_cap_mps,
                curvature_speed,
            )
            if not math.isfinite(speed) or speed <= 1e-9:
                return None, "EXECUTABLE_SPEED_NONPOSITIVE"
            capped = speed + 1e-12 < sample.target_speed_mps
            rebuilt.append(replace(
                sample,
                heading_rad=headings[index],
                curvature_per_m=curvature,
                target_speed_mps=speed,
                target_accel_mps2=0.0 if capped else sample.target_accel_mps2,
                ds_to_next_m=ds_next,
            ))
        return tuple(rebuilt), "GEOMETRY_REBUILT"

    def _entry_continuity(self, samples, start_s, current_d, current_yaw):
        stations = tuple(sample.s_m for sample in samples)
        candidate_d = interpolate_periodic_scalar(
            stations,
            tuple(sample.lateral_offset_m for sample in samples),
            start_s,
            self.geometry.length,
        )
        vector = interpolate_periodic_vector(
            stations,
            tuple(
                (math.cos(sample.heading_rad), math.sin(sample.heading_rad))
                for sample in samples
            ),
            start_s,
            self.geometry.length,
        )
        candidate_heading = math.atan2(vector[1], vector[0])
        lateral_error = abs(candidate_d - current_d)
        heading_error = abs(wrap_angle(candidate_heading - current_yaw))
        return lateral_error, heading_error

    def _execution_edge_indices(
        self,
        samples,
        start_s,
        horizon_s,
    ):
        # Reuse the candidate target-speed clock used by state_at_time().
        if horizon_s is None:
            return tuple(range(len(samples))), "FULL_LOOP"
        if not math.isfinite(horizon_s) or horizon_s <= 0.0:
            return None, "TRACKABILITY_HORIZON_INVALID"

        index, fraction = self._edge_location(samples, start_s)
        remaining = horizon_s
        indices = []
        first = True

        for _ in range(len(samples)):
            sample = samples[index]
            speed = sample.target_speed_mps
            if speed <= 1e-9 or not math.isfinite(speed):
                return None, "EXECUTABLE_SPEED_NONPOSITIVE"

            start_fraction = fraction if first else 0.0
            available_fraction = 1.0 - start_fraction
            edge_distance = sample.ds_to_next_m * available_fraction
            edge_time = edge_distance / speed

            if not math.isfinite(edge_time) or edge_time <= 1e-9:
                return None, "STEERING_RATE_INFEASIBLE"

            indices.append(index)
            if remaining <= edge_time + 1e-12:
                break

            remaining -= edge_time
            index = (index + 1) % len(samples)
            fraction = 0.0
            first = False

        if not indices:
            return None, "TRACKABILITY_SCOPE_EMPTY"
        return tuple(indices), "BOUNDED_PREFIX"

    def _trackability(
        self,
        samples,
        start_s,
        current_d,
        current_yaw,
        horizon_s=None,
    ):
        c = self.config

        active_indices, reason = self._execution_edge_indices(
            samples,
            start_s,
            horizon_s,
        )
        if active_indices is None:
            return None, reason

        steering = tuple(
            math.atan(c.wheelbase_m * sample.curvature_per_m)
            for sample in samples
        )
        if any(not math.isfinite(value) for value in steering):
            return None, "STEERING_ANGLE_INFEASIBLE"

        steering_indices = set(active_indices)
        steering_indices.update(
            (index + 1) % len(samples)
            for index in active_indices
        )
        maximum_steering = max(
            abs(steering[index])
            for index in steering_indices
        )
        if maximum_steering > c.steering_angle_max_rad + 1e-12:
            return None, "STEERING_ANGLE_INFEASIBLE"

        rates = []
        lateral_accelerations = []
        for index in active_indices:
            sample = samples[index]
            speed = sample.target_speed_mps
            if speed <= 1e-9:
                return None, "EXECUTABLE_SPEED_NONPOSITIVE"

            dt = sample.ds_to_next_m / speed
            if not math.isfinite(dt) or dt <= 1e-9:
                return None, "STEERING_RATE_INFEASIBLE"

            rates.append(
                abs(wrap_angle(
                    steering[(index + 1) % len(samples)]
                    - steering[index]
                )) / dt
            )
            lateral_accelerations.append(
                speed * speed * abs(sample.curvature_per_m)
            )

        maximum_rate = max(rates)
        if maximum_rate > c.steering_rate_max_rad_s + 1e-12:
            return None, "STEERING_RATE_INFEASIBLE"

        maximum_lateral_acceleration = max(lateral_accelerations)
        if (
            maximum_lateral_acceleration
            > c.lateral_acceleration_max_mps2 + 1e-9
        ):
            return None, "LATERAL_ACCELERATION_INFEASIBLE"

        entry_lateral_error, entry_heading_error = self._entry_continuity(
            samples,
            start_s,
            current_d,
            current_yaw,
        )
        if entry_lateral_error > c.blue_tracking_uncertainty_m + 1e-12:
            return None, "ENTRY_LATERAL_DISCONTINUITY"
        if not math.isfinite(entry_heading_error):
            return None, "ENTRY_HEADING_NONFINITE"

        # Do not invent an independent heading-error threshold here.
        # The candidate starts from the current yaw tangent and is already
        # gated by curvature, steering angle/rate, lateral acceleration,
        # and lateral-entry continuity.
        return Trackability(
            maximum_abs_steering_rad=maximum_steering,
            maximum_steering_rate_rad_s=maximum_rate,
            maximum_lateral_acceleration_mps2=maximum_lateral_acceleration,
            entry_lateral_error_m=entry_lateral_error,
            entry_heading_error_rad=entry_heading_error,
        ), "TRACKABLE"

    def _candidate_geometry(
        self,
        nominal_samples,
        start_s,
        current_d,
        current_yaw,
        side,
        entry_length_m,
    ):
        if side not in ("LEFT", "RIGHT"):
            return None, -math.inf, None, "PASS_SIDE_INVALID"
        if not nominal_samples:
            return None, -math.inf, None, "NOMINAL_REFERENCE_EMPTY"
        pre_length, hold_end, merge_length = self._finite_pass_window(entry_length_m)
        # A closed reference may contain one maneuver only. Reserve two discrete
        # segments to keep the seam and the final nominal reference distinct.
        largest_ds = max(sample.ds_to_next_m for sample in nominal_samples)
        if (pre_length + hold_end + merge_length + 2.0 * largest_ds
                >= self.geometry.length):
            return None, -math.inf, None, "FINITE_PASS_WINDOW_TOO_LONG"
        target_d = self._side_target_d(nominal_samples, start_s, hold_end, side)
        if target_d is None:
            return None, -math.inf, None, "PASS_LATERAL_SPACE_INSUFFICIENT"

        stations = tuple(sample.s_m for sample in nominal_samples)
        offsets = tuple(sample.lateral_offset_m for sample in nominal_samples)
        nominal_start_d = interpolate_periodic_scalar(
            stations, offsets, start_s, self.geometry.length)
        nominal_merge_d = interpolate_periodic_scalar(
            stations, offsets, self.geometry.wrap_s(start_s + hold_end),
            self.geometry.length)
        current_slope = self._entry_slope(start_s, current_yaw)
        generated = []
        for source in nominal_samples:
            d = self._finite_pass_d(
                source, start_s, current_d, current_slope, target_d,
                entry_length_m, hold_end, merge_length, nominal_start_d,
                nominal_merge_d)
            generated.append(self._placed(source, d))

        rebuilt, reason = self._rebuild(generated)
        if rebuilt is None:
            return None, -math.inf, None, reason
        # Full-path gate is deliberate: nominal-only portions are not exempt
        # from physical corridor safety. Do not mistake their inclusion for a
        # requirement to drive the passing offset around the entire track.
        clearance = self._corridor_clearance(rebuilt)
        if clearance < self.config.corridor_clearance_m:
            return None, clearance, None, "CORRIDOR_INFEASIBLE"
        trackability, reason = self._trackability(
            rebuilt, start_s, current_d, current_yaw,
            horizon_s=self.config.pass_timeout_s)
        if trackability is None:
            return None, clearance, None, reason
        return rebuilt, clearance, trackability, "TRACKABLE"

    def _rejoin_geometry(
        self,
        nominal_samples,
        start_s,
        current_d,
        current_yaw,
        length_m,
    ):
        current_slope = self._entry_slope(start_s, current_yaw)
        generated = []
        for source in nominal_samples:
            d = self._nominal_connection_d(
                nominal_samples,
                source,
                start_s,
                current_d,
                current_slope,
                length_m,
            )
            generated.append(self._placed(source, d))
        rebuilt, reason = self._rebuild(generated)
        if rebuilt is None:
            return None, -math.inf, None, reason
        clearance = self._corridor_clearance(rebuilt)
        if clearance < self.config.corridor_clearance_m:
            return None, clearance, None, "REJOIN_CORRIDOR_INFEASIBLE"
        trackability, reason = self._trackability(
            rebuilt,
            start_s,
            current_d,
            current_yaw,
        )
        if trackability is None:
            return None, clearance, None, reason
        return rebuilt, clearance, trackability, "TRACKABLE"

    def _edge_location(self, samples, station):
        stations = tuple(sample.s_m for sample in samples)
        station = self.geometry.wrap_s(station)
        index = bisect_right(stations, station) - 1
        if index < 0:
            index = len(samples) - 1
        following = (index + 1) % len(samples)
        station_delta = self.geometry.forward_delta_s(
            samples[index].s_m,
            samples[following].s_m,
        )
        offset = self.geometry.forward_delta_s(
            samples[index].s_m,
            station,
        )
        fraction = (
            0.0
            if station_delta <= 1e-12
            else min(1.0, max(0.0, offset / station_delta))
        )
        return index, fraction

    @staticmethod
    def _edge_state(samples, index, fraction, speed):
        current = samples[index]
        following = samples[(index + 1) % len(samples)]
        fraction = min(1.0, max(0.0, fraction))
        return {
            "s_m": None,  # overwritten with station-domain interpolation below
            "x_m": current.x_m + (following.x_m - current.x_m) * fraction,
            "y_m": current.y_m + (following.y_m - current.y_m) * fraction,
            "heading_rad": wrap_angle(current.heading_rad + wrap_angle(
                following.heading_rad - current.heading_rad) * fraction),
            "d_m": current.lateral_offset_m + (
                following.lateral_offset_m - current.lateral_offset_m) * fraction,
            "speed_mps": speed,
        }

    def _reachable_state_at_time(self, samples, start_s, dt_s, initial_speed_mps):
        """Track target speed without instantaneous acceleration or braking.

        The Foundation acceleration/deceleration bounds constrain the *optimistic*
        longitudinal candidate clock. It is not a guarantee of R-B tracking;
        live acceptance still requires command, truth and reference verification.
        """
        c = self.config
        amax = c.longitudinal_acceleration_max_mps2
        dmax = c.longitudinal_deceleration_max_mps2
        if (amax <= 0.0 or dmax <= 0.0 or not math.isfinite(initial_speed_mps)
                or initial_speed_mps < 0.0):
            return None
        index, fraction = self._edge_location(samples, start_s)
        speed = initial_speed_mps
        remaining = dt_s
        traversed = 0
        while remaining > 1e-12:
            if traversed > len(samples) * 3:
                return None
            current = samples[index]
            cap = min(current.target_speed_mps, c.execution_speed_cap_mps)
            if not math.isfinite(cap) or cap <= 0.0:
                return None
            h = min(remaining, 0.02)
            if speed < cap:
                next_speed = min(cap, speed + amax * h)
            else:
                next_speed = max(cap, speed - dmax * h)
            accel = (next_speed - speed) / h
            distance = max(0.0, 0.5 * (speed + next_speed) * h)
            edge_left = current.ds_to_next_m * (1.0 - fraction)
            if not math.isfinite(edge_left) or edge_left <= 0.0:
                return None
            if distance < edge_left - 1e-10:
                fraction += distance / current.ds_to_next_m
                speed = next_speed
                remaining -= h
                continue
            # Resolve the boundary at the true time, then apply the next edge's
            # speed target only after crossing the sample boundary.
            if abs(accel) < 1e-12:
                used = edge_left / speed if speed > 1e-12 else math.inf
            else:
                disc = max(0.0, speed * speed + 2.0 * accel * edge_left)
                used = (math.sqrt(disc) - speed) / accel
            if not math.isfinite(used) or used < -1e-9 or used > h + 1e-8:
                return None
            used = min(h, max(0.0, used))
            speed = max(0.0, speed + accel * used)
            remaining -= used
            index = (index + 1) % len(samples)
            fraction = 0.0
            traversed += 1
        state = self._edge_state(samples, index, fraction, speed)
        following = samples[(index + 1) % len(samples)]
        state["s_m"] = self.geometry.wrap_s(samples[index].s_m +
            self.geometry.forward_delta_s(samples[index].s_m, following.s_m) * fraction)
        return state

    def state_at_time(self, samples, start_s, dt_s, initial_speed_mps=None):
        """Live path clock uses measured ego speed and Foundation acceleration.

        Omission of initial_speed_mps preserves the archived Q0 target-speed
        utility only. The runtime update() always supplies measured speed.
        """
        if dt_s < 0.0 or not math.isfinite(dt_s):
            raise ValueError("time must be finite and nonnegative")
        if initial_speed_mps is not None:
            return self._reachable_state_at_time(
                samples, start_s, dt_s, initial_speed_mps)
        index, fraction = self._edge_location(samples, start_s)
        remaining = dt_s
        edge_count = 0
        first = True
        while edge_count <= len(samples) * 2:
            current = samples[index]
            following = samples[(index + 1) % len(samples)]
            start_fraction = fraction if first else 0.0
            available_fraction = 1.0 - start_fraction
            edge_distance = current.ds_to_next_m * available_fraction
            speed = current.target_speed_mps
            if speed <= 1e-9 or not math.isfinite(speed):
                return None
            edge_time = edge_distance / speed
            if remaining <= edge_time + 1e-12:
                local = 0.0 if edge_time <= 1e-12 else remaining / edge_time
                full_fraction = start_fraction + available_fraction * local
                station_delta = self.geometry.forward_delta_s(
                    current.s_m,
                    following.s_m,
                )
                station = self.geometry.wrap_s(
                    current.s_m + station_delta * full_fraction
                )
                state = self._edge_state(samples, index, full_fraction, speed)
                state["s_m"] = station
                return state
            remaining -= edge_time
            index = (index + 1) % len(samples)
            fraction = 0.0
            first = False
            edge_count += 1
        return None

    def _separation(self, blue, red):
        dx = blue["x_m"] - red.x_m
        dy = blue["y_m"] - red.y_m
        along = abs(
            dx * math.cos(red.heading_rad)
            + dy * math.sin(red.heading_rad)
        )
        across = abs(
            -dx * math.sin(red.heading_rad)
            + dy * math.cos(red.heading_rad)
        )
        longitudinal = (
            along
            - red.occupancy_half_length_m
            - self._footprint_projection(
                blue["heading_rad"],
                red.heading_rad,
            )
        )
        lateral = (
            across
            - red.occupancy_half_width_m
            - self._footprint_projection(
                blue["heading_rad"],
                red.heading_rad + math.pi / 2.0,
            )
        )
        return max(longitudinal, lateral)

    def _temporal_safety(
        self,
        samples,
        start_s,
        prediction,
        qualification_margin_m=None,
        initial_speed_mps=None,
    ):
        margin = (
            self.config.qualification_margin_m
            if qualification_margin_m is None
            else qualification_margin_m
        )
        minimum = math.inf
        for red in prediction.samples:
            blue = self.state_at_time(
                samples, start_s, red.dt_s, initial_speed_mps=initial_speed_mps)
            if blue is None:
                return False, minimum
            separation = self._separation(blue, red)
            minimum = min(minimum, separation)
            if separation < self.config.blue_tracking_uncertainty_m + margin:
                return False, minimum
        return True, minimum

    def _pass_completion(
        self, samples, start_s, prediction, initial_speed_mps=None):
        states = []
        red_speeds = []
        for red in prediction.samples:
            if red.dt_s > self.config.pass_timeout_s + 1e-12:
                break
            blue = self.state_at_time(
                samples, start_s, red.dt_s, initial_speed_mps=initial_speed_mps)
            if blue is None:
                return None
            states.append(blue)
            red_speeds.append(math.hypot(red.vx_mps, red.vy_mps))
            lead = self.geometry.signed_delta_s(red.s_m, blue["s_m"])
            if lead >= self.config.pass_completion_ahead_m:
                # Pass speed advantage must exist near crossing, not at the
                # slower entry instant before the vehicle accelerates.
                minimum_ego_speed = min(
                    state["speed_mps"] for state in states[-min(3, len(states)):])
                maximum_red_speed = max(red_speeds[-min(3, len(red_speeds)):])
                if (
                    minimum_ego_speed + 1e-12
                    < maximum_red_speed + self.config.delta_v_pass_mps
                ):
                    return {
                        "status": "EXECUTABLE_SPEED_ADVANTAGE_INSUFFICIENT",
                        "pass_time_s": red.dt_s,
                        "pass_end_s": blue["s_m"],
                        "minimum_ego_speed_mps": minimum_ego_speed,
                        "maximum_red_speed_mps": maximum_red_speed,
                    }
                return {
                    "status": "PASS_PREDICTED",
                    "pass_time_s": red.dt_s,
                    "pass_end_s": blue["s_m"],
                    "minimum_ego_speed_mps": minimum_ego_speed,
                    "maximum_red_speed_mps": maximum_red_speed,
                }
        return None

    def _rejoin_exists(
        self,
        nominal_samples,
        pass_state,
    ):
        for length_m in self.config.rejoin_lengths_m:
            samples, _, _, _ = self._rejoin_geometry(
                nominal_samples,
                pass_state["s_m"],
                pass_state["d_m"],
                pass_state["heading_rad"],
                length_m,
            )
            if samples is not None:
                return True
        return False

    def _upper_bound_can_complete(self, blue_s, blue_speed, prediction):
        """Reject impossible passes before expensive geometry search.

        This is a deliberately optimistic upper bound (instant access to the
        global qualified speed cap, ignoring local curves and overlap). Failure
        is conclusive; passing this screen NEVER establishes feasibility.
        """
        cap = self.config.execution_speed_cap_mps
        acceleration = self.config.longitudinal_acceleration_max_mps2
        if acceleration <= 0.0:
            return False
        for red in prediction.samples:
            t = red.dt_s
            if t > self.config.pass_timeout_s + 1e-12:
                break
            if blue_speed < cap:
                ramp = min(t, (cap - blue_speed) / acceleration)
                maximum_distance = (blue_speed * ramp +
                    0.5 * acceleration * ramp * ramp + cap * (t-ramp))
                maximum_speed = min(cap, blue_speed + acceleration*t)
            else:
                maximum_distance = blue_speed*t
                maximum_speed = blue_speed
            needed_distance = (self.geometry.forward_delta_s(blue_s, red.s_m)
                + self.config.pass_completion_ahead_m)
            red_speed = math.hypot(red.vx_mps, red.vy_mps)
            if (maximum_distance + 1e-12 >= needed_distance and
                    maximum_speed + 1e-12 >= red_speed + self.config.delta_v_pass_mps):
                return True
        return False

    def _entry_length_candidates(self, topology, blue_s, prediction):
        seed = (
            self.config.entry_length_early_m
            if topology.value.endswith("EARLY")
            else self.config.entry_length_late_m
        )
        if not topology.value.endswith("EARLY"):
            return (seed,)
        if prediction is None or not prediction.samples:
            return (seed,)

        step = self.config.blue_tracking_uncertainty_m
        if not math.isfinite(step) or step <= 1e-9:
            return (seed,)

        red0 = prediction.samples[0]
        ahead = self.geometry.forward_delta_s(blue_s, red0.s_m)
        envelope = (
            red0.occupancy_half_length_m
            + self.config.footprint_half_length_m
            + self.config.blue_tracking_uncertainty_m
            + self.config.qualification_margin_m
        )
        lower = ahead - envelope
        if not math.isfinite(lower):
            return (seed,)
        lower = min(seed, max(step, lower))

        values = [seed]
        length = seed - step
        while length > lower + 1e-12:
            values.append(length)
            length -= step
        if values[-1] > lower + 1e-12:
            values.append(lower)
        return tuple(values)

    def _evaluate_topology_at_entry_length(
        self,
        nominal_samples,
        blue_s,
        current_d,
        current_yaw,
        prediction,
        topology,
        entry_length,
        initial_speed_mps=None,
    ):
        side = "LEFT" if topology.value.startswith("LEFT") else "RIGHT"
        samples, corridor, trackability, reason = self._candidate_geometry(
            nominal_samples,
            blue_s,
            current_d,
            current_yaw,
            side,
            entry_length,
        )
        if samples is None:
            return None, reason

        safe, separation = self._temporal_safety(
            samples,
            blue_s,
            prediction,
            initial_speed_mps=initial_speed_mps,
        )
        if not safe:
            return None, "TEMPORAL_CLEARANCE_INFEASIBLE"

        completion = self._pass_completion(
            samples,
            blue_s,
            prediction,
            initial_speed_mps=initial_speed_mps,
        )
        if completion is None:
            return None, "PASS_NOT_PREDICTED_WITHIN_TIMEOUT"
        if completion["status"] != "PASS_PREDICTED":
            return None, completion["status"]

        pass_state = self.state_at_time(
            samples,
            blue_s,
            completion["pass_time_s"],
            initial_speed_mps=initial_speed_mps,
        )
        if pass_state is None:
            return None, "PASS_STATE_UNAVAILABLE"
        if not self._rejoin_exists(nominal_samples, pass_state):
            return None, "REJOIN_TRACKABILITY_INFEASIBLE"

        return Candidate(
            topology=topology,
            side=side,
            entry_length_m=entry_length,
            samples=samples,
            pass_time_s=completion["pass_time_s"],
            pass_end_s=completion["pass_end_s"],
            minimum_predicted_separation_m=separation,
            minimum_corridor_clearance_m=corridor,
            minimum_executable_speed_mps=completion["minimum_ego_speed_mps"],
            maximum_predicted_opponent_speed_mps=completion["maximum_red_speed_mps"],
            trackability=trackability,
        ), "FEASIBLE"

    def _evaluate_topology(
        self,
        nominal_samples,
        blue_s,
        current_d,
        current_yaw,
        prediction,
        topology,
        initial_speed_mps=None,
    ):
        lengths = self._entry_length_candidates(topology, blue_s, prediction)
        speed_kwargs = ({"initial_speed_mps": initial_speed_mps}
            if initial_speed_mps is not None else {})

        candidate, seed_reason = self._evaluate_topology_at_entry_length(
            nominal_samples,
            blue_s,
            current_d,
            current_yaw,
            prediction,
            topology,
            lengths[0],
            **speed_kwargs,
        )
        if candidate is not None:
            return candidate, "FEASIBLE"

        if (
            not topology.value.endswith("EARLY")
            or seed_reason not in (
                "TEMPORAL_CLEARANCE_INFEASIBLE",
                "PASS_NOT_PREDICTED_WITHIN_TIMEOUT",
            )
        ):
            return None, seed_reason

        for entry_length in lengths[1:]:
            candidate, _ = self._evaluate_topology_at_entry_length(
                nominal_samples,
                blue_s,
                current_d,
                current_yaw,
                prediction,
                topology,
                entry_length,
                **speed_kwargs,
            )
            if candidate is not None:
                return candidate, "FEASIBLE"

        return None, seed_reason

    def evaluate_candidates(
        self,
        nominal_samples,
        blue_s,
        current_d,
        current_yaw,
        prediction,
        initial_speed_mps=None,
    ):
        speed_kwargs = ({"initial_speed_mps": initial_speed_mps}
            if initial_speed_mps is not None else {})
        feasible = []
        results = {}
        for rank, topology in enumerate(self.PASS_TOPOLOGIES):
            candidate, reason = self._evaluate_topology(
                nominal_samples,
                blue_s,
                current_d,
                current_yaw,
                prediction,
                topology,
                **speed_kwargs,
            )
            results[topology.value] = reason
            if candidate is not None:
                feasible.append((candidate, rank))
        self.candidate_results = dict(results)
        if not feasible:
            return None, results
        selected, _ = min(
            feasible,
            key=lambda item: (
                item[0].pass_time_s,
                item[0].trackability.normalized_burden(self.config),
                item[1],
            ),
        )
        return selected, results

    def _new_plan(
        self,
        state,
        side,
        reason,
        prediction,
        start_s,
        commit_end_s,
        pass_end_s,
        merge_end_s,
        samples,
        separation,
        corridor,
        now_s,
        topology,
    ):
        self.revision += 1
        return Plan(
            revision=self.revision,
            state=state,
            side=side,
            valid=True,
            reason=reason,
            source_prediction_id=prediction.identity,
            source_revision=prediction.source_revision,
            source_time_s=prediction.source_time_s,
            source_time_ns=prediction.source_time_ns,
            track_id=self.track_id,
            frame_id=self.frame_id,
            map_image_sha256=self.map_image_sha256,
            start_s=start_s,
            commit_end_s=commit_end_s,
            pass_end_s=pass_end_s,
            merge_end_s=merge_end_s,
            samples=samples,
            minimum_predicted_separation_m=separation,
            minimum_corridor_clearance_m=corridor,
            committed_at_s=now_s,
            topology=topology,
        )

    def commit_candidate(self, candidate, prediction, blue_s, now_s):
        """Atomic PASS_COMMIT. Intended for update() and deterministic Q0 tests."""
        self.plan = self._new_plan(
            state=WireState.COMMITTED_PASS.value,
            side=candidate.side,
            reason="SUCCESSOR_PASS_COMMIT:" + candidate.topology.value,
            prediction=prediction,
            start_s=blue_s,
            commit_end_s=self.geometry.wrap_s(
                blue_s + candidate.entry_length_m
            ),
            pass_end_s=candidate.pass_end_s,
            merge_end_s=self.geometry.wrap_s(
                blue_s + sum(self._finite_pass_window(candidate.entry_length_m)[1:])
            ),  # provisional geometric end; MERGE requires a new qualified plan
            samples=candidate.samples,
            separation=candidate.minimum_predicted_separation_m,
            corridor=candidate.minimum_corridor_clearance_m,
            now_s=now_s,
            topology=candidate.topology.value,
        )
        self.internal_state = InternalState.PASS_COMMIT
        self.last_qualified_prediction_at_s = now_s
        return PlannerOutput(
            internal_state=self.internal_state,
            plan=self.plan,
            action="PUBLISH_OVERRIDE",
            reason=self.plan.reason,
        )

    def _safe_approach_target(self, nominal_samples, blue_s, blue_speed, prediction,
                              red_speed):
        """Bound a FOLLOW speed increase using R-D *declared* future occupancy.

        This is a receding-horizon NOMINAL approach, never an implicit PASS.
        No extrapolation or claimed feasibility. An unverified baseline leaves
        existing conservative FOLLOW speed unchanged, never increases it.
        """
        c = self.config
        if (c.safe_approach_max_closing_speed_mps <= 0.0
                or c.longitudinal_acceleration_max_mps2 <= 0.0
                or c.longitudinal_deceleration_max_mps2 <= 0.0
                or blue_speed is None or not math.isfinite(blue_speed)
                or blue_speed < 0.0 or not nominal_samples
                or self._prediction_status(prediction, prediction.source_time_s)
                   != "PREDICTION_QUALIFIED"):
            return red_speed, "APPROACH_DISABLED_OR_UNQUALIFIED"
        ceiling = min(c.execution_speed_cap_mps,
                      red_speed + c.safe_approach_max_closing_speed_mps)
        step = c.delta_v_pass_mps
        if step <= 0.0 or ceiling <= red_speed + 1e-9:
            return red_speed, "APPROACH_NO_SPEED_HEADROOM"
        n = max(1, math.ceil((ceiling - red_speed) / step))
        speeds = [ceiling] + [red_speed + (ceiling-red_speed)*i/n
                               for i in range(n-1, -1, -1)]
        for speed in speeds:
            trial = tuple(replace(sample,
                        target_speed_mps=min(sample.target_speed_mps, speed),
                        target_accel_mps2=0.0) for sample in nominal_samples)
            permitted = True
            for red in prediction.samples:
                blue = self.state_at_time(trial, blue_s, red.dt_s,
                                          initial_speed_mps=blue_speed)
                if blue is None:
                    permitted = False
                    break
                ahead = self.geometry.signed_delta_s(blue["s_m"], red.s_m)
                # Occupancy includes R-D covariance, do not subtract it.
                # One full R-D timestep of relative motion guards sampled-time
                # gaps; it is an additional reserve, not a continuous proof.
                red_v = math.hypot(red.vx_mps, red.vy_mps)
                reserve = (red.occupancy_half_length_m
                           + c.footprint_half_length_m
                           + c.blue_tracking_uncertainty_m
                           + c.qualification_margin_m
                           + max(0.0, max(blue_speed, speed) - red_v)
                             * prediction.timestep_s)
                if not (math.isfinite(ahead) and ahead > reserve + 1e-9):
                    permitted = False
                    break
            if permitted:
                return speed, "APPROACH_OCCUPANCY_GATED"
        return red_speed, "APPROACH_REJECTED_BY_R_D_OCCUPANCY"

    def _follow(self, nominal_samples, blue_s, prediction, now_s, reason, blue_speed=None):
        red_speed = min(
            math.hypot(sample.vx_mps, sample.vy_mps)
            for sample in prediction.samples
        )
        follow_speed = min(red_speed, self.config.execution_speed_cap_mps)
        target, approach_reason = self._safe_approach_target(
            nominal_samples, blue_s, blue_speed, prediction, follow_speed)
        if target <= 1e-9:
            return PlannerOutput(
                InternalState.TRAIL,
                None,
                "KEEP_NOMINAL",
                "FOLLOW_SPEED_NONPOSITIVE",
            )
        samples = tuple(
            replace(
                sample,
                target_speed_mps=min(sample.target_speed_mps, target),
                target_accel_mps2=0.0,
            )
            for sample in nominal_samples
        )
        corridor = self._corridor_clearance(samples)
        self.plan = self._new_plan(
            state=WireState.BLOCKING_HOLD.value,
            side="NONE",
            reason=("SUCCESSOR_FOLLOW:" + reason
                    + (":" + approach_reason if target > follow_speed + 1e-9 else "")),
            prediction=prediction,
            start_s=blue_s,
            commit_end_s=blue_s,
            pass_end_s=blue_s,
            merge_end_s=blue_s,
            samples=samples,
            separation=-math.inf,
            corridor=corridor,
            now_s=now_s,
            topology=Topology.FOLLOW.value,
        )
        self.internal_state = InternalState.TRAIL
        return PlannerOutput(
            self.internal_state,
            self.plan,
            "PUBLISH_OVERRIDE",
            self.plan.reason,
        )

    def _longitudinal_clear(self, blue_s, prediction):
        lead = self.geometry.signed_delta_s(
            prediction.samples[0].s_m,
            blue_s,
        )
        return lead >= self.config.pass_completion_ahead_m

    def _fresh_frozen_safety(self, blue_s, prediction, blue_speed=None):
        return self._temporal_safety(
            self.plan.samples,
            blue_s,
            prediction,
            qualification_margin_m=0.0,
            initial_speed_mps=(blue_speed if
                self.config.longitudinal_acceleration_max_mps2 > 0.0 else None),
        )

    def _create_rejoin(
        self,
        nominal_samples,
        blue_s,
        current_d,
        current_yaw,
        prediction,
        now_s,
        blue_speed=None,
        emergency=False,
    ):
        for length_m in self.config.rejoin_lengths_m:
            samples, corridor, _, reason = self._rejoin_geometry(
                nominal_samples,
                blue_s,
                current_d,
                current_yaw,
                length_m,
            )
            if samples is None:
                continue
            safe, separation = self._temporal_safety(
                samples,
                blue_s,
                prediction,
                qualification_margin_m=0.0,
                initial_speed_mps=(blue_speed if
                    self.config.longitudinal_acceleration_max_mps2 > 0.0 else None),
            )
            if not safe:
                reason = "REJOIN_TEMPORAL_CLEARANCE_INFEASIBLE"
                continue
            end_s = self.geometry.wrap_s(blue_s + length_m)
            self.plan = self._new_plan(
                state=WireState.MERGE.value,
                side=self.plan.side,
                reason=("EMERGENCY_AVOIDANCE:FRESH_R_D_SAFE_REJOIN" if emergency
                        else "PASS_CONFIRMED:FROZEN_REJOIN"),
                prediction=prediction,
                start_s=blue_s,
                commit_end_s=blue_s,
                pass_end_s=blue_s,
                merge_end_s=end_s,
                samples=samples,
                separation=separation,
                corridor=corridor,
                now_s=now_s,
                topology=self.plan.topology,
            )
            self.internal_state = InternalState.PASS_CONFIRMED
            self.last_qualified_prediction_at_s = now_s
            return PlannerOutput(
                self.internal_state,
                self.plan,
                "PUBLISH_OVERRIDE",
                self.plan.reason,
            )
        return PlannerOutput(
            self.internal_state,
            self.plan,
            "CONTINUE_FROZEN",
            "PASS_CONFIRMED_REJOIN_PENDING:" + reason,
        )

    def _abort_degrade(self, reason):
        """Latch fail-closed ABORT without changing identity-bearing payload.

        The runtime adapter gives this one published ABORT a one-planning-cycle
        continuation lease.  S0-R can therefore accept the lifecycle-only
        committed abort and then expire the override to an invalid active
        reference if no safe replacement exists.  No new geometry is invented.
        """
        before = self.plan.identity_digest
        self.plan = replace(
            self.plan,
            state=WireState.ABORT.value,
            valid=True,
            reason="SUCCESSOR_EMERGENCY_DEGRADE:" + reason,
        )
        if self.plan.identity_digest != before:
            raise RuntimeError("emergency ABORT changed identity-bearing payload")
        return PlannerOutput(
            self.internal_state,
            self.plan,
            "PUBLISH_ABORT_DEGRADE",
            self.plan.reason,
        )

    def _update_frozen_pass(
        self,
        nominal_samples,
        blue_s,
        current_d,
        current_yaw,
        prediction,
        now_s,
        blue_speed=None,
    ):
        if self.internal_state == InternalState.PASS_COMMIT:
            self.internal_state = InternalState.FROZEN_PASS

        if (
            self.plan.committed_at_s > 0.0
            and now_s + 1e-12
            >= self.plan.committed_at_s + self.config.pass_timeout_s
        ):
            return self._abort_degrade("FROZEN_PASS_TIMEOUT")

        status = self._prediction_status(prediction, now_s)
        if status == "PREDICTION_QUALIFIED":
            self.last_qualified_prediction_at_s = now_s
            safe, _ = self._fresh_frozen_safety(blue_s, prediction, blue_speed)
            if not safe:
                # A new moving merge is legal ONLY if R-D is fresh AND the
                # geometry, physical trackability and full Red temporal swept
                # occupancy checks all succeed from this measured Ego pose.
                # Otherwise retain the original fail-closed ABORT behavior.
                if self.config.emergency_rejoin_enabled:
                    try:
                        alternative = self._create_rejoin(
                            nominal_samples, blue_s, current_d, current_yaw,
                            prediction, now_s, blue_speed=blue_speed,
                            emergency=True)
                        if (alternative.action == 'PUBLISH_OVERRIDE' and
                                alternative.plan is not None and
                                alternative.plan.state == WireState.MERGE.value):
                            return alternative
                    except (ValueError, ArithmeticError, OverflowError, TypeError):
                        pass
                return self._abort_degrade(
                    "FROZEN_PASS_PREDICTED_COLLISION"
                )
        elif (self.last_qualified_prediction_at_s is None
              or now_s - self.last_qualified_prediction_at_s
              > self.config.prediction_max_age_s + 1e-12):
            return self._abort_degrade("FROZEN_PASS_PREDICTION_LOST:" + status)
        if status == "PREDICTION_QUALIFIED":
            if self._longitudinal_clear(blue_s, prediction):
                return self._create_rejoin(
                    nominal_samples,
                    blue_s,
                    current_d,
                    current_yaw,
                    prediction,
                    now_s,
                    blue_speed=blue_speed,
                )

        progressed = self.geometry.forward_delta_s(
            self.plan.start_s,
            blue_s,
        )
        entry_progress = self.geometry.forward_delta_s(
            self.plan.start_s,
            self.plan.commit_end_s,
        )
        if (
            self.plan.state == WireState.COMMITTED_PASS.value
            and progressed >= entry_progress
        ):
            before = self.plan.identity_digest
            self.plan = replace(
                self.plan,
                state=WireState.PASSING.value,
                reason="FROZEN_PASS:PASS_CORRIDOR_REACHED",
            )
            if self.plan.identity_digest != before:
                raise RuntimeError("wire lifecycle changed identity-bearing payload")
        return PlannerOutput(
            self.internal_state,
            self.plan,
            "CONTINUE_FROZEN",
            status,
        )

    def _update_frozen_rejoin(
        self,
        nominal_samples,
        blue_s,
        current_d,
        prediction,
        now_s,
        blue_speed=None,
    ):
        if self.internal_state == InternalState.PASS_CONFIRMED:
            self.internal_state = InternalState.FROZEN_REJOIN

        status = self._prediction_status(prediction, now_s)
        if status == "PREDICTION_QUALIFIED":
            self.last_qualified_prediction_at_s = now_s
            safe, _ = self._fresh_frozen_safety(blue_s, prediction, blue_speed)
            if not safe:
                return self._abort_degrade(
                    "FROZEN_REJOIN_PREDICTED_COLLISION"
                )
        # Measured completed merge may release nominal even if R-D stopped after
        # the vehicle finished its frozen rejoin. An *unfinished* stale merge
        # fails closed below before another CONTINUE_FROZEN is allowed.

        stations = tuple(sample.s_m for sample in nominal_samples)
        nominal_d = interpolate_periodic_scalar(
            stations,
            tuple(sample.lateral_offset_m for sample in nominal_samples),
            blue_s,
            self.geometry.length,
        )
        progressed = self.geometry.forward_delta_s(
            self.plan.start_s,
            blue_s,
        )
        merge_progress = self.geometry.forward_delta_s(
            self.plan.start_s,
            self.plan.merge_end_s,
        )
        if (
            progressed >= merge_progress
            and abs(current_d - nominal_d)
            <= self.config.blue_tracking_uncertainty_m
        ):
            before = self.plan.identity_digest
            self.plan = replace(
                self.plan,
                state=WireState.COMPLETE.value,
                valid=False,
                reason="FROZEN_REJOIN_COMPLETE",
            )
            if self.plan.identity_digest != before:
                raise RuntimeError("COMPLETE changed identity-bearing payload")
            self.internal_state = InternalState.NOMINAL
            return PlannerOutput(
                self.internal_state,
                self.plan,
                "RELEASE_TO_NOMINAL",
                self.plan.reason,
            )
        if (status != "PREDICTION_QUALIFIED"
                and (self.last_qualified_prediction_at_s is None
                     or now_s - self.last_qualified_prediction_at_s
                     > self.config.prediction_max_age_s + 1e-12)):
            return self._abort_degrade("FROZEN_REJOIN_PREDICTION_LOST:" + status)
        return PlannerOutput(
            self.internal_state,
            self.plan,
            "CONTINUE_FROZEN",
            status,
        )

    def update(
        self,
        nominal_samples,
        blue_x,
        blue_y,
        blue_speed,
        prediction,
        now_s,
        blue_yaw=None,
    ):
        """Run one deterministic Q0 planning step.

        Live candidate time is anchored to measured ego speed, bounded by
        Foundation longitudinal dynamics and each path segment's speed cap.
        """
        if not math.isfinite(blue_speed) or blue_speed < 0.0:
            if self.plan is not None and self.plan.state in (
                WireState.COMMITTED_PASS.value, WireState.PASSING.value,
                WireState.MERGE.value):
                return self._abort_degrade("EGO_SPEED_INVALID")
            return PlannerOutput(
                InternalState.TRAIL, None, "KEEP_NOMINAL", "EGO_SPEED_INVALID")
        projection = self.geometry.project(blue_x, blue_y)
        blue_s = projection.s
        current_d = projection.d
        if blue_yaw is None:
            vector = interpolate_periodic_vector(
                tuple(sample.s_m for sample in nominal_samples),
                tuple(
                    (math.cos(sample.heading_rad), math.sin(sample.heading_rad))
                    for sample in nominal_samples
                ),
                blue_s,
                self.geometry.length,
            )
            blue_yaw = math.atan2(vector[1], vector[0])

        previous_hold = None
        if self.plan is not None:
            if self.plan.state in (
                WireState.COMMITTED_PASS.value,
                WireState.PASSING.value,
            ):
                return self._update_frozen_pass(
                    nominal_samples,
                    blue_s,
                    current_d,
                    blue_yaw,
                    prediction,
                    now_s,
                    blue_speed=blue_speed,
                )
            if self.plan.state == WireState.MERGE.value:
                return self._update_frozen_rejoin(
                    nominal_samples,
                    blue_s,
                    current_d,
                    prediction,
                    now_s,
                    blue_speed=blue_speed,
                )
            if self.plan.state == WireState.COMPLETE.value:
                return PlannerOutput(
                    InternalState.NOMINAL,
                    self.plan,
                    "AWAIT_COMPLETE_PUBLICATION",
                    "COMPLETE_NOT_ACKNOWLEDGED",
                )
            if self.plan.state == WireState.ABORT.value:
                return PlannerOutput(
                    self.internal_state,
                    self.plan,
                    "ABORT_LATCHED_NO_REPUBLISH",
                    self.plan.reason,
                )
            if self.plan.state == WireState.BLOCKING_HOLD.value:
                previous_hold = self.plan
                self.plan = None
                self.internal_state = InternalState.TRAIL

        eligibility = self._eligible(blue_s, prediction, now_s)
        if eligibility != "ELIGIBLE":
            if previous_hold is not None:
                before_digest = previous_hold.identity_digest
                self.plan = replace(
                    previous_hold,
                    state=WireState.COMPLETE.value,
                    valid=False,
                    reason="BLOCKING_HOLD_CLEAR:" + eligibility,
                )
                if self.plan.identity_digest != before_digest:
                    raise RuntimeError(
                        "BLOCKING_HOLD COMPLETE changed identity-bearing payload"
                    )
                self.internal_state = InternalState.NOMINAL
                return PlannerOutput(
                    self.internal_state,
                    self.plan,
                    "RELEASE_TO_NOMINAL",
                    self.plan.reason,
                )
            return PlannerOutput(
                InternalState.TRAIL,
                None,
                "KEEP_NOMINAL",
                eligibility,
            )

        if (self.config.longitudinal_acceleration_max_mps2 <= 0.0
                or self.config.longitudinal_deceleration_max_mps2 <= 0.0):
            return PlannerOutput(
                InternalState.TRAIL, None, "KEEP_NOMINAL",
                "LONGITUDINAL_AUTHORITY_UNAVAILABLE")
        if not self._upper_bound_can_complete(blue_s, blue_speed, prediction):
            return self._follow(
                nominal_samples, blue_s, prediction, now_s,
                "PASS_PHYSICALLY_UNREACHABLE_WITHIN_TIMEOUT", blue_speed=blue_speed)

        candidate, results = self.evaluate_candidates(
            nominal_samples,
            blue_s,
            current_d,
            blue_yaw,
            prediction,
            initial_speed_mps=blue_speed,
        )
        if candidate is None:
            ledger = ",".join(
                f"{topology.value}={results[topology.value]}"
                for topology in self.PASS_TOPOLOGIES
            )
            return self._follow(
                nominal_samples,
                blue_s,
                prediction,
                now_s,
                "NO_FEASIBLE_PASS:" + ledger,
                blue_speed=blue_speed,
            )

        return self.commit_candidate(
            candidate,
            prediction,
            blue_s,
            now_s,
        )
