from dataclasses import dataclass, replace
import math
import time

import numpy as np

from f1tenth_racing_control.core import EgoState
from f1tenth_racing_control.mpc import EnvelopeBoundReference


@dataclass(frozen=True)
class FallbackSafetyAssessment:
    recoverable: bool
    reason: str
    initial_margin: float = math.nan
    terminal_margin: float = math.nan
    min_predicted_margin: float = math.nan
    min_speed_headroom_mps: float = math.nan
    min_physical_clearance_m: float = math.nan
    # Node telemetry consumes the same six-state terminal contract as B0:
    # x, y, yaw, speed, slip angle, yaw rate.
    terminal_state: tuple = (
        math.nan, math.nan, math.nan,
        math.nan, math.nan, math.nan,
    )


@dataclass(frozen=True)
class FallbackTransitionResult:
    decision: object
    assessment: FallbackSafetyAssessment
    phase: str


class FallbackSafetyTransitionSupervisor:
    """Runtime fallback-startup and short-horizon safety supervisor.

    Its preview is only a limited-time risk check, not a full stopping proof.
    Do not infer that passing this test qualifies high-speed or race operation.
    """

    BLEND_START_MPS = 0.15
    BLEND_END_MPS = 0.75
    ACCEL_CAP_MPS2 = 4.0
    ACCEL_RELEASE_SPEED_MPS = 1.25
    RESERVE = 0.05
    CORRIDOR_GUARD_M = 0.30
    FULL_HOLD_TICKS = 2
    # CPU budget is intentionally independent of the MPC's primary deadline.
    # If the short preview is too slow, retain braking without changing the
    # previous steering. This can never be treated as a safety certificate.
    EMERGENCY_STEER_BUDGET_S = 0.008

    def __init__(self, mpc, governor, fallback, stability):
        self.mpc = mpc
        self.governor = governor
        self.fallback = fallback
        self.stability = stability
        self.steer_rate = float(mpc.limits["steering_rate_abs_max_rad_s"])
        self.jerk = float(mpc.limits["jerk_abs_max_m_s3"])
        self.reset()

    def reset(self):
        self.full_hold_ticks = 0
        self.steering_handoff_complete = False
        self.startup_complete = False

    @staticmethod
    def _rate(previous, target, rate_limit, dt_s):
        step = max(0.0, float(rate_limit) * max(float(dt_s), 0.0))
        return min(float(previous) + step, max(float(previous) - step, float(target)))

    @staticmethod
    def _state6_from_message(msg):
        return (
            float(msg.px),
            float(msg.py),
            float(msg.yaw),
            float(msg.v),
            float(msg.slip_angle),
            float(msg.omega),
        )

    @staticmethod
    def _state6_from_vector(vector):
        values = np.asarray(vector, dtype=float).reshape(-1)
        if values.size < 6:
            return (
                math.nan, math.nan, math.nan,
                math.nan, math.nan, math.nan,
            )
        return tuple(float(values[index]) for index in range(6))

    def current_safety(self, msg):
        slip_margin, yaw_margin = self.stability._dynamic_margins(msg)
        clearance = self.stability.corridor.clearance(
            float(msg.px), float(msg.py), float(msg.yaw)
        )
        values = (float(slip_margin), float(yaw_margin), float(clearance))
        finite = all(math.isfinite(v) for v in values)
        ok = finite and values[0] >= 0.0 and values[1] >= 0.0 and values[2] >= self.CORRIDOR_GUARD_M
        return ok, values

    def shape(self, msg, decision, previous_control, dt_s):
        speed = max(0.0, float(msg.v))
        raw_steer = float(decision.steering_rad)
        raw_accel = float(decision.acceleration_mps2)

        if self.startup_complete:
            return decision, "COMPLETE"

        if self.steering_handoff_complete and speed >= self.ACCEL_RELEASE_SPEED_MPS:
            self.startup_complete = True
            return decision, "COMPLETE"

        if self.steering_handoff_complete:
            accel = self._rate(
                previous_control[1],
                min(raw_accel, self.ACCEL_CAP_MPS2),
                self.jerk,
                dt_s,
            )
            return replace(
                decision,
                acceleration_mps2=accel,
                mode="FALLBACK_STARTUP_ACCEL_RELEASE",
                reason="STARTUP_ACCEL_RELEASE:REAL_OBJECT_PREFLIGHT",
            ), "STARTUP_ACCEL_RELEASE"

        if speed < self.BLEND_START_MPS:
            alpha = 0.0
            phase = "STARTUP_BOOTSTRAP"
        elif speed < self.BLEND_END_MPS:
            alpha = (speed - self.BLEND_START_MPS) / (self.BLEND_END_MPS - self.BLEND_START_MPS)
            phase = "STARTUP_BLEND"
        else:
            alpha = 1.0
            phase = "STARTUP_FULL_HOLD"

        steer = self._rate(
            previous_control[0],
            alpha * raw_steer,
            self.steer_rate,
            dt_s,
        )
        accel = self._rate(
            previous_control[1],
            min(raw_accel, self.ACCEL_CAP_MPS2),
            self.jerk,
            dt_s,
        )
        return replace(
            decision,
            steering_rad=steer,
            acceleration_mps2=accel,
            mode=f"FALLBACK_{phase}",
            reason=f"{phase}:REAL_OBJECT_PREFLIGHT",
        ), phase

    def point_metrics(self, reference, envelope, vector):
        # Identical B0 margin/station/envelope calculation; use one projection
        # instead of repeating geometric project/interpolate/envelope.at.
        margin, component, station, execution = self.governor._margin_detail(
            reference, envelope, vector)
        speed = max(float(vector[3]), 0.0)
        headroom = execution - speed
        clearance = self.stability.corridor.clearance(
            float(vector[0]), float(vector[1]), float(vector[2])
        )
        return float(margin), str(component), float(headroom), float(clearance)

    def preview(self, reference, envelope, msg, decision, deadline_ns=None):
        # Deadline expiration means NOT CERTIFIED. Never truncate the horizon
        # and return recoverable=True for a partially evaluated rollout.
        def expired():
            return deadline_ns is not None and time.perf_counter_ns() >= deadline_ns

        if expired():
            return FallbackSafetyAssessment(False, "PREVIEW_DEADLINE_UNPROVEN")
        bounded = EnvelopeBoundReference(reference, envelope)
        vector = np.asarray(self.mpc.state_vector(msg), dtype=float)
        previous = (float(decision.steering_rad), float(decision.acceleration_mps2))

        if not np.isfinite(vector).all():
            return FallbackSafetyAssessment(
                False,
                "NONFINITE_STATE",
                terminal_state=self._state6_from_vector(vector),
            )

        vector = np.asarray(self.mpc._step(vector, previous, vector[2]), dtype=float)
        if not np.isfinite(vector).all():
            return FallbackSafetyAssessment(
                False,
                "NONFINITE_NEXT_STATE",
                terminal_state=self._state6_from_vector(vector),
            )

        if expired():
            return FallbackSafetyAssessment(
                False, "PREVIEW_DEADLINE_UNPROVEN",
                terminal_state=self._state6_from_vector(vector),
            )
        margin, component, headroom, clearance = self.point_metrics(reference, envelope, vector)
        initial = margin
        terminal = margin
        min_margin = margin
        min_headroom = headroom
        min_clearance = clearance

        def reason_now():
            if min_margin < self.RESERVE:
                return f"PREDICTED_MARGIN:{component}"
            if min_headroom < 0.0:
                return "PREDICTED_SPEED_ENVELOPE"
            if min_clearance < self.CORRIDOR_GUARD_M:
                return "PREDICTED_CORRIDOR"
            return ""

        reason = reason_now()
        if reason:
            return FallbackSafetyAssessment(
                False,
                reason,
                initial,
                terminal,
                min_margin,
                min_headroom,
                min_clearance,
                terminal_state=self._state6_from_vector(vector),
            )

        for _ in range(int(self.mpc.config.horizon_steps)):
            if expired():
                return FallbackSafetyAssessment(
                    False, "PREVIEW_DEADLINE_UNPROVEN",
                    initial, terminal, min_margin, min_headroom,
                    min_clearance, terminal_state=self._state6_from_vector(vector),
                )
            state = EgoState(float(vector[0]), float(vector[1]), float(vector[2]), float(vector[3]))
            follow = self.fallback.compute(
                bounded,
                state,
                previous[0],
                previous[1],
                float(self.mpc.config.timestep_s),
            )
            previous = (float(follow.steering_rad), float(follow.acceleration_mps2))
            vector = np.asarray(self.mpc._step(vector, previous, vector[2]), dtype=float)
            if not np.isfinite(vector).all():
                return FallbackSafetyAssessment(
                    False,
                    "NONFINITE_ROLLOUT",
                    terminal_state=self._state6_from_vector(vector),
                )
            if expired():
                return FallbackSafetyAssessment(
                    False, "PREVIEW_DEADLINE_UNPROVEN",
                    initial, terminal, min_margin, min_headroom,
                    min_clearance, terminal_state=self._state6_from_vector(vector),
                )
            margin, component, headroom, clearance = self.point_metrics(reference, envelope, vector)
            terminal = margin
            min_margin = min(min_margin, margin)
            min_headroom = min(min_headroom, headroom)
            min_clearance = min(min_clearance, clearance)
            reason = reason_now()
            if reason:
                return FallbackSafetyAssessment(
                    False,
                    reason,
                    initial,
                    terminal,
                    min_margin,
                    min_headroom,
                    min_clearance,
                    terminal_state=self._state6_from_vector(vector),
                )

        return FallbackSafetyAssessment(
            True,
            "PREDICTIVE_SAFE",
            initial,
            terminal,
            min_margin,
            min_headroom,
            min_clearance,
            terminal_state=self._state6_from_vector(vector),
        )

    def select_braking_steer(self, reference, envelope, msg, fallback_steer,
                              previous_steer, braking_accel, dt_s):
        """Bounded short-horizon mitigation for rejected fallback commands.

        Select among rate/angle-compliant steering candidates using the
        installed dynamic bicycle model, dynamic margins and the physical
        footprint corridor. No candidate is called 'safe to stop'; checking
        0.1 s cannot certify the full braking trajectory.
        """
        try:
            values = (float(previous_steer), float(fallback_steer),
                      float(braking_accel), float(dt_s))
            vector = np.asarray(self.mpc.state_vector(msg), dtype=float)
            if (not all(math.isfinite(v) for v in values) or dt_s <= 0.0 or
                    not np.isfinite(vector).all()):
                return math.nan, "STEER_UNPROVEN_INVALID_STATE"
            max_angle = float(self.mpc.limits["steering_angle_abs_max_rad"])
            slew = self.steer_rate * min(float(dt_s), 0.1)
            if not math.isfinite(max_angle) or max_angle <= 0.0 or slew <= 0.0:
                return math.nan, "STEER_UNPROVEN_INVALID_LIMITS"
            lo = max(-max_angle, previous_steer - slew)
            hi = min(max_angle, previous_steer + slew)
            if lo > hi:
                return math.nan, "STEER_UNPROVEN_INVALID_RANGE"
            requested = min(hi, max(lo, fallback_steer))
            neutral = min(hi, max(lo, 0.0))
            baseline = min(hi, max(lo, previous_steer))
            candidates = tuple(dict.fromkeys((baseline, requested, neutral, lo, hi)))
            started = time.perf_counter()
            best = None
            horizon = min(4, int(self.mpc.config.horizon_steps))
            for steer in candidates:
                if time.perf_counter() - started > self.EMERGENCY_STEER_BUDGET_S:
                    return math.nan, "STEER_UNPROVEN_BUDGET_EXCEEDED"
                future = vector.copy()
                minimum = math.inf
                valid = True
                for _ in range(horizon):
                    if time.perf_counter() - started > self.EMERGENCY_STEER_BUDGET_S:
                        return math.nan, "STEER_UNPROVEN_BUDGET_EXCEEDED"
                    future = np.asarray(self.mpc._step(
                        future, (steer, braking_accel), future[2]
                    ), dtype=float)
                    if not np.isfinite(future).all():
                        valid = False
                        break
                    margin, _component, headroom, clearance = self.point_metrics(
                        reference, envelope, future,
                    )
                    if not all(math.isfinite(v) for v in (margin, headroom, clearance)):
                        valid = False
                        break
                    # Dimensionless, signed reserves: <0 is not qualified.
                    score = min(
                        margin - self.RESERVE,
                        headroom / max(float(envelope.speed_cap_mps), 0.5),
                        (clearance - self.CORRIDOR_GUARD_M) /
                        max(self.CORRIDOR_GUARD_M, 0.1),
                    )
                    minimum = min(minimum, score)
                if valid and (best is None or minimum > best[0] + 1e-9 or
                              (abs(minimum - best[0]) <= 1e-9 and
                               abs(steer - requested) < abs(best[1] - requested))):
                    best = (minimum, float(steer))
            if best is None:
                return math.nan, "STEER_UNPROVEN_NO_FINITE_CANDIDATE"
            if best[0]<0:
                return math.nan, "STEER_SHORT_PREVIEW_UNSAFE"
            return best[1], "STEER_SHORT_PREVIEW_POSITIVE_NOT_STOP_CERTIFIED"
        except (RuntimeError, TypeError, ValueError, ArithmeticError, KeyError):
            return math.nan, "STEER_UNPROVEN_EVALUATION_FAILED"

    def evaluate(self, reference, envelope, msg, decision, previous_control, dt_s, deadline_ns=None):
        current_ok, current_values = self.current_safety(msg)
        shaped, phase = self.shape(msg, decision, previous_control, dt_s)

        if not current_ok:
            self.full_hold_ticks = 0
            return FallbackTransitionResult(
                shaped,
                FallbackSafetyAssessment(
                    False,
                    "CURRENT_UNSAFE",
                    min_physical_clearance_m=current_values[2],
                    terminal_state=self._state6_from_message(msg),
                ),
                phase,
            )

        speed = max(0.0, float(msg.v))
        if (
            not self.steering_handoff_complete
            and phase == "STARTUP_BOOTSTRAP"
            and speed < self.BLEND_START_MPS
        ):
            assessment = FallbackSafetyAssessment(
                True,
                "BOOTSTRAP_CURRENT_SAFE",
                min_physical_clearance_m=current_values[2],
                terminal_state=self._state6_from_message(msg),
            )
        else:
            assessment = self.preview(reference, envelope, msg, shaped, deadline_ns=deadline_ns)

        if not assessment.recoverable:
            if not self.steering_handoff_complete:
                self.full_hold_ticks = 0
            return FallbackTransitionResult(shaped, assessment, phase)

        if not self.steering_handoff_complete:
            if phase == "STARTUP_FULL_HOLD":
                self.full_hold_ticks += 1
                if self.full_hold_ticks >= self.FULL_HOLD_TICKS:
                    self.steering_handoff_complete = True
            else:
                self.full_hold_ticks = 0

        return FallbackTransitionResult(shaped, assessment, phase)
