from collections import deque
from dataclasses import dataclass, replace
import math


def clamp(value, low, high):
    return min(high, max(low, value))


@dataclass(frozen=True)
class DynamicAdmissionResult:
    active: bool
    mode: str
    reason: str
    decision: object
    slip_margin: float = math.nan
    yaw_rate_margin: float = math.nan
    physical_clearance_m: float = math.nan
    time_to_zero_s: float = math.inf


class PostBootstrapDynamicAdmission:
    """Preventive positive-authority shaping before reactive STABILIZE.

    Thresholds are intentionally derived from the already-qualified
    StabilityRecoverySupervisor entry detector rather than creating a second
    unrelated stability definition:

      - trend margin ceiling: identical to CausalEntryDetector (0.6)
      - recovery trigger T_zero: identical to CausalEntryDetector (0.15 s)
      - preventive horizon: 2 x recovery trigger horizon (0.30 s)
      - trend window: identical to CausalEntryDetector (0.10 s)

    The owner never changes steering. It only reduces positive longitudinal
    authority. Existing negative acceleration is never weakened. If the
    reactive StabilityRecoverySupervisor becomes active later in the same
    control cycle, that supervisor still has final command authority.
    """

    def __init__(
        self,
        stability,
        degraded_deceleration_mps2,
        jerk_limit_mps3,
        minimum_speed_mps,
        corridor_guard_m,
    ):
        self.stability = stability
        self.degraded_deceleration_mps2 = float(degraded_deceleration_mps2)
        self.jerk_limit_mps3 = float(jerk_limit_mps3)
        self.minimum_speed_mps = float(minimum_speed_mps)
        self.corridor_guard_m = float(corridor_guard_m)

        entry = stability.entry
        self.trend_margin_ceiling = float(entry.trend_margin_ceiling)
        self.recovery_ttz_s = float(entry.time_to_zero_threshold_s)
        self.preventive_ttz_s = 2.0 * self.recovery_ttz_s
        self.trend_window_s = float(entry.trend_window_s)

        if not 0.0 < self.recovery_ttz_s < self.preventive_ttz_s:
            raise ValueError("invalid dynamic-admission horizons")
        if self.degraded_deceleration_mps2 <= 0.0:
            raise ValueError("degraded deceleration must be positive")
        if self.jerk_limit_mps3 <= 0.0:
            raise ValueError("jerk limit must be positive")

        self.history = deque(maxlen=16)

    def reset(self):
        self.history.clear()

    def _risk_time_to_zero(self, now_s, slip_margin, yaw_margin):
        now_s = float(now_s)
        slip_margin = float(slip_margin)
        yaw_margin = float(yaw_margin)

        if min(slip_margin, yaw_margin) <= 0.0:
            return 0.0, "CURRENT_DYNAMIC_MARGIN"

        target = now_s - self.trend_window_s
        prior = (
            min(self.history, key=lambda item: abs(item[0] - target))
            if self.history else None
        )
        if prior is None:
            return math.inf, ""

        dt_s = now_s - prior[0]
        if dt_s < 0.05:
            return math.inf, ""

        best_ttz = math.inf
        component = ""
        for name, margin, old_margin in (
            ("SLIP", slip_margin, prior[1]),
            ("YAW_RATE", yaw_margin, prior[2]),
        ):
            slope = (margin - old_margin) / dt_s
            if (
                margin <= self.trend_margin_ceiling
                and slope < -1e-9
            ):
                ttz = max(0.0, margin / (-slope))
                if ttz < best_ttz:
                    best_ttz = ttz
                    component = name
        return best_ttz, component

    def _rate(self, previous, target, dt_s):
        step = self.jerk_limit_mps3 * clamp(float(dt_s), 1e-4, 0.05)
        return clamp(float(target), float(previous) - step, float(previous) + step)

    def update(self, message, decision, previous_acceleration, now_s, dt_s):
        slip_margin, yaw_margin = self.stability._dynamic_margins(message)
        clearance = self.stability.corridor.clearance(
            float(message.px), float(message.py), float(message.yaw)
        )
        ttz, component = self._risk_time_to_zero(
            now_s, slip_margin, yaw_margin
        )
        self.history.append(
            (float(now_s), float(slip_margin), float(yaw_margin))
        )

        speed = max(0.0, float(message.v))
        raw_accel = float(decision.acceleration_mps2)

        # Existing braking/deceleration is already conservative; never weaken it.
        if raw_accel <= 0.0:
            return DynamicAdmissionResult(
                False, "DYNAMIC_ADMISSION_IDLE", "NONPOSITIVE_ACCELERATION",
                decision, slip_margin, yaw_margin, clearance, ttz,
            )

        if speed < self.minimum_speed_mps:
            return DynamicAdmissionResult(
                False, "DYNAMIC_ADMISSION_IDLE", "STARTUP_OWNER_ACTIVE",
                decision, slip_margin, yaw_margin, clearance, ttz,
            )

        mode = ""
        reason = ""
        target_accel = raw_accel

        if clearance < self.corridor_guard_m:
            mode = "DYNAMIC_ADMISSION_BRAKE"
            reason = "POST_BOOTSTRAP_CORRIDOR_RESERVE"
            target_accel = -self.degraded_deceleration_mps2
        elif ttz <= self.recovery_ttz_s:
            # Enter preventive braking continuously as T_zero approaches zero.
            fraction = clamp(ttz / self.recovery_ttz_s, 0.0, 1.0)
            target_accel = -self.degraded_deceleration_mps2 * (1.0 - fraction)
            mode = "DYNAMIC_ADMISSION_BRAKE"
            reason = f"POST_BOOTSTRAP_{component}_TRENDED_RISK"
        elif ttz <= self.preventive_ttz_s:
            # Reduce only positive authority before the reactive recovery trigger.
            authority = (
                (ttz - self.recovery_ttz_s)
                / (self.preventive_ttz_s - self.recovery_ttz_s)
            )
            target_accel = raw_accel * clamp(authority, 0.0, 1.0)
            mode = "DYNAMIC_ADMISSION_TAPER"
            reason = f"POST_BOOTSTRAP_{component}_TREND_RESERVE"
        else:
            return DynamicAdmissionResult(
                False, "DYNAMIC_ADMISSION_IDLE", "DYNAMIC_RESERVE_OK",
                decision, slip_margin, yaw_margin, clearance, ttz,
            )

        shaped_accel = self._rate(previous_acceleration, target_accel, dt_s)
        # This owner may only remove positive authority, never add it.
        shaped_accel = min(raw_accel, shaped_accel)

        target_speed = min(
            float(decision.target_speed_mps),
            speed,
        )
        shaped = replace(
            decision,
            acceleration_mps2=shaped_accel,
            mode=mode,
            reason=reason,
            target_speed_mps=target_speed,
            degradation_active=True,
        )
        return DynamicAdmissionResult(
            True, mode, reason, shaped,
            slip_margin, yaw_margin, clearance, ttz,
        )
