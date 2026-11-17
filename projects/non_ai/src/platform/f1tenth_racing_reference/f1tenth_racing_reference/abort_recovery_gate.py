"""Deterministic stationary H2H ABORT-to-nominal recovery precondition.

Only returns an eligibility bit; never authorizes control, a stop certificate,
or a Red pass. S0-R publishes nominal only after conservative measured checks,
then R-B and independent Guard MUST separately qualify every actuator command.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RecoveryDecision:
    allowed: bool
    reason: str


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _distance_point_segment(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    d2 = dx * dx + dy * dy
    if d2 < 1e-12:
        return math.hypot(px-x0, py-y0), 0.0
    t = max(0.0, min(1.0, ((px-x0)*dx+(py-y0)*dy)/d2))
    return math.hypot(px-x0-t*dx, py-y0-t*dy), t


def _segment_minimum(a, b, x, y):
    return _distance_point_segment(x, y, a[0], a[1], b[0], b[1])[0]


def evaluate_abort_recovery(nominal_samples, ego, red_prediction, *,
                            now_ns, ego_radius_m, collision_known,
                            collision_active, source_age_limit_ns=75_000_000,
                            prediction_age_limit_ns=350_000_000,
                            separation_margin_m=0.20,
                            nominal_distance_max_m=0.15,
                            heading_error_max_rad=0.20,
                            minimum_future_s=2.0,
                            red_accel_uncertainty_mps2=1.0):
    """Conservative *parked* return to nominal; no on-track moving bypass.

    A dropped prediction, stale ego sample, unknown collision signal, moving ego,
    large deviation from nominal, or uncertain Red occupancy denies recovery.
    """
    no=lambda reason: RecoveryDecision(False, reason)
    if not collision_known or collision_active:
        return no('RECOVERY_COLLISION_NOT_CLEARED')
    if ego is None or red_prediction is None:
        return no('RECOVERY_EGO_OR_RED_MISSING')
    try:
        now_ns=int(now_ns)
        observed_ns=_stamp_ns(ego.physics_stamp)
        source_ns=_stamp_ns(red_prediction.source_observation_time)
        scalars=(ego.px,ego.py,ego.yaw,ego.v,ego.vx,ego.vy,ego.omega,
                 ego.a,ego.accel,ego_radius_m,separation_margin_m,
                 nominal_distance_max_m,heading_error_max_rad,minimum_future_s,
                 red_accel_uncertainty_mps2)
        if not all(math.isfinite(float(v)) for v in scalars):
            return no('RECOVERY_NONFINITE')
        if not (0 <= now_ns-observed_ns <= source_age_limit_ns):
            return no('RECOVERY_EGO_STALE_OR_FUTURE')
        if not (0 <= now_ns-source_ns <= prediction_age_limit_ns):
            return no('RECOVERY_RED_STALE_OR_FUTURE')
        if not (red_accel_uncertainty_mps2>=0 and ego_radius_m>0
                and separation_margin_m>=0 and
                nominal_distance_max_m>0 and heading_error_max_rad>0 and minimum_future_s>0):
            return no('RECOVERY_INVALID_BOUND')
        if (abs(float(ego.v))>0.025 or math.hypot(ego.vx,ego.vy)>0.025 or
                abs(ego.omega)>0.05 or abs(ego.a)>0.25 or
                float(ego.accel)>0.05):
            return no('RECOVERY_EGO_NOT_PARKED')
        samples=tuple(nominal_samples)
        if len(samples)<3:
            return no('RECOVERY_NOMINAL_UNAVAILABLE')
        nearest=(float('inf'),None)
        for a,b in zip(samples,samples[1:]+samples[:1]):
            values=(a.x_m,a.y_m,b.x_m,b.y_m,a.heading_rad,b.heading_rad)
            if not all(math.isfinite(float(v)) for v in values):
                return no('RECOVERY_NOMINAL_NONFINITE')
            distance,t=_distance_point_segment(ego.px,ego.py,a.x_m,a.y_m,b.x_m,b.y_m)
            if distance<nearest[0]:
                heading=math.atan2(math.sin(a.heading_rad)*(1-t)+math.sin(b.heading_rad)*t,
                                    math.cos(a.heading_rad)*(1-t)+math.cos(b.heading_rad)*t)
                nearest=(distance,heading)
        if (nearest[0]>nominal_distance_max_m or
                abs(math.atan2(math.sin(ego.yaw-nearest[1]),
                               math.cos(ego.yaw-nearest[1])))>heading_error_max_rad):
            return no('RECOVERY_NOMINAL_ALIGNMENT_UNPROVEN')
        red=red_prediction
        if (not red.valid or red.schema_version != 1 or
                red.source_lifecycle!='TRACKED' or
                not red.source_opponent_id or not red.track_id or
                not red.frame_id or
                not math.isfinite(float(red.source_confidence)) or
                red.source_confidence <= 0 or
                red.source_hit_count<2):
            return no('RECOVERY_RED_IDENTITY_UNPROVEN')
        predicted=tuple(red.samples)
        if len(predicted)<2:
            return no('RECOVERY_RED_HORIZON_MISSING')
        last_t=0.0
        prev=None
        age_s=(now_ns-source_ns)*1e-9
        required_horizon_s=age_s+minimum_future_s+0.025
        for sm in predicted:
            t=float(sm.dt_s)
            values=(t,sm.x_m,sm.y_m,sm.vx_mps,sm.vy_mps,
                    sm.occupancy_half_length_m,sm.occupancy_half_width_m)
            if (not all(math.isfinite(float(v)) for v in values) or
                    t<=last_t or t-last_t>0.21 or
                    min(sm.occupancy_half_length_m,sm.occupancy_half_width_m)<=0):
                return no('RECOVERY_RED_SAMPLING_INVALID')
            r=math.hypot(sm.occupancy_half_length_m,sm.occupancy_half_width_m)
            r+=0.5*red_accel_uncertainty_mps2*t*t+ego_radius_m+separation_margin_m
            if prev is None:
                # The first segment is unknown; bound backward by current
                # velocity, with uncertainty inflation at both endpoints.
                start=(float(sm.x_m)-float(sm.vx_mps)*t,
                       float(sm.y_m)-float(sm.vy_mps)*t)
                prev=start
            end=(float(sm.x_m),float(sm.y_m))
            if _segment_minimum(prev,end,float(ego.px),float(ego.py))<=r:
                return no('RECOVERY_RED_SWEPT_OCCUPANCY_UNPROVEN')
            prev=end
            last_t=t
            if last_t >= required_horizon_s:
                break
        if last_t < required_horizon_s:
            return no('RECOVERY_RED_HORIZON_UNCOVERED')
        return RecoveryDecision(True,'PARKED_NOMINAL_RED_CLEAR_PRECONDITION_ONLY')
    except (ValueError,TypeError,AttributeError,OverflowError):
        return no('RECOVERY_INPUT_INVALID')
