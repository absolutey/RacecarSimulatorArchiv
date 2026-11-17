"""Regression: MPC's bounded reference must be consumable by Pursuit."""
import math
from types import SimpleNamespace

from f1tenth_racing_control.core import (
    ControlConfig, EgoState, ModelAidedPursuit, PursuitReference,
)
from f1tenth_racing_control.mpc import (
    EnvelopeBoundReference, ExecutionSpeedEnvelope,
)


def test_envelope_bound_reference_supports_pursuit_iteration_and_limits_speed():
    points = ((0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0))
    lengths = [math.dist(points[i], points[(i + 1) % 4]) for i in range(4)]
    stations = (0.0, lengths[0], lengths[0] + lengths[1],
                lengths[0] + lengths[1] + lengths[2])
    samples = tuple(SimpleNamespace(
        sample_index=i, s_m=stations[i], x_m=x, y_m=y,
        heading_rad=0.0, curvature_per_m=1.0, lateral_offset_m=0.0,
        target_speed_mps=10.0, target_accel_mps2=0.0,
        ds_to_next_m=lengths[i],
    ) for i, (x, y) in enumerate(points))
    nominal = PursuitReference(samples, sum(lengths))
    envelope = ExecutionSpeedEnvelope(nominal, 20.0, 4.0, 2.0, 1.0, 1.0)
    bounded = EnvelopeBoundReference(nominal, envelope)
    assert bounded.samples is nominal.samples
    assert bounded.scalar('target_speed_mps', 1.0) < 10.0

    control = ModelAidedPursuit(ControlConfig(
        wheelbase_m=0.33, lookahead_base_m=1.0,
        lookahead_speed_gain_s=0.1, lookahead_min_m=0.5,
        lookahead_max_m=3.0, speed_feedback_gain_per_s=1.0,
        qualified_speed_cap_mps=20.0, steering_max_rad=0.45,
        steering_rate_max_rad_s=1.2, acceleration_max_mps2=2.0,
        braking_max_mps2=4.0, jerk_max_mps3=10.0,
        degraded_deceleration_mps2=2.0,
    ))
    state = EgoState(1.0, 0.1, 0.0, 0.0)
    decision = control.compute(bounded, state, 0.0, 0.0, 0.025)
    projection = bounded.geometry.project(state.x_m, state.y_m)
    physical = bounded.geometry.interpolate(projection.s)
    station = bounded.station_at_physical(physical)
    limit = bounded.scalar('target_speed_mps', station)
    assert 0.0 <= decision.target_speed_mps <= limit + 1e-9
    assert math.isfinite(decision.steering_rad)
    assert math.isfinite(decision.acceleration_mps2)
