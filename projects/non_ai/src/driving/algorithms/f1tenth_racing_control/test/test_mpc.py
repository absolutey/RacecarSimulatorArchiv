import math
from types import SimpleNamespace

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from f1tenth_racing_control import PursuitReference
from f1tenth_racing_control.mpc import (
    B0RecoverabilityGovernor,
    EnvelopeBoundReference,
    ExecutionSpeedEnvelope,
    LtvDynamicBicycleMpc,
    MpcConfig,
    PrimaryAdoptionGate,
)


def reference():
    points = ((0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0))
    lengths = [math.dist(point, points[(i + 1) % len(points)]) for i, point in enumerate(points)]
    stations, distance = [], 0.0
    for length in lengths:
        stations.append(distance); distance += length
    samples = tuple(SimpleNamespace(
        sample_index=i, s_m=stations[i], x_m=point[0], y_m=point[1],
        heading_rad=math.atan2(points[(i + 1) % 4][1] - point[1], points[(i + 1) % 4][0] - point[0]),
        curvature_per_m=0.0, lateral_offset_m=0.0, target_speed_mps=2.0,
        target_accel_mps2=0.0, ds_to_next_m=lengths[i],
    ) for i, point in enumerate(points))
    return PursuitReference(samples, distance)


def configuration(**updates):
    values = dict(
        horizon_steps=4, timestep_s=0.025, maximum_solver_iterations=200,
        solver_penalty=1.0, solver_tolerance=0.001, primary_deadline_s=1.0,
        lateral_error_weight=30.0, longitudinal_error_weight=1.0,
        heading_error_weight=12.0, speed_error_weight=4.0, slip_weight=1.0,
        yaw_rate_weight=0.5, steering_state_weight=0.2,
        acceleration_state_weight=0.05, steering_input_weight=0.05,
        acceleration_input_weight=0.01, steering_rate_weight=0.5,
        acceleration_rate_weight=0.02, preview_distance_m=3.0,
        preview_step_m=0.25, preview_braking_mps2=8.0,
        lateral_acceleration_limit_mps2=12.0, lateral_error_limit_m=0.28,
        slip_angle_limit_rad=0.35,
        fallback_recoverable_lateral_error_m=0.55,
        fallback_recoverable_heading_error_rad=0.85,
    )
    values.update(updates)
    return MpcConfig(**values)


@pytest.fixture
def foundation():
    path = get_package_share_directory("f1tenth_racing_sim") + "/config/competition_environment.yaml"
    dynamics = yaml.safe_load(open(path))["dynamics"]
    return dynamics, dynamics["limits"]


def state(**updates):
    values = dict(px=1.0, py=0.1, yaw=0.0, v=2.0, slip_angle=0.0,
                  omega=0.0, steer=0.0, accel=0.0)
    values.update(updates)
    return SimpleNamespace(**values)


def test_mpc_returns_finite_rate_bounded_primary_command(foundation):
    dynamics, limits = foundation
    result = LtvDynamicBicycleMpc(dynamics, limits, configuration()).solve(
        reference(), state(), 1.0, (0.0, 0.0), 2.0
    )
    assert result.status == "PRIMARY_VALID"
    assert math.isfinite(result.steering_rad)
    assert math.isfinite(result.acceleration_mps2)
    assert abs(result.steering_rad) <= limits["steering_rate_abs_max_rad_s"] * 0.025 + 1e-6
    assert abs(result.acceleration_mps2) <= limits["jerk_abs_max_m_s3"] * 0.025 + 1e-6
    assert result.target_speed_mps <= 2.0
    assert result.max_constraint_residual <= configuration().solver_tolerance


def test_mpc_marks_solution_late_without_changing_command(foundation):
    dynamics, limits = foundation
    result = LtvDynamicBicycleMpc(
        dynamics, limits, configuration(primary_deadline_s=1e-9)
    ).solve(reference(), state(), 1.0, (0.0, 0.0), 2.0)
    assert result.status == "PRIMARY_LATE"


def test_mpc_configuration_rejects_invalid_horizon():
    with pytest.raises(ValueError):
        configuration(horizon_steps=1)


def test_curvature_preview_only_reduces_upstream_speed(foundation):
    dynamics, limits = foundation
    curved = reference()
    samples = tuple(SimpleNamespace(**{**sample.__dict__, "curvature_per_m": 0.5,
                                       "target_speed_mps": 10.0})
                    for sample in curved.samples)
    curved = PursuitReference(samples, curved.station_period_m)
    target, reason = LtvDynamicBicycleMpc(
        dynamics, limits, configuration()
    )._local_speed_limit(curved, 1.0, 20.0)
    assert reason == "LATERAL_DYNAMIC_LIMIT"
    assert target == pytest.approx(math.sqrt(12.0 / 0.5))
    assert target <= 10.0


def test_execution_envelope_propagates_braking_and_preserves_authority():
    source = reference()
    samples = tuple(SimpleNamespace(**{
        **sample.__dict__,
        "target_speed_mps": 12.0,
        "curvature_per_m": 1.0 if sample.sample_index == 2 else 0.0,
    }) for sample in source.samples)
    source = PursuitReference(samples, source.station_period_m)
    envelope = ExecutionSpeedEnvelope(source, 20.0, 4.0, 2.0, 1.0, 1.0)
    assert envelope.final[1] < envelope.raw[1]
    assert envelope.backward[1] == pytest.approx(math.sqrt(2.0 ** 2 + 2.0 * 2.0 * 2.0))
    assert all(0.0 <= value <= 12.0 for value in envelope.final)


def test_execution_envelope_limits_acceleration_and_is_cyclic_at_seam():
    source = reference()
    samples = tuple(SimpleNamespace(**{
        **sample.__dict__,
        "target_speed_mps": 12.0,
        "curvature_per_m": 1.0 if sample.sample_index == 3 else 0.0,
    }) for sample in source.samples)
    source = PursuitReference(samples, source.station_period_m)
    envelope = ExecutionSpeedEnvelope(source, 20.0, 4.0, 20.0, 1.0, 1.0)
    seam_reachable = math.sqrt(envelope.final[-1] ** 2 + 2.0 * 1.0 * samples[-1].ds_to_next_m)
    assert envelope.final[0] <= seam_reachable + 1e-9
    before = envelope.at(source.station_period_m - 1e-8).execution_envelope_mps
    after = envelope.at(1e-8).execution_envelope_mps
    assert before == pytest.approx(after, abs=1e-6)


def test_fallback_reference_is_bounded_by_cap_and_execution_envelope():
    source = reference()
    samples = tuple(SimpleNamespace(**{
        **sample.__dict__, "target_speed_mps": 10.0,
        "curvature_per_m": 1.0,
    }) for sample in source.samples)
    source = PursuitReference(samples, source.station_period_m)
    envelope = ExecutionSpeedEnvelope(source, 20.0, 4.0, 8.0, 40.0, 1.0)
    bounded = EnvelopeBoundReference(source, envelope)
    assert bounded.scalar("target_speed_mps", 1.0) == pytest.approx(2.0)
    assert bounded.scalar("target_speed_mps", 1.0) <= 5.6


def test_primary_gate_rejects_one_tick_and_admits_one_horizon():
    gate = PrimaryAdoptionGate(0.1)
    assert gate.update(True, 1.0)[:2] == (False, "PRIMARY_ADMISSION_PENDING")
    assert gate.update(False, 1.025)[:2] == (False, "FALLBACK_REQUIRED")
    assert not gate.update(True, 2.0)[0]
    assert not gate.update(True, 2.075)[0]
    assert gate.update(True, 2.1)[:2] == (True, "PRIMARY_ADMITTED")


def test_primary_gate_hysteresis_tolerates_short_continuation_gap():
    gate = PrimaryAdoptionGate(0.1)
    gate.update(True, 1.0)
    gate.update(True, 1.1)
    assert gate.update(False, 1.125)[:2] == (False, "PRIMARY_CONTINUATION_GAP")
    assert gate.update(True, 1.15)[:2] == (True, "PRIMARY_CONTINUATION")
    gate.reset()
    assert gate.update(True, 2.0)[:2] == (False, "PRIMARY_ADMISSION_PENDING")


class NeutralFallback:
    def __init__(self):
        self.calls = 0

    def compute(self, _reference, _state, steering, acceleration, _dt):
        self.calls += 1
        return SimpleNamespace(steering_rad=steering, acceleration_mps2=acceleration)


def test_recoverability_governor_accepts_low_risk_command_with_bounded_work(foundation):
    dynamics, limits = foundation
    mpc = LtvDynamicBicycleMpc(dynamics, limits, configuration())
    fallback = NeutralFallback()
    envelope = mpc.speed_envelope(reference(), 2.0)
    result = B0RecoverabilityGovernor(mpc, fallback, stopping_checker=lambda *_:True).evaluate(
        reference(), envelope, state(py=0.0), (0.0, 0.0),
    )
    assert result.recoverable
    assert result.reason == "B0_STOP_CERTIFIED"
    assert fallback.calls == configuration().horizon_steps


def test_recoverability_governor_rejects_before_unsafe_primary_adoption(foundation):
    dynamics, limits = foundation
    mpc = LtvDynamicBicycleMpc(dynamics, limits, configuration())
    fallback = NeutralFallback()
    envelope = mpc.speed_envelope(reference(), 2.0)
    result = B0RecoverabilityGovernor(mpc, fallback).evaluate(
        reference(), envelope, state(py=1.0), (0.0, 0.0),
    )
    assert not result.recoverable
    assert "LATERAL_ERROR" in result.reason
    assert fallback.calls == 0


def test_recoverability_governor_fails_closed_without_stop_certificate(foundation):
    dynamics, limits = foundation
    mpc = LtvDynamicBicycleMpc(dynamics, limits, configuration())
    fallback = NeutralFallback()
    envelope = mpc.speed_envelope(reference(), 2.0)
    result = B0RecoverabilityGovernor(mpc, fallback).evaluate(
        reference(), envelope, state(py=0.0), (0.0, 0.0),
    )
    assert not result.recoverable
    assert result.reason == "B0_STOP_UNPROVEN_OR_FAILED"
