import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from f1tenth_racing_control import (
    ControlConfig,
    EgoState,
    ModelAidedPursuit,
    PursuitReference,
)


def config(**updates):
    values = {
        "wheelbase_m": 0.34,
        "lookahead_base_m": 0.7,
        "lookahead_speed_gain_s": 0.1,
        "lookahead_min_m": 0.6,
        "lookahead_max_m": 1.0,
        "speed_feedback_gain_per_s": 2.0,
        "qualified_speed_cap_mps": 2.0,
        "steering_max_rad": 4.0,
        "steering_rate_max_rad_s": 4.0,
        "acceleration_max_mps2": 40.0,
        "braking_max_mps2": 40.0,
        "jerk_max_mps3": 100.0,
        "degraded_deceleration_mps2": 2.0,
    }
    values.update(updates)
    return ControlConfig(**values)


def reference(points, curvature=0.0, speed=10.0, acceleration=0.0):
    lengths = [math.dist(point, points[(index + 1) % len(points)]) for index, point in enumerate(points)]
    period = sum(lengths)
    stations = [0.0]
    for length in lengths[:-1]:
        stations.append(stations[-1] + length)
    samples = tuple(SimpleNamespace(
        sample_index=index,
        s_m=stations[index],
        x_m=point[0],
        y_m=point[1],
        heading_rad=math.atan2(
            points[(index + 1) % len(points)][1] - point[1],
            points[(index + 1) % len(points)][0] - point[0],
        ),
        curvature_per_m=curvature,
        lateral_offset_m=0.0,
        target_speed_mps=speed,
        target_accel_mps2=acceleration,
        ds_to_next_m=lengths[index],
    ) for index, point in enumerate(points))
    return PursuitReference(samples, period)


def rectangular_reference(**kwargs):
    return reference(((0, 0), (4, 0), (4, 2), (0, 2)), **kwargs)


def decide(state, ref=None, previous_steering=0.0, previous_acceleration=0.0, dt=0.025):
    return ModelAidedPursuit(config()).compute(
        ref or rectangular_reference(), state, previous_steering, previous_acceleration, dt
    )


def test_straight_centered_and_speed_cap_are_deterministic():
    state = EgoState(1.0, 0.0, 0.0, 0.0)
    first = decide(state)
    second = decide(state)
    assert first == second
    assert first.steering_rad == pytest.approx(0.0, abs=1e-12)
    assert first.target_speed_mps == 2.0
    assert first.acceleration_mps2 > 0.0
    assert first.qualification_cap_active is True
    assert first.mode == "LOCAL_SPEED_CAPPED"


def test_lateral_and_heading_errors_correct_in_expected_direction():
    assert decide(EgoState(1.0, 0.2, 0.0, 1.0)).steering_rad < 0.0
    assert decide(EgoState(1.0, -0.2, 0.0, 1.0)).steering_rad > 0.0
    assert decide(EgoState(1.0, 0.0, 0.2, 1.0)).steering_rad < 0.0


def test_constant_left_curvature_has_positive_feedforward():
    points = tuple(
        (2.0 * math.cos(index * math.pi / 8), 2.0 * math.sin(index * math.pi / 8))
        for index in range(16)
    )
    result = decide(EgoState(2.0, 0.0, math.pi / 2, 1.0), reference(points, curvature=0.5))
    assert result.steering_rad > 0.0


def test_speed_feedback_accelerates_and_brakes_without_raising_reference():
    accelerate = decide(EgoState(1.0, 0.0, 0.0, 0.0))
    brake = decide(EgoState(1.0, 0.0, 0.0, 3.0))
    assert accelerate.acceleration_mps2 > 0.0
    assert brake.acceleration_mps2 < 0.0
    assert accelerate.target_speed_mps <= 2.0
    uncapped = rectangular_reference(speed=1.0, acceleration=0.5)
    result = decide(EgoState(1.0, 0.0, 0.0, 1.0), uncapped)
    assert result.target_speed_mps == 1.0
    assert result.acceleration_mps2 > 0.0


def test_rate_and_absolute_limits_include_seam():
    controller = ModelAidedPursuit(config())
    ref = rectangular_reference()
    result = controller.compute(ref, EgoState(0.0, 0.1, -math.pi / 2, 4.0), 0.0, 0.0, 0.01)
    assert 0.0 <= result.target_progress_m < ref.station_period_m
    assert abs(result.steering_rad) <= 0.04 + 1e-12
    assert abs(result.acceleration_mps2) <= 1.0 + 1e-12
    assert abs(result.steering_rad) <= 4.0
    assert abs(result.acceleration_mps2) <= 40.0


def test_invalid_state_reference_and_timing_degrade_deterministically():
    controller = ModelAidedPursuit(config())
    ref = rectangular_reference()
    cases = (
        controller.compute(ref, EgoState(float("nan"), 0, 0, 0), 0.3, 0, 0.025),
        controller.compute(None, EgoState(0, 0, 0, 0), 0.3, 0, 0.025),
        controller.compute(ref, EgoState(0, 0, 0, 0), 0.3, 0, -1.0),
    )
    assert [case.mode for case in cases] == ["INVALID_STATE", "INVALID_REFERENCE", "INVALID_TIMING"]
    for index, case in enumerate(cases):
        assert case.degradation_active is True
        assert case.steering_rad == 0.3
        assert case.acceleration_mps2 == (-2.0 if index != 1 else 0.0)
        assert math.isfinite(case.steering_rad)
        assert math.isfinite(case.acceleration_mps2)


def test_degraded_control_brakes_to_zero_without_commanding_reverse():
    controller = ModelAidedPursuit(config())
    assert controller.degraded(0.2, "INVALID_REFERENCE", 0.0, 0.025).acceleration_mps2 == 0.0
    assert math.isclose(
        controller.degraded(0.2, "INVALID_REFERENCE", 0.01, 0.025).acceleration_mps2,
        -0.4,
    )


@pytest.mark.parametrize(
    "bad",
    [
        {"qualified_speed_cap_mps": 0.0},
        {"lookahead_min_m": 0.8, "lookahead_base_m": 0.7},
        {"degraded_deceleration_mps2": 50.0},
    ],
)
def test_invalid_configuration_rejected(bad):
    with pytest.raises(ValueError):
        config(**bad).validate()


def test_malformed_reference_rejected():
    with pytest.raises(ValueError):
        PursuitReference((), 1.0)
    samples = rectangular_reference().samples
    with pytest.raises(ValueError):
        PursuitReference(samples, float("nan"))
def test_diagnostic_primary_withhold_is_opt_in_and_skips_primary_solve():
    source = (Path(__file__).parents[1] / "f1tenth_racing_control/node.py").read_text()
    assert 'declare_parameter("diagnostic_withhold_primary", False)' in source
    diagnostic = source.index('primary_status = "DIAGNOSTIC_WITHHELD"')
    solve = source.index("primary = self.primary.solve", diagnostic)
    assert diagnostic < solve
