import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from f1tenth_racing_raceline.speed_adapter import maximum_tire_force
from f1tenth_racing_raceline.speed_adapter import load_speed_problem, solve_speed_problem, validate_dense
from f1tenth_racing_raceline.speed_profile import SpeedEnvelopeConfig, generate_speed_profile


CONFIG = SpeedEnvelopeConfig(1.0e-12, 1.0e-12, 1000)


def circle(count=40, radius=5.0):
    points = [
        (radius * math.cos(2.0 * math.pi * index / count), radius * math.sin(2.0 * math.pi * index / count))
        for index in range(count)
    ]
    segment = math.dist(points[0], points[1])
    return [index * segment for index in range(count)], points


def profile(curvature, global_speed=20.0, acceleration=4.0, braking=6.0, lateral=10.0):
    station_s, points = circle(len(curvature))
    return generate_speed_profile(
        station_s, points, curvature, global_speed, acceleration, braking, lateral, CONFIG
    )


def assert_invariants(result, tolerance=1.0e-10):
    speed = np.asarray(result.target_speed)
    ds = np.asarray(result.segment_length)
    following = np.roll(speed, -1)
    curvature = np.abs(np.asarray(result.curvature))
    assert np.all(speed >= 0.0)
    assert np.all(speed <= result.global_speed_ceiling + tolerance)
    assert np.all(speed * speed * curvature <= result.lateral_acceleration_ceiling + tolerance)
    assert np.all(following * following - speed * speed <= 2.0 * result.acceleration_limit * ds + tolerance)
    assert np.all(speed * speed - following * following <= 2.0 * result.braking_limit * ds + tolerance)


def test_low_curvature_loop_reaches_global_ceiling_without_zero_speed_seam():
    result = profile([0.0] * 40)
    assert result.target_speed == pytest.approx([20.0] * 40)
    assert result.target_speed[0] > 0.0
    assert_invariants(result)


def test_constant_radius_uses_predictable_lateral_ceiling():
    result = profile([0.25] * 40, lateral=9.0)
    assert result.target_speed == pytest.approx([6.0] * 40)
    assert set(result.limiting_constraint) == {"LATERAL_CURVATURE"}
    assert_invariants(result)


def test_forward_acceleration_and_backward_braking_propagate_around_seam():
    curvature = [0.0] * 20
    curvature[0] = 4.0
    result = profile(curvature, acceleration=1.5, braking=3.0, lateral=4.0)
    speed = np.asarray(result.target_speed)
    ds = np.asarray(result.segment_length)
    forward = result.limiting_constraint.index("FORWARD_ACCELERATION")
    previous = (forward - 1) % len(speed)
    braking = result.limiting_constraint.index("BACKWARD_BRAKING")
    following = (braking + 1) % len(speed)
    assert speed[forward] ** 2 - speed[previous] ** 2 == pytest.approx(2.0 * 1.5 * ds[previous])
    assert speed[braking] ** 2 - speed[following] ** 2 == pytest.approx(2.0 * 3.0 * ds[braking])
    assert_invariants(result)


def test_mixed_curvature_has_multiple_limiting_classes_and_is_deterministic():
    curvature = [0.0] * 30
    curvature[5:8] = [0.2, 0.5, 0.2]
    first = profile(curvature, acceleration=2.0, braking=3.0)
    second = profile(curvature, acceleration=2.0, braking=3.0)
    assert first == second
    assert len(set(first.limiting_constraint)) >= 3
    assert_invariants(first)


def test_invalid_inputs_fail_explicitly():
    station_s, points = circle(6)
    with pytest.raises(ValueError):
        generate_speed_profile(station_s, points[:-1], [0.0] * 6, 20, 1, 1, 1, CONFIG)
    with pytest.raises(ValueError):
        generate_speed_profile(station_s, points, [math.nan] * 6, 20, 1, 1, 1, CONFIG)
    with pytest.raises(ValueError):
        generate_speed_profile(station_s, points, [0.0] * 6, -1, 1, 1, 1, CONFIG)
    with pytest.raises(ValueError):
        generate_speed_profile(station_s, points, [0.0] * 6, 20, 0, 1, 1, CONFIG)
    invalid_s = list(station_s)
    invalid_s[2] = invalid_s[1]
    with pytest.raises(ValueError):
        generate_speed_profile(invalid_s, points, [0.0] * 6, 20, 1, 1, 1, CONFIG)
    duplicate = list(points)
    duplicate[1] = duplicate[0]
    with pytest.raises(ValueError):
        generate_speed_profile(station_s, duplicate, [0.0] * 6, 20, 1, 1, 1, CONFIG)
    with pytest.raises(ValueError):
        SpeedEnvelopeConfig(0.0, 1.0e-12, 10)


def test_foundation_pacejka_peak_force_derivation_matches_equation():
    force, slip = maximum_tire_force({"pacejka_b": 1.5, "pacejka_c": 1.5, "pacejka_d_n": 30.0})
    assert force == pytest.approx(30.0)
    assert slip == pytest.approx(math.tan(math.pi / 3.0) / 1.5)


def test_iccas2025_speed_profile_satisfies_dense_and_periodic_constraints():
    problem = load_speed_problem()
    result = solve_speed_problem(problem)
    period = problem["r_a0_metadata"]["metrics"]["centerline"]["length"]
    validation = validate_dense(result, period, 20)
    assert problem["hashes"]["r_a0_csv"] == "38e5a070066d5a7f37832029ae256ba12ec044979ac3d4cd24fc49eb1f34aa9c"
    assert result.iterations <= problem["config"].maximum_iterations
    assert all(value <= 1.0e-12 for key, value in validation.items() if key != "dense_sample_count")
    assert_invariants(result)


def test_installed_r_a1_artifact_is_self_consistent():
    data = Path(get_package_share_directory("f1tenth_racing_raceline")) / "data"
    csv_path = data / "iccas2025_r_a1_speed_profile.csv"
    metadata = json.loads((data / "iccas2025_r_a1_speed_profile.json").read_text())
    assert hashlib.sha256(csv_path.read_bytes()).hexdigest() == metadata["output_csv_sha256"]
    assert metadata["source_r_a0"]["csv_sha256"] == "38e5a070066d5a7f37832029ae256ba12ec044979ac3d4cd24fc49eb1f34aa9c"
    assert metadata["profile"]["sample_count"] == 66
    assert metadata["valid"] is True
