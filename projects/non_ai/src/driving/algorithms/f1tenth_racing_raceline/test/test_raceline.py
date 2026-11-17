import math
import hashlib
import json
from pathlib import Path

import pytest

from f1tenth_racing_geometry import ClosedTrackGeometry
from ament_index_python.packages import get_package_share_directory
from f1tenth_racing_raceline.foundation_adapter import load_problem, solve_problem
from f1tenth_racing_raceline.raceline import MinimumCurvatureOptimizer, OptimizerConfig, TrackCorridor, curvature_metrics


CONFIG = OptimizerConfig((0.2, 0.1, 0.05, 0.025), 8, 0.05, 1.0e-14)


def corridor(points, left_width=1.0, right_width=1.0):
    geometry = ClosedTrackGeometry(points)
    left, right = [], []
    for point in points:
        projection = geometry.project(*point)
        left.append((point[0] - left_width * math.sin(projection.heading), point[1] + left_width * math.cos(projection.heading)))
        right.append((point[0] + right_width * math.sin(projection.heading), point[1] - right_width * math.cos(projection.heading)))
    return TrackCorridor.create(points, left, right, [left_width] * len(points), [right_width] * len(points), 0.1)


def test_rectangular_loop_reduces_real_curvature_objective_and_is_periodic():
    points = [(0, 0), (4, 0), (6, 0), (6, 2), (6, 4), (2, 4), (0, 4), (0, 2)]
    result = MinimumCurvatureOptimizer(corridor(points), CONFIG).solve()
    assert result.final_objective < result.initial_objective
    assert result.line_length > 0.0
    geometry = ClosedTrackGeometry(result.points)
    start, end = geometry.interpolate(0.0), geometry.interpolate(geometry.length)
    assert (end.x, end.y) == pytest.approx((start.x, start.y), abs=1.0e-12)


def test_oval_and_asymmetric_corridor_are_feasible_and_deterministic():
    points = [(4 * math.cos(2 * math.pi * i / 24), 2 * math.sin(2 * math.pi * i / 24)) for i in range(24)]
    track = corridor(points, left_width=1.2, right_width=0.8)
    first = MinimumCurvatureOptimizer(track, CONFIG).solve()
    second = MinimumCurvatureOptimizer(track, CONFIG).solve()
    assert first == second
    assert first.final_objective <= first.initial_objective + 1.0e-12
    assert all(low <= d <= high for d, low, high in zip(first.lateral_offset, track.lower_offsets, track.upper_offsets))


def test_invalid_corridor_and_offsets_fail_explicitly():
    with pytest.raises(ValueError):
        TrackCorridor.create([(0, 0), (1, 0)], [(0, 1)] * 2, [(0, -1)] * 2, [1] * 2, [1] * 2, 0.1)
    track = corridor([(0, 0), (2, 0), (2, 2), (0, 2)])
    with pytest.raises(ValueError):
        track.points_for_offsets([math.nan] * 4)
    with pytest.raises(ValueError):
        track.points_for_offsets([10.0] * 4)


def test_curvature_objective_matches_circle_limit_sensibly():
    radius = 3.0
    points = [(radius * math.cos(2 * math.pi * i / 200), radius * math.sin(2 * math.pi * i / 200)) for i in range(200)]
    metrics = curvature_metrics(points)
    assert metrics["maximum_absolute_curvature"] == pytest.approx(1.0 / radius, rel=2.0e-4)
    assert metrics["objective"] == pytest.approx(2.0 * math.pi / radius, rel=2.0e-4)


def test_installed_s0_geometry_is_the_only_geometry_owner():
    assert ClosedTrackGeometry.__module__ == "f1tenth_racing_geometry.closed_track"
    source = (Path(__file__).parents[1] / "f1tenth_racing_raceline/raceline.py").read_text()
    assert "def wrap_s" not in source
    assert "def project(" not in source
    assert "def from_frenet" not in source


def test_iccas2025_full_track_qualification():
    problem = load_problem()
    line, validation, centerline_metrics, line_metrics = solve_problem(problem)
    assert len(line.x) == 66
    assert line.final_objective < line.initial_objective
    assert line_metrics["objective"] == pytest.approx(line.final_objective, abs=1.0e-12)
    assert validation["minimum_footprint_adjusted_corridor_margin_m"] >= -1.0e-9
    assert validation["wall_collision_corner_hits"] == 0
    assert validation["self_intersection_count"] == 0
    assert validation["seam_position_error_m"] == 0.0


def test_installed_qualified_artifact_is_self_consistent():
    data = Path(get_package_share_directory("f1tenth_racing_raceline")) / "data"
    csv_path = data / "iccas2025_r_a0_baseline.csv"
    metadata = json.loads((data / "iccas2025_r_a0_baseline.json").read_text())
    assert hashlib.sha256(csv_path.read_bytes()).hexdigest() == metadata["output_csv_sha256"]
    assert metadata["sample_count"] == 66
    assert metadata["valid"] is True
