import math
from pathlib import Path

import pytest

from f1tenth_racing_geometry import (
    ClosedTrackGeometry,
    interpolate_periodic_angle,
    interpolate_periodic_scalar,
    interpolate_periodic_vector,
    load_centerline_csv,
)


SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]


def assert_progress_close(actual, expected, length, tolerance=1.0e-9):
    error = abs((actual - expected + 0.5 * length) % length - 0.5 * length)
    assert error <= tolerance


def test_wrap_progress_deltas_and_seam():
    track = ClosedTrackGeometry(SQUARE)
    assert track.wrap_s(track.length) == 0.0
    assert track.wrap_s(-1.0) == 39.0
    assert track.wrap_s(3.0 + 8.0 * track.length) == pytest.approx(3.0)
    assert track.forward_delta_s(39.0, 1.0) == pytest.approx(2.0)
    assert track.signed_delta_s(1.0, 39.0) == pytest.approx(-2.0)
    assert track.interpolate(0.0) == track.interpolate(track.length)


def test_projection_sign_inverse_and_corner_tie_break():
    track = ClosedTrackGeometry(SQUARE)
    left = track.project(4.0, 2.0)
    right = track.project(4.0, -2.0)
    assert (left.s, left.d, left.segment_index) == pytest.approx((4.0, 2.0, 0))
    assert (right.s, right.d, right.segment_index) == pytest.approx((4.0, -2.0, 0))
    assert track.from_frenet(left.s, left.d).x == pytest.approx(4.0)
    assert track.from_frenet(left.s, left.d).y == pytest.approx(2.0)
    assert track.project(10.0, 10.0).segment_index == 1


def test_duplicate_closure_and_invalid_geometry_queries():
    assert ClosedTrackGeometry(SQUARE + [SQUARE[0]]).points == tuple(SQUARE)
    for points in (SQUARE[:2], [(0, 0), (1, 0), (1, 0), (0, 1)], [(0, 0), (math.nan, 0), (0, 1)]):
        with pytest.raises(ValueError):
            ClosedTrackGeometry(points)
    track = ClosedTrackGeometry(SQUARE)
    for query in (math.nan, math.inf):
        with pytest.raises(ValueError):
            track.project(query, 0.0)


def test_periodic_scalar_vector_and_angle_interpolation():
    sample_s = [0.0, 10.0, 20.0, 30.0]
    assert interpolate_periodic_scalar(sample_s, [0, 10, 20, 30], 35, 40) == pytest.approx(15.0)
    assert interpolate_periodic_vector(sample_s, [(0, 0), (10, 20), (20, 40), (30, 60)], 5, 40) == pytest.approx((5, 10))
    angle = interpolate_periodic_angle([0.0, 5.0], [math.radians(179), math.radians(-179)], 2.5, 10.0)
    assert abs(abs(angle) - math.pi) < math.radians(1.1)


def test_deterministic_synthetic_property_sweeps():
    track = ClosedTrackGeometry(SQUARE)
    for index in range(160):
        s = track.length * (index + 0.25) / 160.0
        assert track.wrap_s(s + 7 * track.length) == pytest.approx(track.wrap_s(s), abs=1.0e-12)
        sample = track.interpolate(s)
        projected = track.project(sample.x, sample.y)
        assert_progress_close(projected.s, s, track.length)
        assert projected.d == pytest.approx(0.0, abs=1.0e-12)
    for segment_start in (0.0, 10.0, 20.0, 30.0):
        for along in (2.0, 5.0, 8.0):
            s = segment_start + along
            for d in (-0.25, 0.25):
                point = track.from_frenet(s, d)
                recovered = track.project(point.x, point.y)
                assert_progress_close(recovered.s, s, track.length)
                assert recovered.d == pytest.approx(d, abs=1.0e-9)


def test_projection_is_rigid_transform_consistent():
    base = ClosedTrackGeometry(SQUARE)
    angle, tx, ty = 0.7, 3.0, -4.0
    cosine, sine = math.cos(angle), math.sin(angle)

    def transform(point):
        x, y = point
        return (cosine * x - sine * y + tx, sine * x + cosine * y + ty)

    moved = ClosedTrackGeometry([transform(point) for point in SQUARE])
    query = (4.0, 2.0)
    original, transformed = base.project(*query), moved.project(*transform(query))
    assert transformed.s == pytest.approx(original.s, abs=1.0e-12)
    assert transformed.d == pytest.approx(original.d, abs=1.0e-12)
    assert transformed.heading == pytest.approx(original.heading + angle, abs=1.0e-12)


def test_full_iccas2025_geometry_qualification():
    asset = Path(__file__).parents[3] / "environment/f1tenth_racing_sim/maps/iccas2025/iccas2025_centerline.csv"
    track = ClosedTrackGeometry(load_centerline_csv(asset))
    assert len(track.points) == 66
    assert track.segment_count == 66
    assert track.length == pytest.approx(43.0470539284193, abs=1.0e-12)
    assert track.interpolate(0.0).x == pytest.approx(track.interpolate(track.length).x, abs=1.0e-12)
    assert track.interpolate(0.0).y == pytest.approx(track.interpolate(track.length).y, abs=1.0e-12)

    signatures = []
    for repeat in range(2):
        run = []
        for index in range(660):
            s = track.length * (index + 0.37) / 660.0
            sample = track.interpolate(s)
            center = track.project(sample.x, sample.y)
            assert_progress_close(center.s, s, track.length, 2.0e-12)
            assert center.d == pytest.approx(0.0, abs=2.0e-12)
            run.append((center.s, center.d, center.segment_index))
        signatures.append(run)
    assert signatures[0] == signatures[1]

    cumulative = 0.0
    for start, end in zip(track.points, track.points[1:] + track.points[:1]):
        segment_length = math.hypot(end[0] - start[0], end[1] - start[1])
        for fraction in (0.2, 0.5, 0.8):
            s = cumulative + fraction * segment_length
            for d in (-0.2, 0.2):
                point = track.from_frenet(s, d)
                recovered = track.project(point.x, point.y)
                assert_progress_close(recovered.s, s, track.length, 2.0e-9)
                assert recovered.d == pytest.approx(d, abs=2.0e-9)
        cumulative += segment_length
