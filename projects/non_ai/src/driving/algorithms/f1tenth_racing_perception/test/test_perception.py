import math
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from f1tenth_racing_geometry import ClosedTrackGeometry
from rclpy.qos import ReliabilityPolicy

from f1tenth_racing_perception import (
    CvKalmanTracker,
    Pose2D,
    PoseBuffer,
    TrackCorridor,
    UkfExtentTracker,
    extract_measurements,
)
from f1tenth_racing_perception.core import ClusterMeasurement
from f1tenth_racing_perception.node import SENSOR_QOS
from f1tenth_racing_perception.expected_scan import StaticExpectedScan


def test_perception_sensor_qos_cannot_backpressure_physics():
    assert SENSOR_QOS.reliability == ReliabilityPolicy.BEST_EFFORT


def config():
    return {
        "static_residual_tolerance_m": 0.12,
        "breakpoint_noise_m": 0.03,
        "breakpoint_scale": 2.0,
        "minimum_cluster_points": 2,
        "minimum_extent_m": 0.03,
        "maximum_extent_m": 0.70,
        "maximum_measurement_range_m": 9.5,
        "association_mahalanobis_squared": 16.0,
        "measurement_variance_m2": 0.01,
        "process_acceleration_variance_m2_s4": 2.0,
        "maximum_missed_scans": 2,
        "minimum_confirmed_hits": 2,
        "minimum_confirmation_along_speed_mps": 0.25,
        "visible_surface_offset_m": 0.2,
    }


class FreeMap:
    @staticmethod
    def occupied(x, y):
        return False


class GridMap:
    resolution = 0.1
    origin_x = origin_y = 0.0
    threshold = 128
    flip_y = False
    pixel_array = np.full((20, 30), 255, dtype=np.uint8)
    pixel_array[:, 25] = 0
    height, width = pixel_array.shape


def test_static_expected_scan_preserves_fixed_grid_hit_samples():
    from f1tenth_racing_sim.sensing.lidar import scan_occupancy

    grid = GridMap()
    projector = StaticExpectedScan(grid, -0.5, 0.01, 101, 0.1, 3.0)
    angles = -0.5 + np.arange(101) * 0.01

    authoritative = scan_occupancy(grid, 0.5, 1.0, angles, 0.1, 3.0)
    accelerated = projector.scan(0.5, 1.0, 0.0)

    assert accelerated == authoritative


def test_iccas2025_expected_scan_matches_frozen_golden_corpus_exactly():
    from f1tenth_racing_sim.world.map import OccupancyMap

    golden = json.loads(Path(__file__).with_name("golden_expected_scan.json").read_text())
    share = Path(get_package_share_directory("f1tenth_racing_sim"))
    environment = yaml.safe_load((share / "config/competition_environment.yaml").read_text())
    map_path = share / environment["map"]["image_file"]
    assert hashlib.sha256(map_path.read_bytes()).hexdigest() == golden["map_sha256"]
    occupancy = OccupancyMap(map_path, environment["map"])
    projector = StaticExpectedScan(
        occupancy, golden["angle_min_rad"], golden["angle_increment_rad"],
        golden["beam_count"], golden["range_min_m"], golden["range_max_m"],
    )
    for _, x_m, y_m, yaw_rad, expected_hash in golden["cases"]:
        ranges = np.asarray(projector.scan(x_m, y_m, yaw_rad), dtype=np.float64)
        assert hashlib.sha256(ranges.tobytes()).hexdigest() == expected_hash


def measurement(x, sensor_x=0.0):
    return ClusterMeasurement(x, 0.0, x, 0.0, 0.3, abs(x - sensor_x), 5, sensor_x, 0.0)


def test_pose_buffer_interpolates_position_and_wrapped_yaw():
    buffer = PoseBuffer()
    buffer.add(Pose2D(1.0, 0.0, 0.0, math.pi - 0.1))
    buffer.add(Pose2D(1.1, 1.0, 2.0, -math.pi + 0.1))
    pose = buffer.interpolate(1.05, 0.2)
    assert pose.x_m == pytest.approx(0.5)
    assert pose.y_m == pytest.approx(1.0)
    assert abs(abs(pose.yaw_rad) - math.pi) < 1e-12
    assert buffer.interpolate(0.9, 0.2) is None
    assert buffer.at_or_before(1.08, 0.1) == buffer.poses[0]
    assert buffer.at_or_before(1.2, 0.05) is None
    assert buffer.candidates_at_or_before(1.1, 0.2) == tuple(buffer.poses)
    assert buffer.bracket(1.05, 0.1) == tuple(buffer.poses)
    assert buffer.bracket(1.2, 0.05) is None


def test_static_residual_and_range_adaptive_cluster():
    observed = np.full(20, 8.0)
    observed[8:13] = 5.0
    measurements, residuals = extract_measurements(
        observed, np.full(20, 8.0), -0.1, 0.01, Pose2D(1.0, 0, 0, 0),
        0.12, 0.0, config(), FreeMap(),
    )
    assert residuals == 5
    assert len(measurements) == 1
    assert measurements[0].point_count == 5
    assert 0.15 < measurements[0].extent_m < 0.25


def test_stage_timing_marks_do_not_change_measurement_semantics():
    observed = np.full(20, 8.0)
    observed[8:13] = 5.0
    marks = {}
    profiled = extract_measurements(
        observed, np.full(20, 8.0), -0.1, 0.01, Pose2D(1.0, 0, 0, 0),
        0.12, 0.0, config(), FreeMap(), stage_marks=marks,
    )
    plain = extract_measurements(
        observed, np.full(20, 8.0), -0.1, 0.01, Pose2D(1.0, 0, 0, 0),
        0.12, 0.0, config(), FreeMap(),
    )
    assert profiled == plain
    assert tuple(marks) == ("residual", "dynamic", "clustering", "candidates")


def test_foundation_corridor_rejects_wall_side_vehicle_centers():
    rows = [
        {"x_m": "0", "y_m": "0", "w_tr_left_m": "1", "w_tr_right_m": "1"},
        {"x_m": "1", "y_m": "0", "w_tr_left_m": "1", "w_tr_right_m": "1"},
        {"x_m": "1", "y_m": "1", "w_tr_left_m": "1", "w_tr_right_m": "1"},
        {"x_m": "0", "y_m": "1", "w_tr_left_m": "1", "w_tr_right_m": "1"},
    ]
    corridor = TrackCorridor(rows, 0.1)
    assert corridor.contains(0.5, 0.2)
    assert not corridor.contains(0.5, -0.95)


def test_cv_lifecycle_covariance_and_loss_are_deterministic():
    tracker = CvKalmanTracker(config())
    assert not tracker.update([], 1.0).valid
    assert not tracker.update([measurement(5.0)], 1.1).valid
    tracked = tracker.update([measurement(5.1)], 1.2)
    assert tracked.valid and tracked.lifecycle == "TRACKED"
    assert np.all(np.linalg.eigvalsh(tracked.covariance) > 0.0)
    assert tracker.update([], 1.3).lifecycle == "COASTING"
    tracker.update([], 1.4)
    assert tracker.update([], 1.5).lifecycle == "UNINITIALIZED"


def test_tentative_track_requires_consecutive_measurements():
    settings = config()
    settings["minimum_confirmed_hits"] = 3
    tracker = CvKalmanTracker(settings)
    tracker.update([measurement(5.0)], 1.0)
    assert tracker.update([], 1.1).lifecycle == "UNINITIALIZED"
    assert not tracker.update([measurement(5.1)], 1.2).valid


def test_lost_track_resets_before_reacquisition():
    settings = config()
    tracker = CvKalmanTracker(settings)
    tracker.update([measurement(5.0)], 1.0)
    tracker.update([measurement(5.1)], 1.1)
    for index in range(settings["maximum_missed_scans"] + 1):
        estimate = tracker.update([], 1.2 + 0.1 * index)
    assert estimate.lifecycle == "UNINITIALIZED"
    assert not tracker.update([measurement(-5.0)], 2.0).valid


def test_confirmation_requires_canonical_forward_motion():
    settings = config()
    settings["minimum_confirmed_hits"] = 3
    geometry = ClosedTrackGeometry([(0, 0), (10, 0), (10, 10), (0, 10)])
    forward = CvKalmanTracker(settings, geometry)
    backward = CvKalmanTracker(settings, geometry)
    for index, x_m in enumerate((1.0, 1.2, 1.4)):
        forward_estimate = forward.update([measurement(x_m)], 1.0 + 0.1 * index)
    for index, x_m in enumerate((1.4, 1.2, 1.0)):
        backward_estimate = backward.update([measurement(x_m)], 1.0 + 0.1 * index)
    assert forward_estimate.valid
    assert not backward_estimate.valid


def test_stationary_visible_opponent_remains_a_stable_tentative_observation():
    settings = config()
    settings["minimum_confirmed_hits"] = 3
    geometry = ClosedTrackGeometry([(0, 0), (10, 0), (10, 10), (0, 10)])
    tracker = CvKalmanTracker(settings, geometry)

    estimates = [tracker.update([measurement(5.0)], 1.0 + 0.1 * index)
                 for index in range(6)]

    assert all(not estimate.valid for estimate in estimates)
    assert all(estimate.lifecycle == "TENTATIVE" for estimate in estimates)
    assert all(estimate.hits >= 2 and estimate.measurement_age_s == 0.0
               for estimate in estimates[1:])


def test_extent_ukf_corrects_visible_surface_bias():
    cv = CvKalmanTracker(config())
    ukf = UkfExtentTracker(config())
    visible_surface = measurement(4.8)
    cv_estimate = cv.update([visible_surface], 1.0)
    ukf_estimate = ukf.update([visible_surface], 1.0)
    assert cv_estimate.state[0] == pytest.approx(4.8)
    assert ukf_estimate.state[0] == pytest.approx(5.0)


def test_invalid_tracker_time_rejected():
    tracker = CvKalmanTracker(config())
    tracker.update([measurement(1.0)], 1.0)
    with pytest.raises(ValueError):
        tracker.update([measurement(1.1)], 1.0)
