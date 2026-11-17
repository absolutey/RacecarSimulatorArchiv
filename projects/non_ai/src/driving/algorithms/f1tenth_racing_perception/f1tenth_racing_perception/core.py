from dataclasses import dataclass
import bisect
import math
import time

import numpy as np
from f1tenth_racing_geometry import ClosedTrackGeometry, interpolate_periodic_scalar


def wrap_angle(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2D:
    time_s: float
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class ClusterMeasurement:
    x_m: float
    y_m: float
    nearest_x_m: float
    nearest_y_m: float
    extent_m: float
    range_m: float
    point_count: int
    sensor_x_m: float
    sensor_y_m: float


@dataclass(frozen=True)
class TrackEstimate:
    valid: bool
    lifecycle: str
    reason: str
    state: np.ndarray
    covariance: np.ndarray
    hits: int
    misses: int
    age_s: float
    measurement_age_s: float
    confidence: float
    extent_m: float
    revision: int


class TrackCorridor:
    """Foundation-centerline corridor gate for physical vehicle centers."""

    def __init__(self, centerline_rows, half_width_m):
        points = [(float(row["x_m"]), float(row["y_m"])) for row in centerline_rows]
        self.geometry = ClosedTrackGeometry(points)
        self.sample_s = tuple(self.geometry.project(*point).s for point in points)
        self.left = tuple(float(row["w_tr_left_m"]) for row in centerline_rows)
        self.right = tuple(float(row["w_tr_right_m"]) for row in centerline_rows)
        self.half_width_m = float(half_width_m)
        if self.half_width_m <= 0.0:
            raise ValueError("vehicle half width must be positive")

    def contains(self, x_m, y_m):
        projection = self.geometry.project(x_m, y_m)
        left = interpolate_periodic_scalar(
            self.sample_s, self.left, projection.s, self.geometry.length,
        )
        right = interpolate_periodic_scalar(
            self.sample_s, self.right, projection.s, self.geometry.length,
        )
        return -right + self.half_width_m <= projection.d <= left - self.half_width_m


class PoseBuffer:
    def __init__(self, maximum_age_s=1.0):
        self.maximum_age_s = maximum_age_s
        self.poses = []

    def add(self, pose):
        if not all(math.isfinite(value) for value in vars(pose).values()):
            raise ValueError("pose values must be finite")
        if self.poses and pose.time_s <= self.poses[-1].time_s:
            return
        self.poses.append(pose)
        cutoff = pose.time_s - self.maximum_age_s
        while len(self.poses) > 2 and self.poses[1].time_s < cutoff:
            self.poses.pop(0)

    def interpolate(self, time_s, maximum_gap_s):
        if len(self.poses) < 2 or time_s < self.poses[0].time_s or time_s > self.poses[-1].time_s:
            return None
        times = [pose.time_s for pose in self.poses]
        upper = bisect.bisect_left(times, time_s)
        if upper == 0:
            return self.poses[0]
        if upper == len(self.poses):
            return self.poses[-1]
        low = self.poses[upper - 1]
        high = self.poses[upper]
        if high.time_s - low.time_s > maximum_gap_s:
            return None
        ratio = (time_s - low.time_s) / (high.time_s - low.time_s)
        return Pose2D(
            time_s,
            low.x_m + ratio * (high.x_m - low.x_m),
            low.y_m + ratio * (high.y_m - low.y_m),
            wrap_angle(low.yaw_rad + ratio * wrap_angle(high.yaw_rad - low.yaw_rad)),
        )

    def at_or_before(self, time_s, maximum_age_s):
        if not self.poses:
            return None
        times = [pose.time_s for pose in self.poses]
        index = bisect.bisect_right(times, time_s) - 1
        if index < 0 or time_s - self.poses[index].time_s > maximum_age_s:
            return None
        return self.poses[index]

    def candidates_at_or_before(self, time_s, maximum_age_s):
        return tuple(
            pose for pose in self.poses
            if 0.0 <= time_s - pose.time_s <= maximum_age_s
        )

    def bracket(self, time_s, maximum_gap_s):
        if len(self.poses) < 2:
            return None
        times = [pose.time_s for pose in self.poses]
        upper = bisect.bisect_left(times, time_s)
        if upper == 0 or upper == len(self.poses):
            return None
        low, high = self.poses[upper - 1], self.poses[upper]
        if time_s - low.time_s > maximum_gap_s or high.time_s - time_s > maximum_gap_s:
            return None
        return low, high


def extract_measurements(ranges, expected, angle_min, angle_increment, pose,
                         lidar_x_m, lidar_y_m, config, occupancy_map, corridor=None,
                         map_clearance_m=0.0, stage_marks=None):
    def mark(name):
        if stage_marks is not None:
            stage_marks[name] = (time.perf_counter_ns(), time.process_time_ns())

    def finish_early():
        for name in ("dynamic", "clustering", "candidates"):
            mark(name)

    observed = np.asarray(ranges, dtype=float)
    static = np.asarray(expected, dtype=float)
    if observed.shape != static.shape or observed.ndim != 1:
        raise ValueError("observed and expected scan shapes differ")
    finite = np.isfinite(observed) & (observed > 0.0)
    residual_indices = np.flatnonzero(finite & (
        observed < static - config["static_residual_tolerance_m"]
    ))
    mark("residual")
    sensor_x = pose.x_m + lidar_x_m * math.cos(pose.yaw_rad) - lidar_y_m * math.sin(pose.yaw_rad)
    sensor_y = pose.y_m + lidar_x_m * math.sin(pose.yaw_rad) + lidar_y_m * math.cos(pose.yaw_rad)
    if residual_indices.size == 0:
        finish_early()
        return (), 0
    angles = pose.yaw_rad + angle_min + residual_indices * angle_increment
    xs = sensor_x + observed[residual_indices] * np.cos(angles)
    ys = sensor_y + observed[residual_indices] * np.sin(angles)
    dynamic = np.array([
        not occupancy_map.occupied(float(x), float(y)) for x, y in zip(xs, ys)
    ])
    residual_indices, xs, ys = residual_indices[dynamic], xs[dynamic], ys[dynamic]
    mark("dynamic")
    if residual_indices.size == 0:
        mark("clustering"); mark("candidates")
        return (), 0
    clusters, start = [], 0
    for position in range(1, residual_indices.size):
        previous_range = observed[residual_indices[position - 1]]
        current_range = observed[residual_indices[position]]
        angular_gap = (residual_indices[position] - residual_indices[position - 1]) * angle_increment
        adaptive = config["breakpoint_noise_m"] + config["breakpoint_scale"] * (
            2.0 * min(previous_range, current_range) * math.sin(abs(angular_gap) * 0.5)
        )
        distance = math.hypot(xs[position] - xs[position - 1], ys[position] - ys[position - 1])
        if residual_indices[position] != residual_indices[position - 1] + 1 or distance > adaptive:
            clusters.append((start, position)); start = position
    clusters.append((start, residual_indices.size))
    mark("clustering")
    measurements = []
    for low, high in clusters:
        count = high - low
        if count < config["minimum_cluster_points"]:
            continue
        cluster_x, cluster_y = xs[low:high], ys[low:high]
        extent = math.hypot(cluster_x[-1] - cluster_x[0], cluster_y[-1] - cluster_y[0])
        centroid_x, centroid_y = float(np.mean(cluster_x)), float(np.mean(cluster_y))
        distance = math.hypot(centroid_x - sensor_x, centroid_y - sensor_y)
        if not (config["minimum_extent_m"] <= extent <= config["maximum_extent_m"]):
            continue
        if distance > config["maximum_measurement_range_m"] or occupancy_map.occupied(centroid_x, centroid_y):
            continue
        if corridor is not None and not corridor.contains(centroid_x, centroid_y):
            continue
        if map_clearance_m > 0.0 and any(
            occupancy_map.occupied(
                centroid_x + map_clearance_m * math.cos(angle),
                centroid_y + map_clearance_m * math.sin(angle),
            )
            for angle in np.linspace(0.0, 2.0 * math.pi, 16, endpoint=False)
        ):
            continue
        local = int(np.argmin((cluster_x - sensor_x) ** 2 + (cluster_y - sensor_y) ** 2))
        measurements.append(ClusterMeasurement(
            centroid_x, centroid_y, float(cluster_x[local]), float(cluster_y[local]),
            extent, distance, count, sensor_x, sensor_y,
        ))
    mark("candidates")
    return tuple(measurements), int(residual_indices.size)


class CvKalmanTracker:
    def __init__(self, config, geometry=None):
        self.c = config
        self.geometry = geometry
        self.x = None
        self.p = None
        self.last_time_s = None
        self.last_measurement_s = None
        self.first_time_s = None
        self.hits = self.misses = self.revision = 0
        self.extent_m = 0.0

    def _confirmation_along_speed(self):
        if self.geometry is None:
            return math.inf
        heading = self.geometry.project(self.x[0], self.x[1]).heading
        return self.x[2] * math.cos(heading) + self.x[3] * math.sin(heading)

    def _confirmation_valid(self):
        return self._confirmation_along_speed() >= self.c["minimum_confirmation_along_speed_mps"]

    def _reject_confirmation(self, time_s):
        if abs(self._confirmation_along_speed()) < self.c["minimum_confirmation_along_speed_mps"]:
            self.x[2:] = 0.0
            self.hits = self.c["minimum_confirmed_hits"] - 1
            return self.estimate(time_s, "STATIONARY_OBSERVED")
        self.x = self.p = self.last_time_s = self.last_measurement_s = self.first_time_s = None
        self.hits = self.misses = 0
        return self.estimate(time_s, "NON_FORWARD_TENTATIVE")

    def _predict(self, time_s):
        dt = time_s - self.last_time_s
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(
                f"tracker timestamp delta is invalid: current={time_s:.9f} "
                f"previous={self.last_time_s:.9f} dt={dt:.9f}"
            )
        f = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        q = self.c["process_acceleration_variance_m2_s4"]
        g = np.array([[0.5 * dt * dt, 0], [0, 0.5 * dt * dt], [dt, 0], [0, dt]])
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q * g @ g.T
        self.last_time_s = time_s

    def _measurement_position(self, measurement):
        return np.array([measurement.x_m, measurement.y_m])

    def _associate(self, measurements):
        h = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        r = self.c["measurement_variance_m2"] * np.eye(2)
        s = h @ self.p @ h.T + r
        inverse = np.linalg.inv(s)
        ranked = []
        for measurement in measurements:
            innovation = self._measurement_position(measurement) - h @ self.x
            ranked.append((float(innovation.T @ inverse @ innovation), measurement, innovation, s))
        if not ranked:
            return None
        best = min(ranked, key=lambda item: item[0])
        return best if best[0] <= self.c["association_mahalanobis_squared"] else None

    def update(self, measurements, time_s):
        if not math.isfinite(time_s):
            raise ValueError("tracker time must be finite")
        if self.x is None:
            if not measurements:
                return self.estimate(time_s, "NO_MEASUREMENT")
            measurement = min(measurements, key=lambda item: item.range_m)
            position = self._measurement_position(measurement)
            self.x = np.array([position[0], position[1], 0.0, 0.0])
            self.p = np.diag([0.04, 0.04, 4.0, 4.0])
            self.last_time_s = self.last_measurement_s = self.first_time_s = time_s
            self.hits, self.misses = 1, 0
            self.revision += 1
            self.extent_m = measurement.extent_m
            return self.estimate(time_s, "INITIALIZED")
        self._predict(time_s)
        association = self._associate(measurements)
        if association is None:
            if self.hits < self.c["minimum_confirmed_hits"]:
                self.x = self.p = self.last_time_s = self.last_measurement_s = self.first_time_s = None
                self.hits = self.misses = 0
                return self.estimate(time_s, "TENTATIVE_MISSED")
            self.misses += 1
            if self.misses > self.c["maximum_missed_scans"]:
                self.x = self.p = self.last_time_s = self.last_measurement_s = self.first_time_s = None
                self.hits = self.misses = 0
                return self.estimate(time_s, "TRACK_LOST")
            return self.estimate(time_s, "COASTING" if self.misses <= self.c["maximum_missed_scans"] else "LOST")
        _, measurement, innovation, s = association
        h = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x += k @ innovation
        self.p = (np.eye(4) - k @ h) @ self.p
        self.hits += 1; self.misses = 0; self.last_measurement_s = time_s
        self.extent_m = measurement.extent_m
        if self.hits == self.c["minimum_confirmed_hits"] and not self._confirmation_valid():
            return self._reject_confirmation(time_s)
        return self.estimate(time_s, "MEASUREMENT_ACCEPTED")

    def estimate(self, time_s, reason):
        if self.x is None:
            return TrackEstimate(False, "UNINITIALIZED", reason, np.zeros(4), np.eye(4) * 1e6, 0, 0, 0.0, math.inf, 0.0, 0.0, 0)
        confirmed = self.hits >= self.c["minimum_confirmed_hits"]
        valid = confirmed and self.misses <= self.c["maximum_missed_scans"]
        lifecycle = "TRACKED" if valid and self.misses == 0 else "COASTING" if valid else "TENTATIVE" if not confirmed else "LOST"
        confidence = max(0.0, min(1.0, self.hits / self.c["minimum_confirmed_hits"] * math.exp(-0.5 * self.misses)))
        return TrackEstimate(valid, lifecycle, reason, self.x.copy(), self.p.copy(), self.hits, self.misses,
                             time_s - self.first_time_s, time_s - self.last_measurement_s,
                             confidence, self.extent_m, self.revision)


class UkfExtentTracker(CvKalmanTracker):
    def _measurement_position(self, measurement):
        dx, dy = measurement.x_m - measurement.sensor_x_m, measurement.y_m - measurement.sensor_y_m
        distance = max(math.hypot(dx, dy), 1e-9)
        offset = self.c["visible_surface_offset_m"]
        return np.array([measurement.x_m + offset * dx / distance,
                         measurement.y_m + offset * dy / distance])

    def _associate(self, measurements):
        # The extent-aware inverse measurement is nonlinear in view direction;
        # sigma points propagate that correction into the common center-state.
        if not measurements:
            return None
        n, alpha, beta, kappa = 4, 0.3, 2.0, 0.0
        scale = alpha * alpha * (n + kappa)
        root = np.linalg.cholesky((n + scale - n) * self.p + 1e-12 * np.eye(n))
        sigma = [self.x]
        sigma += [self.x + root[:, i] for i in range(n)]
        sigma += [self.x - root[:, i] for i in range(n)]
        wm = np.full(2 * n + 1, 1.0 / (2.0 * scale)); wm[0] = (scale - n) / scale
        wc = wm.copy(); wc[0] += 1.0 - alpha * alpha + beta
        ranked = []
        for measurement in measurements:
            sensor = np.array([measurement.sensor_x_m, measurement.sensor_y_m])
            transformed = []
            for point in sigma:
                delta = point[:2] - sensor
                transformed.append(point[:2] - self.c["visible_surface_offset_m"] * delta / max(np.linalg.norm(delta), 1e-9))
            transformed = np.asarray(transformed)
            mean = np.sum(wm[:, None] * transformed, axis=0)
            covariance = self.c["measurement_variance_m2"] * np.eye(2)
            cross = np.zeros((4, 2))
            for weight, point, projected in zip(wc, sigma, transformed):
                covariance += weight * np.outer(projected - mean, projected - mean)
                cross += weight * np.outer(point - self.x, projected - mean)
            innovation = np.array([measurement.x_m, measurement.y_m]) - mean
            score = float(innovation.T @ np.linalg.inv(covariance) @ innovation)
            ranked.append((score, measurement, innovation, covariance, cross))
        best = min(ranked, key=lambda item: item[0])
        return best if best[0] <= self.c["association_mahalanobis_squared"] else None

    def update(self, measurements, time_s):
        if self.x is None:
            return super().update(measurements, time_s)
        self._predict(time_s)
        association = self._associate(measurements)
        if association is None:
            if self.hits < self.c["minimum_confirmed_hits"]:
                self.x = self.p = self.last_time_s = self.last_measurement_s = self.first_time_s = None
                self.hits = self.misses = 0
                return self.estimate(time_s, "TENTATIVE_MISSED")
            self.misses += 1
            if self.misses > self.c["maximum_missed_scans"]:
                self.x = self.p = self.last_time_s = self.last_measurement_s = self.first_time_s = None
                self.hits = self.misses = 0
                return self.estimate(time_s, "TRACK_LOST")
            return self.estimate(time_s, "COASTING" if self.misses <= self.c["maximum_missed_scans"] else "LOST")
        _, measurement, innovation, covariance, cross = association
        gain = cross @ np.linalg.inv(covariance)
        self.x += gain @ innovation
        self.p -= gain @ covariance @ gain.T
        self.p = 0.5 * (self.p + self.p.T) + 1e-12 * np.eye(4)
        self.hits += 1; self.misses = 0; self.last_measurement_s = time_s
        self.extent_m = measurement.extent_m
        if self.hits == self.c["minimum_confirmed_hits"] and not self._confirmation_valid():
            return self._reject_confirmation(time_s)
        return self.estimate(time_s, "MEASUREMENT_ACCEPTED")
