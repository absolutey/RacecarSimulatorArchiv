from dataclasses import dataclass
import math

import numpy as np

from f1tenth_racing_geometry import ClosedTrackGeometry


@dataclass(frozen=True)
class OptimizerConfig:
    step_schedule_m: tuple
    maximum_sweeps_per_step: int
    smoothness_weight: float
    minimum_improvement_epsilon: float

    def __post_init__(self):
        if not self.step_schedule_m or any(step <= 0.0 for step in self.step_schedule_m):
            raise ValueError("step schedule must be positive")
        if any(a <= b for a, b in zip(self.step_schedule_m, self.step_schedule_m[1:])):
            raise ValueError("step schedule must be strictly decreasing")
        if self.maximum_sweeps_per_step < 1 or self.smoothness_weight < 0.0:
            raise ValueError("invalid optimizer configuration")


@dataclass(frozen=True)
class TrackCorridor:
    centerline: tuple
    left_boundary: tuple
    right_boundary: tuple
    left_widths: tuple
    right_widths: tuple
    station_s: tuple
    normals: tuple
    lower_offsets: tuple
    upper_offsets: tuple
    footprint_radius: float
    geometry: ClosedTrackGeometry

    @classmethod
    def create(cls, centerline, left_boundary, right_boundary, left_widths, right_widths, footprint_radius):
        centerline = tuple(tuple(map(float, point)) for point in centerline)
        left_boundary = tuple(tuple(map(float, point)) for point in left_boundary)
        right_boundary = tuple(tuple(map(float, point)) for point in right_boundary)
        left_widths = tuple(map(float, left_widths))
        right_widths = tuple(map(float, right_widths))
        footprint_radius = float(footprint_radius)
        size = len(centerline)
        if size < 3 or any(len(values) != size for values in (left_boundary, right_boundary, left_widths, right_widths)):
            raise ValueError("corridor fields must have equal length and at least three points")
        if not math.isfinite(footprint_radius) or footprint_radius < 0.0:
            raise ValueError("footprint radius must be finite and nonnegative")

        geometry = ClosedTrackGeometry(centerline)
        station_s = []
        distance = 0.0
        for index, point in enumerate(centerline):
            station_s.append(distance)
            next_point = centerline[(index + 1) % size]
            distance += math.hypot(next_point[0] - point[0], next_point[1] - point[1])

        normals = []
        for center, left, right in zip(centerline, left_boundary, right_boundary):
            to_left = np.asarray(left) - center
            from_right = np.asarray(center) - right
            left_norm, right_norm = np.linalg.norm(to_left), np.linalg.norm(from_right)
            if left_norm == 0.0 or right_norm == 0.0:
                raise ValueError("boundary point coincides with centerline")
            normal = to_left / left_norm + from_right / right_norm
            normal_norm = np.linalg.norm(normal)
            if normal_norm == 0.0:
                raise ValueError("left/right boundary directions disagree")
            normals.append(tuple(normal / normal_norm))

        lower = tuple(-width + footprint_radius for width in right_widths)
        upper = tuple(width - footprint_radius for width in left_widths)
        if any(not math.isfinite(value) for value in left_widths + right_widths + lower + upper):
            raise ValueError("corridor widths must be finite")
        if any(low > high for low, high in zip(lower, upper)):
            raise ValueError("footprint leaves no legal corridor")
        return cls(
            centerline, left_boundary, right_boundary, left_widths, right_widths,
            tuple(station_s), tuple(normals), lower, upper, footprint_radius, geometry,
        )

    def points_for_offsets(self, offsets):
        offsets = np.asarray(offsets, dtype=float)
        if offsets.shape != (len(self.centerline),) or not np.all(np.isfinite(offsets)):
            raise ValueError("one finite lateral offset is required per centerline point")
        lower, upper = np.asarray(self.lower_offsets), np.asarray(self.upper_offsets)
        if np.any(offsets < lower) or np.any(offsets > upper):
            raise ValueError("lateral offset outside footprint-adjusted corridor")
        return np.asarray(self.centerline) + offsets[:, None] * np.asarray(self.normals)


@dataclass(frozen=True)
class BaselineRacingLine:
    station_s: tuple
    x: tuple
    y: tuple
    heading: tuple
    curvature: tuple
    lateral_offset: tuple
    initial_objective: float
    final_objective: float
    composite_objective: float
    line_length: float
    iterations: int
    accepted_moves: int
    termination_reason: str

    @property
    def points(self):
        return tuple(zip(self.x, self.y))


def curvature_metrics(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
        raise ValueError("at least three finite XY points are required")
    previous, following = np.roll(points, 1, axis=0), np.roll(points, -1, axis=0)
    incoming, outgoing = points - previous, following - points
    incoming_length = np.linalg.norm(incoming, axis=1)
    outgoing_length = np.linalg.norm(outgoing, axis=1)
    chord_length = np.linalg.norm(following - previous, axis=1)
    if np.any(incoming_length <= 1.0e-12) or np.any(outgoing_length <= 1.0e-12) or np.any(chord_length <= 1.0e-12):
        raise ValueError("degenerate racing-line geometry")
    cross = incoming[:, 0] * outgoing[:, 1] - incoming[:, 1] * outgoing[:, 0]
    curvature = 2.0 * cross / (incoming_length * outgoing_length * chord_length)
    weights = 0.5 * (incoming_length + outgoing_length)
    objective = float(np.sum(curvature * curvature * weights))
    return {
        "curvature": curvature,
        "objective": objective,
        "length": float(np.sum(incoming_length)),
        "maximum_absolute_curvature": float(np.max(np.abs(curvature))),
        "rms_curvature": math.sqrt(objective / float(np.sum(weights))),
    }


def _headings(points):
    points = np.asarray(points)
    tangent = np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)
    return np.arctan2(tangent[:, 1], tangent[:, 0])


class MinimumCurvatureOptimizer:
    def __init__(self, corridor, config):
        self.corridor = corridor
        self.config = config

    def _score(self, offsets):
        metrics = curvature_metrics(self.corridor.points_for_offsets(offsets))
        second_difference = np.roll(offsets, -1) - 2.0 * offsets + np.roll(offsets, 1)
        return metrics["objective"] + self.config.smoothness_weight * float(second_difference @ second_difference)

    def solve(self):
        count = len(self.corridor.centerline)
        offsets = np.zeros(count)
        lower, upper = np.asarray(self.corridor.lower_offsets), np.asarray(self.corridor.upper_offsets)
        initial = curvature_metrics(self.corridor.points_for_offsets(offsets))
        score = self._score(offsets)
        sweeps = accepted_moves = 0
        final_step_moved = False

        for step_index, step in enumerate(self.config.step_schedule_m):
            step_moved = False
            for _ in range(self.config.maximum_sweeps_per_step):
                sweep_moved = False
                for index in range(count):
                    best_score, best_offset = score, offsets[index]
                    for candidate in (max(lower[index], offsets[index] - step), min(upper[index], offsets[index] + step)):
                        if candidate == offsets[index]:
                            continue
                        trial = offsets.copy()
                        trial[index] = candidate
                        trial_score = self._score(trial)
                        if trial_score < best_score - self.config.minimum_improvement_epsilon:
                            best_score, best_offset = trial_score, candidate
                    if best_offset != offsets[index]:
                        offsets[index], score = best_offset, best_score
                        accepted_moves += 1
                        sweep_moved = step_moved = True
                sweeps += 1
                if not sweep_moved:
                    break
            if step_index == len(self.config.step_schedule_m) - 1:
                final_step_moved = step_moved

        points = self.corridor.points_for_offsets(offsets)
        final = curvature_metrics(points)
        if final["objective"] > initial["objective"] + 1.0e-12:
            raise RuntimeError("optimizer worsened curvature objective")
        headings = _headings(points)
        return BaselineRacingLine(
            self.corridor.station_s,
            tuple(map(float, points[:, 0])), tuple(map(float, points[:, 1])),
            tuple(map(float, headings)), tuple(map(float, final["curvature"])),
            tuple(map(float, offsets)), initial["objective"], final["objective"],
            score, final["length"], sweeps, accepted_moves,
            "STEP_SCHEDULE_EXHAUSTED" if final_step_moved else "NO_MOVE_AT_FINAL_STEP",
        )
