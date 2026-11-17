import csv
import hashlib
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from f1tenth_racing_geometry import (
    ClosedTrackGeometry,
    interpolate_periodic_angle,
    interpolate_periodic_scalar,
    wrap_angle,
)
from f1tenth_racing_sim.world.map import OccupancyMap

from .raceline import MinimumCurvatureOptimizer, OptimizerConfig, TrackCorridor, curvature_metrics


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_problem(config_path=None, environment_path_override=None):
    simulator_share = Path(get_package_share_directory("f1tenth_racing_sim"))
    package_share = Path(get_package_share_directory("f1tenth_racing_raceline"))
    environment_path = (Path(environment_path_override) if environment_path_override is not None
                        else simulator_share / "config/competition_environment.yaml")
    environment = yaml.safe_load(environment_path.read_text())
    def resolve_asset(rel):
        """Resolve registered map assets from a supplied official config.

        An environment override under <sim_share>/config must still resolve
        ``maps/`` relative to the package root, never the config directory.
        This fixes the old silent ICCAS-only fallback for Spielberg R-A.
        """
        asset = Path(rel)
        if asset.is_absolute():
            return asset.resolve(strict=True)
        candidates = [environment_path.parent / asset]
        if environment_path.parent.name == 'config':
            candidates.append(environment_path.parent.parent / asset)
        candidates.append(simulator_share / asset)
        for candidate in candidates:
            if candidate.is_file():
                return candidate.resolve(strict=True)
        raise FileNotFoundError('OFFLINE_RACELINE_ASSET_NOT_FOUND:' + str(rel))
    centerline_path = resolve_asset(environment["map"]["centerline_file"])
    map_path = resolve_asset(environment["map"]["image_file"])
    map_metadata_path = resolve_asset(environment["map"]["metadata_file"])
    optimizer_path = Path(config_path) if config_path else package_share / "config/r_a0_baseline.json"
    optimizer_data = json.loads(optimizer_path.read_text())
    # The geometry solver must not optimize an asset from the wrong map, even
    # when a caller supplied an override path that happens to exist.
    for path, declared in (
        (centerline_path, 'centerline_sha256'),
        (map_path, 'image_sha256'),
        (map_metadata_path, 'metadata_sha256'),
    ):
        if sha256(path) != environment['map'][declared]:
            raise ValueError('OFFLINE_RACELINE_MAP_SHA_MISMATCH:' + declared)

    with centerline_path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    centerline = [(float(row["x_m"]), float(row["y_m"])) for row in rows]
    left = [(float(row["left_x_m"]), float(row["left_y_m"])) for row in rows]
    right = [(float(row["right_x_m"]), float(row["right_y_m"])) for row in rows]
    left_widths = [float(row["w_tr_left_m"]) for row in rows]
    right_widths = [float(row["w_tr_right_m"]) for row in rows]
    footprint = environment["collision"]["footprint_relative_to_base_link"]
    footprint_radius = max(
        math.hypot(x, y)
        for x in (footprint["min_x_m"], footprint["max_x_m"])
        for y in (footprint["min_y_m"], footprint["max_y_m"])
    )
    corridor = TrackCorridor.create(centerline, left, right, left_widths, right_widths, footprint_radius)
    config = OptimizerConfig(
        tuple(optimizer_data["step_schedule_m"]),
        int(optimizer_data["maximum_sweeps_per_step"]),
        float(optimizer_data["smoothness_weight"]),
        float(optimizer_data["minimum_improvement_epsilon"]),
    )
    return {
        "corridor": corridor,
        "optimizer_config": config,
        "optimizer_data": optimizer_data,
        "environment": environment,
        "footprint": footprint,
        "occupancy_map": OccupancyMap(map_path, environment["map"]),
        "paths": {
            "centerline": centerline_path,
            "map": map_path,
            "map_metadata": map_metadata_path,
            "environment": environment_path,
            "optimizer": optimizer_path,
        },
        "hashes": {
            "centerline": sha256(centerline_path),
            "map": sha256(map_path),
            "map_metadata": sha256(map_metadata_path),
            "environment": sha256(environment_path),
            "optimizer": sha256(optimizer_path),
        },
    }


def _proper_intersection(a, b, c, d):
    def orientation(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return orientation(a, b, c) * orientation(a, b, d) < 0.0 and orientation(c, d, a) * orientation(c, d, b) < 0.0


def validate_line(line, problem):
    corridor = problem["corridor"]
    geometry = ClosedTrackGeometry(line.points)
    spacing = float(problem["optimizer_data"]["dense_validation_spacing_m"])
    samples = max(3, math.ceil(geometry.length / spacing))
    lower, upper = corridor.lower_offsets, corridor.upper_offsets
    footprint, occupancy = problem["footprint"], problem["occupancy_map"]
    minimum_margin, maximum_abs_d, wall_collisions, maximum_progress_step = math.inf, 0.0, 0, 0.0
    minimum_progress_step = math.inf
    previous_s = None

    for index in range(samples):
        sample = geometry.interpolate(index * geometry.length / samples)
        projection = corridor.geometry.project(sample.x, sample.y)
        low = interpolate_periodic_scalar(corridor.station_s, lower, projection.s, corridor.geometry.length)
        high = interpolate_periodic_scalar(corridor.station_s, upper, projection.s, corridor.geometry.length)
        minimum_margin = min(minimum_margin, projection.d - low, high - projection.d)
        maximum_abs_d = max(maximum_abs_d, abs(projection.d))
        if previous_s is not None:
            progress_step = corridor.geometry.signed_delta_s(previous_s, projection.s)
            minimum_progress_step = min(minimum_progress_step, progress_step)
            maximum_progress_step = max(maximum_progress_step, progress_step)
        previous_s = projection.s
        cosine, sine = math.cos(sample.heading), math.sin(sample.heading)
        for local_x in (footprint["min_x_m"], footprint["max_x_m"]):
            for local_y in (footprint["min_y_m"], footprint["max_y_m"]):
                x = sample.x + local_x * cosine - local_y * sine
                y = sample.y + local_x * sine + local_y * cosine
                wall_collisions += int(occupancy.occupied(x, y))

    points = line.points
    intersections = []
    for first in range(len(points)):
        a, b = points[first], points[(first + 1) % len(points)]
        for second in range(first + 1, len(points)):
            if second in (first, (first + 1) % len(points)) or first == (second + 1) % len(points):
                continue
            c, d = points[second], points[(second + 1) % len(points)]
            if _proper_intersection(a, b, c, d):
                intersections.append((first, second))

    maximum_centerline_segment = max(
        math.hypot(
            corridor.centerline[(index + 1) % len(corridor.centerline)][0] - point[0],
            corridor.centerline[(index + 1) % len(corridor.centerline)][1] - point[1],
        )
        for index, point in enumerate(corridor.centerline)
    )
    invalid = any((
        minimum_margin < -1.0e-9,
        wall_collisions,
        intersections,
        minimum_progress_step < -1.0e-9,
        maximum_progress_step > maximum_centerline_segment + spacing,
    ))
    if invalid:
        raise RuntimeError(
            f"invalid racing line: margin={minimum_margin}, walls={wall_collisions}, "
            f"intersections={intersections}, progress_step=[{minimum_progress_step}, {maximum_progress_step}]"
        )
    seam_start, seam_end = geometry.interpolate(0.0), geometry.interpolate(geometry.length)
    seam_heading_start = interpolate_periodic_angle(
        line.station_s, line.heading, 0.0, corridor.geometry.length
    )
    seam_heading_end = interpolate_periodic_angle(
        line.station_s, line.heading, corridor.geometry.length, corridor.geometry.length
    )
    seam_curvature_start = interpolate_periodic_scalar(
        line.station_s, line.curvature, 0.0, corridor.geometry.length
    )
    seam_curvature_end = interpolate_periodic_scalar(
        line.station_s, line.curvature, corridor.geometry.length, corridor.geometry.length
    )
    return {
        "dense_sample_count": samples,
        "minimum_footprint_adjusted_corridor_margin_m": minimum_margin,
        "maximum_absolute_projected_d_m": maximum_abs_d,
        "wall_collision_corner_hits": wall_collisions,
        "self_intersection_count": len(intersections),
        "maximum_projected_progress_step_m": maximum_progress_step,
        "minimum_projected_progress_step_m": minimum_progress_step,
        "seam_position_error_m": math.hypot(seam_end.x - seam_start.x, seam_end.y - seam_start.y),
        "seam_heading_error_rad": abs(wrap_angle(seam_heading_end - seam_heading_start)),
        "seam_curvature_error_1_m": abs(seam_curvature_end - seam_curvature_start),
    }


def solve_problem(problem):
    optimizer = MinimumCurvatureOptimizer(problem["corridor"], problem["optimizer_config"])
    line = optimizer.solve()
    validation = validate_line(line, problem)
    centerline_metrics = curvature_metrics(problem["corridor"].centerline)
    line_metrics = curvature_metrics(line.points)
    return line, validation, centerline_metrics, line_metrics
