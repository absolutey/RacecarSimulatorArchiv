import csv
import hashlib
import json
from pathlib import Path


def _format(value):
    return f"{float(value):.12f}"


def write_artifacts(output_dir, line, problem, validation, centerline_metrics, line_metrics):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    track = problem["environment"]["environment_identity"]["active_track"]
    if not track or not all(c.isalnum() or c in "_-" for c in track):
        raise ValueError("invalid R-A0 track for filename")
    csv_path = output_dir / f"{track}_r_a0_baseline.csv"
    metadata_path = output_dir / f"{track}_r_a0_baseline.json"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("s_m", "x_m", "y_m", "heading_rad", "curvature_1_m", "d_m"))
        for row in zip(line.station_s, line.x, line.y, line.heading, line.curvature, line.lateral_offset):
            writer.writerow(tuple(_format(value) for value in row))
    output_hash = hashlib.sha256(csv_path.read_bytes()).hexdigest()
    metadata = {
        "artifact_version": problem["optimizer_data"]["artifact_version"],
        "centerline_sha256": problem["hashes"]["centerline"],
        "competition_environment_sha256": problem["hashes"]["environment"],
        "generator_config_sha256": problem["hashes"]["optimizer"],
        "map_sha256": problem["hashes"]["map"],
        "map_metadata_sha256": problem["hashes"]["map_metadata"],
        "output_csv_sha256": output_hash,
        "sample_count": len(line.x),
        "source_track": track,
        "solver": {
            "accepted_moves": line.accepted_moves,
            "composite_objective": line.composite_objective,
            "config": problem["optimizer_data"],
            "iterations": line.iterations,
            "name": "deterministic_projected_coordinate_descent",
            "termination_reason": line.termination_reason,
        },
        "metrics": {
            "centerline": {key: value for key, value in centerline_metrics.items() if key != "curvature"},
            "optimized": {key: value for key, value in line_metrics.items() if key != "curvature"},
            "objective_delta": line.final_objective - line.initial_objective,
            "maximum_absolute_lateral_offset_m": max(map(abs, line.lateral_offset)),
            **validation,
        },
        "valid": True,
    }
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return csv_path, metadata_path, metadata


def write_svg(path, corridor, line):
    path = Path(path)
    all_points = corridor.left_boundary + corridor.right_boundary + corridor.centerline + line.points
    xs, ys = [point[0] for point in all_points], [point[1] for point in all_points]
    padding, width, height = 20.0, 900.0, 700.0
    scale = min((width - 2 * padding) / (max(xs) - min(xs)), (height - 2 * padding) / (max(ys) - min(ys)))

    def pixels(points):
        closed = points + (points[0],)
        return " ".join(f"{padding + (x - min(xs)) * scale:.2f},{height - padding - (y - min(ys)) * scale:.2f}" for x, y in closed)

    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{int(width)}" height="{int(height)}" viewBox="0 0 {int(width)} {int(height)}">\n'
        '<rect width="100%" height="100%" fill="white"/>\n'
        f'<polyline points="{pixels(corridor.left_boundary)}" fill="none" stroke="#555" stroke-width="2"/>\n'
        f'<polyline points="{pixels(corridor.right_boundary)}" fill="none" stroke="#555" stroke-width="2"/>\n'
        f'<polyline points="{pixels(corridor.centerline)}" fill="none" stroke="#4b7bec" stroke-width="1.5" stroke-dasharray="5,4"/>\n'
        f'<polyline points="{pixels(line.points)}" fill="none" stroke="#d62728" stroke-width="2.5"/>\n'
        '</svg>\n'
    )
    path.write_text(svg)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_speed_artifacts(output_dir, profile, problem, dense_validation, comparator):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    track = problem["r_a0_metadata"]["source_track"]
    if not track or not all(c.isalnum() or c in "_-" for c in track):
        raise ValueError("invalid R-A1 track for filename")
    csv_path = output_dir / f"{track}_r_a1_speed_profile.csv"
    metadata_path = output_dir / f"{track}_r_a1_speed_profile.json"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow((
            "s_m",
            "target_speed_m_s",
            "implied_acceleration_m_s2",
            "curvature_speed_ceiling_m_s",
            "global_speed_ceiling_m_s",
            "limiting_constraint",
        ))
        for row in zip(
            profile.station_s,
            profile.target_speed,
            profile.implied_longitudinal_acceleration,
            profile.curvature_speed_ceiling,
            profile.limiting_constraint,
        ):
            writer.writerow((*(_format(value) for value in row[:-1]), _format(profile.global_speed_ceiling), row[-1]))
    output_hash = hashlib.sha256(csv_path.read_bytes()).hexdigest()
    counts = {label: profile.limiting_constraint.count(label) for label in sorted(set(profile.limiting_constraint))}
    metadata = {
        "artifact_version": problem["optimizer_data"]["artifact_version"],
        "foundation_authority": {
            "classification": {
                "acceleration": "DIRECT_CONFIG_AND_CODE",
                "braking": "DIRECT_CONFIG_AND_CODE",
                "jerk": "DIRECT_CONFIG_AND_CODE_DEFERRED_TO_R_B",
                "lateral_acceleration": "DERIVED_FROM_FROZEN_MODEL",
                "mass": "DIRECT_CONFIG",
                "maximum_speed": "DIRECT_CONFIG_AND_CODE",
            },
            "hashes": {
                "competition_environment": problem["hashes"]["environment"],
                "pacejka_source": problem["hashes"]["pacejka"],
                "vehicle_dynamics_source": problem["hashes"]["vehicle"],
            },
            "limits": problem["limits"],
        },
        "generator_config_sha256": problem["hashes"]["optimizer"],
        "jerk_feasibility": "DEFERRED_TO_R-B",
        "output_csv_sha256": output_hash,
        "profile": {
            "convergence_delta_m_s": profile.convergence_delta,
            "iterations": profile.iterations,
            "limiting_constraint_counts": counts,
            "maximum_speed_m_s": max(profile.target_speed),
            "mean_traversal_speed_m_s": sum(profile.segment_length) / profile.predicted_lap_time,
            "mean_sample_speed_m_s": sum(profile.target_speed) / len(profile.target_speed),
            "minimum_speed_m_s": min(profile.target_speed),
            "offline_envelope_predicted_lap_time_s": profile.predicted_lap_time,
            "path_length_m": sum(profile.segment_length),
            "sample_count": len(profile.station_s),
        },
        "dense_validation": dense_validation,
        "centerline_comparator": comparator,
        "source_r_a0": {
            "csv_sha256": problem["hashes"]["r_a0_csv"],
            "metadata_sha256": problem["hashes"]["r_a0_metadata"],
            "centerline_sha256": problem["r_a0_metadata"]["centerline_sha256"],
            "map_sha256": problem["r_a0_metadata"]["map_sha256"],
            "map_metadata_sha256": problem["r_a0_metadata"]["map_metadata_sha256"],
        },
        "valid": True,
    }
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return csv_path, metadata_path, metadata


def write_speed_svg(path, profile):
    path = Path(path)
    width, height, padding = 900.0, 420.0, 30.0
    period = profile.station_period
    maximum = max(profile.global_speed_ceiling, max(profile.curvature_speed_ceiling))

    def pixels(values):
        samples = list(zip(profile.station_s, values)) + [(period, values[0])]
        return " ".join(
            f"{padding + s / period * (width - 2 * padding):.2f},"
            f"{height - padding - value / maximum * (height - 2 * padding):.2f}"
            for s, value in samples
        )

    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{int(width)}" height="{int(height)}" viewBox="0 0 {int(width)} {int(height)}">\n'
        '<rect width="100%" height="100%" fill="white"/>\n'
        f'<polyline points="{pixels(profile.curvature_speed_ceiling)}" fill="none" stroke="#999" stroke-width="1.5"/>\n'
        f'<polyline points="{pixels(profile.target_speed)}" fill="none" stroke="#d62728" stroke-width="2.5"/>\n'
        '</svg>\n'
    )
    path.write_text(svg)
    return hashlib.sha256(path.read_bytes()).hexdigest()
