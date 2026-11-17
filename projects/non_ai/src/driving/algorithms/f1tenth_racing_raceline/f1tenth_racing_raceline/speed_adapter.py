import csv
import hashlib
import inspect
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from f1tenth_racing_geometry import interpolate_periodic_scalar
from f1tenth_racing_sim.physics import pacejka as pacejka_module
from f1tenth_racing_sim.physics import vehicle as vehicle_module
from f1tenth_racing_sim.physics.pacejka import lateral_force

from .raceline import curvature_metrics
from .speed_profile import SpeedEnvelopeConfig, generate_speed_profile


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def maximum_tire_force(tire):
    b = float(tire["pacejka_b"])
    c = float(tire["pacejka_c"])
    d = float(tire["pacejka_d_n"])
    if any(not math.isfinite(value) or value <= 0.0 for value in (b, c, d)):
        raise ValueError("Pacejka B/C/D must be finite and positive")
    peak_slip = math.tan(math.pi / (2.0 * c)) / b if c >= 1.0 else math.inf
    peak_force = d if c >= 1.0 else d * math.sin(c * math.pi / 2.0)
    if math.isfinite(peak_slip) and not math.isclose(
        lateral_force(tire, peak_slip), peak_force, rel_tol=1.0e-12, abs_tol=1.0e-12
    ):
        raise RuntimeError("Foundation Pacejka peak-force derivation mismatch")
    return peak_force, peak_slip


def load_csv(path):
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = ("s_m", "x_m", "y_m", "curvature_1_m")
    if not rows or any(field not in rows[0] for field in required):
        raise ValueError(f"missing required racing-line fields: {required}")
    return {
        "station_s": [float(row["s_m"]) for row in rows],
        "points": [(float(row["x_m"]), float(row["y_m"])) for row in rows],
        "curvature": [float(row["curvature_1_m"]) for row in rows],
    }


def load_speed_problem(config_path=None, *, r_a0_csv_path=None, r_a0_metadata_path=None, environment_path_override=None):
    raceline_share = Path(get_package_share_directory("f1tenth_racing_raceline"))
    simulator_share = Path(get_package_share_directory("f1tenth_racing_sim"))
    r_a0_csv = Path(r_a0_csv_path) if r_a0_csv_path is not None else raceline_share / "data/iccas2025_r_a0_baseline.csv"
    r_a0_metadata = Path(r_a0_metadata_path) if r_a0_metadata_path is not None else raceline_share / "data/iccas2025_r_a0_baseline.json"
    environment_path = (Path(environment_path_override) if environment_path_override is not None
                        else simulator_share / "config/competition_environment.yaml")
    vehicle_path = Path(inspect.getsourcefile(vehicle_module))
    pacejka_path = Path(inspect.getsourcefile(pacejka_module))
    optimizer_path = Path(config_path) if config_path else raceline_share / "config/r_a1_speed_profile.json"
    environment = yaml.safe_load(environment_path.read_text())
    optimizer_data = json.loads(optimizer_path.read_text())
    r_a0_data = json.loads(r_a0_metadata.read_text())
    if sha256(r_a0_csv) != r_a0_data["output_csv_sha256"]:
        raise RuntimeError("R-A0 artifact hash does not match its metadata")
    if r_a0_data["source_track"] != environment["environment_identity"]["active_track"]:
        raise ValueError("R-A0 artifact track mismatch")
    if (r_a0_data.get("map_sha256") != environment["map"]["image_sha256"] or
        r_a0_data.get("centerline_sha256") != environment["map"]["centerline_sha256"]):
        raise ValueError("R-A0 map/centerline fingerprints mismatch")

    front_force, front_peak_slip = maximum_tire_force(environment["dynamics"]["front_tire"])
    rear_force, rear_peak_slip = maximum_tire_force(environment["dynamics"]["rear_tire"])
    mass = float(environment["dynamics"]["mass_kg"])
    lateral_acceleration = (front_force + rear_force) / mass
    limits = environment["dynamics"]["limits"]
    config = SpeedEnvelopeConfig(
        float(optimizer_data["convergence_tolerance_m_s"]),
        float(optimizer_data["curvature_epsilon_1_m"]),
        int(optimizer_data["maximum_iterations"]),
    )
    return {
        "r_a0": load_csv(r_a0_csv),
        "r_a0_metadata": r_a0_data,
        "environment": environment,
        "config": config,
        "optimizer_data": optimizer_data,
        "limits": {
            "mass_kg": mass,
            "global_speed_m_s": float(limits["speed_abs_max_m_s"]),
            "acceleration_m_s2": float(limits["acceleration_max_m_s2"]),
            "braking_m_s2": float(limits["deceleration_max_m_s2"]),
            "jerk_m_s3": float(limits["jerk_abs_max_m_s3"]),
            "front_lateral_force_n": front_force,
            "rear_lateral_force_n": rear_force,
            "front_peak_slip_rad": front_peak_slip,
            "rear_peak_slip_rad": rear_peak_slip,
            "lateral_acceleration_m_s2": lateral_acceleration,
        },
        "paths": {
            "r_a0_csv": r_a0_csv,
            "r_a0_metadata": r_a0_metadata,
            "environment": environment_path,
            "vehicle": vehicle_path,
            "pacejka": pacejka_path,
            "optimizer": optimizer_path,
        },
        "hashes": {
            "r_a0_csv": sha256(r_a0_csv),
            "r_a0_metadata": sha256(r_a0_metadata),
            "environment": sha256(environment_path),
            "vehicle": sha256(vehicle_path),
            "pacejka": sha256(pacejka_path),
            "optimizer": sha256(optimizer_path),
        },
    }


def solve_speed_problem(problem, line_data=None):
    data = line_data or problem["r_a0"]
    limits = problem["limits"]
    return generate_speed_profile(
        data["station_s"],
        data["points"],
        data["curvature"],
        limits["global_speed_m_s"],
        limits["acceleration_m_s2"],
        limits["braking_m_s2"],
        limits["lateral_acceleration_m_s2"],
        problem["config"],
        problem["r_a0_metadata"]["metrics"]["centerline"]["length"]
        if line_data is None else line_data.get("station_period"),
    )


def validate_dense(profile, station_period, samples_per_segment):
    count = len(profile.station_s)
    maximum_lateral_excess = maximum_global_excess = 0.0
    for index in range(count):
        following = (index + 1) % count
        start_s = profile.station_s[index]
        end_s = profile.station_s[following] if following else station_period
        for dense_index in range(samples_per_segment):
            ratio = dense_index / samples_per_segment
            query_s = start_s + ratio * (end_s - start_s)
            speed_squared = interpolate_periodic_scalar(
                profile.station_s,
                tuple(speed * speed for speed in profile.target_speed),
                query_s,
                station_period,
            )
            curvature = interpolate_periodic_scalar(
                profile.station_s, profile.curvature, query_s, station_period
            )
            maximum_global_excess = max(
                maximum_global_excess,
                math.sqrt(max(0.0, speed_squared)) - profile.global_speed_ceiling,
            )
            maximum_lateral_excess = max(
                maximum_lateral_excess,
                speed_squared * abs(curvature) - profile.lateral_acceleration_ceiling,
            )
    speed_squared = tuple(speed * speed for speed in profile.target_speed)
    seam_start = interpolate_periodic_scalar(
        profile.station_s, speed_squared, 0.0, station_period
    )
    seam_end = interpolate_periodic_scalar(
        profile.station_s, speed_squared, station_period, station_period
    )
    maximum_forward_excess = max(
        0.0,
        max(profile.implied_longitudinal_acceleration) - profile.acceleration_limit,
    )
    maximum_braking_excess = max(
        0.0,
        -min(profile.implied_longitudinal_acceleration) - profile.braking_limit,
    )
    validation = {
        "dense_sample_count": count * samples_per_segment,
        "maximum_global_speed_excess_m_s": maximum_global_excess,
        "maximum_lateral_acceleration_excess_m_s2": maximum_lateral_excess,
        "maximum_forward_acceleration_excess_m_s2": maximum_forward_excess,
        "maximum_braking_excess_m_s2": maximum_braking_excess,
        "seam_speed_error_m_s": abs(math.sqrt(seam_end) - math.sqrt(seam_start)),
    }
    if any(value > 1.0e-9 for key, value in validation.items() if key != "dense_sample_count"):
        raise RuntimeError(f"speed profile failed dense validation: {validation}")
    return validation


def centerline_comparator(problem):
    simulator_share = Path(get_package_share_directory("f1tenth_racing_sim"))
    envpath = problem["paths"]["environment"]
    candidate = Path(problem["environment"]["map"]["centerline_file"])
    centerline_path = (candidate if candidate.is_absolute() else
        (simulator_share / candidate if (simulator_share / candidate).is_file()
         else envpath.parent.parent / candidate))
    with centerline_path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    line = {
        "points": [(float(row["x_m"]), float(row["y_m"])) for row in rows],
    }
    metrics = curvature_metrics(line["points"])
    line["curvature"] = list(map(float, metrics["curvature"]))
    line["station_s"] = []
    distance = 0.0
    for index, point in enumerate(line["points"]):
        line["station_s"].append(distance)
        following = line["points"][(index + 1) % len(line["points"])]
        distance += math.hypot(following[0] - point[0], following[1] - point[1])
    profile = solve_speed_problem(problem, line)
    return profile, sha256(centerline_path)
