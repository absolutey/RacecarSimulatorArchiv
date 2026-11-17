import hashlib
from dataclasses import replace
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from .foundation_adapter import load_problem
from .raceline import BaselineRacingLine
from .scp import ScpConfig, solve_scp
from .speed_adapter import load_speed_problem, validate_dense
from .trajectory import (RacingTrajectory, TrajectorySample, load_racing_trajectory,
                         write_racing_trajectory, validate_racing_trajectory)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_scp_problem(config_path=None, *, baseline_csv=None, baseline_metadata=None,
                     environment_path=None, r_a0_csv=None, r_a0_metadata=None):
    share = Path(get_package_share_directory("f1tenth_racing_raceline"))
    config_path = Path(config_path) if config_path else share / "config/r_a3_scp.json"
    config_data = json.loads(config_path.read_text())
    baseline = (load_racing_trajectory(baseline_csv, baseline_metadata)
                if baseline_csv is not None and baseline_metadata is not None
                else load_racing_trajectory())
    if (baseline_csv is None) != (baseline_metadata is None):
        raise ValueError("baseline CSV and metadata must be provided together")
    line_problem = load_problem(environment_path_override=environment_path)
    speed_problem = load_speed_problem(r_a0_csv_path=r_a0_csv,
        r_a0_metadata_path=r_a0_metadata, environment_path_override=environment_path)
    if baseline.metadata.get("track_id") != line_problem["environment"]["environment_identity"]["active_track"]:
        raise ValueError("SCP input baseline track identity mismatch")
    if baseline.metadata.get("source", {}).get("map_image_sha256") != line_problem["environment"]["map"]["image_sha256"]:
        raise ValueError("SCP baseline map SHA mismatch")
    samples = baseline.samples
    initial_line = BaselineRacingLine(
        tuple(sample.s_m for sample in samples),
        tuple(sample.x_m for sample in samples), tuple(sample.y_m for sample in samples),
        tuple(sample.heading_rad for sample in samples),
        tuple(sample.curvature_per_m for sample in samples),
        tuple(sample.lateral_offset_m for sample in samples),
        0.0, 0.0, 0.0, baseline.metadata["path_length_m"], 0, 0, "R_A0_WARM_START",
    )
    fields = {key: value for key, value in config_data.items() if key != "artifact_version"}
    return {
        "baseline": baseline,
        "initial_line": initial_line,
        "line_problem": line_problem,
        "speed_problem": speed_problem,
        "config": ScpConfig(**fields),
        "config_data": config_data,
        "config_path": config_path,
    }


def solve_scp_problem(problem):
    result = solve_scp(
        problem["line_problem"]["corridor"], problem["initial_line"],
        problem["speed_problem"], problem["line_problem"], problem["config"],
    )
    line, profile = result.line, result.speed_profile
    if problem["config"].use_joint_speed_profile and result.accepted_steps < 1:
        raise ValueError("joint speed adoption has no feasible accepted step; do not publish; "
                         + json.dumps(result.diagnostics, sort_keys=True))
    # Geometry validation uses corridor projection station; exported trajectory
    # uses actual arc length of the newly optimized spatial path. Keeping the
    # original station period after changing the path silently corrupts
    # interpolation/progress/lease at the seam.
    if problem["config"].use_joint_speed_profile:
        export_stations = []
        station = 0.0
        for length in profile.segment_length:
            export_stations.append(station)
            station += float(length)
        actual_station_period = station
    else:
        export_stations = line.station_s
        actual_station_period = baseline_period = problem["baseline"].station_period_m
    if problem["config"].use_joint_speed_profile:
        physically_stationed_profile = replace(
            profile, station_s=tuple(export_stations), station_period=actual_station_period)
        physical_dense_validation = validate_dense(
            physically_stationed_profile, actual_station_period,
            problem["speed_problem"]["optimizer_data"]["dense_samples_per_segment"])
    else:
        physical_dense_validation = result.dense_speed_validation
    samples = tuple(TrajectorySample(
        index, export_stations[index], line.x[index], line.y[index], line.heading[index],
        line.curvature[index], line.lateral_offset[index], profile.target_speed[index],
        profile.implied_longitudinal_acceleration[index], profile.segment_length[index],
    ) for index in range(len(line.x)))
    baseline = problem["baseline"]
    metadata = dict(baseline.metadata)
    metadata.pop("trajectory_id", None)
    metadata.pop("csv_sha256", None)
    metadata.update({
        "generation_method": "R_A3_SCP_JOINT_PATH_SPEED",
        "generator_code_sha256": sha256(Path(__file__).with_name("scp.py")),
        "geometry_qualification_status": "VERIFIED",
        "offline_envelope_predicted_lap_time_s": result.final_predicted_time_s,
        "path_length_m": sum(profile.segment_length),
        "station_period_m": actual_station_period,
        "qualification_state": "OFFLINE_PRIMARY_CANDIDATE_QUALIFIED",
        "runtime_tracking_qualified": False,
        "sample_count": len(samples),
        "speed_envelope_qualification_status": "VERIFIED",
        "trajectory_tier": "PRIMARY_CANDIDATE",
    })
    if problem["config"].use_joint_speed_profile:
        metadata["generation_method"] = "R_A3_SPARSE_SCP_TRUE_JOINT_EXPERIMENTAL"
        metadata["qualification_state"] = "OFFLINE_EXPERIMENTAL_UNQUALIFIED"
        metadata["geometry_qualification_status"] = "PREFLIGHT_PENDING"
        metadata["speed_envelope_qualification_status"] = "PREFLIGHT_PENDING"
        metadata["trajectory_tier"] = "EXPERIMENTAL_UNQUALIFIED"
    metadata["source"] = dict(metadata["source"], **{
        "r_a2_baseline_trajectory_id": baseline.trajectory_id,
        "r_a3_config_sha256": sha256(problem["config_path"]),
    })
    metadata["scp"] = {
        "accepted_steps": result.accepted_steps,
        "convex_iterations": result.convex_iterations,
        "final_predicted_time_s": result.final_predicted_time_s,
        "initial_predicted_time_s": result.initial_predicted_time_s,
        "iterations": result.scp_iterations,
        "termination_reason": result.termination_reason,
        "validation": result.validation,
        "dense_speed_validation": result.dense_speed_validation,
        "joint_diagnostics": result.diagnostics,
        "physical_station_dense_validation": physical_dense_validation,
    }
    trajectory = RacingTrajectory(samples, metadata)
    if problem["config"].use_joint_speed_profile:
        validate_racing_trajectory(trajectory)
    return trajectory, result


def write_scp_candidate(output_dir, trajectory):
    track = trajectory.metadata["track_id"]
    if not track or not all(c.isalnum() or c in "_-" for c in track):
        raise ValueError("unsafe track identity for output filename")
    return write_racing_trajectory(
        output_dir, trajectory, stem=f"{track}_r_a3_scp_racing_trajectory"
    )
