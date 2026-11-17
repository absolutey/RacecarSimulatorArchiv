import argparse
import json
from pathlib import Path
from time import perf_counter

from .artifacts import write_speed_artifacts, write_speed_svg
from .speed_adapter import centerline_comparator, load_speed_problem, solve_speed_problem, validate_dense


def _summary(profile):
    return {
        "maximum_speed_m_s": max(profile.target_speed),
        "mean_traversal_speed_m_s": sum(profile.segment_length) / profile.predicted_lap_time,
        "mean_sample_speed_m_s": sum(profile.target_speed) / len(profile.target_speed),
        "minimum_speed_m_s": min(profile.target_speed),
        "offline_envelope_predicted_lap_time_s": profile.predicted_lap_time,
        "path_length_m": sum(profile.segment_length),
    }


def main():
    parser = argparse.ArgumentParser(description="Generate the deterministic R-A1 ICCAS2025 speed envelope")
    parser.add_argument("--output-dir", type=Path, default=Path.cwd())
    parser.add_argument("--config", type=Path)
    parser.add_argument("--environment", type=Path)
    parser.add_argument("--r-a0-csv", type=Path)
    parser.add_argument("--r-a0-metadata", type=Path)
    parser.add_argument("--svg", type=Path)
    args = parser.parse_args()
    started = perf_counter()
    problem = load_speed_problem(args.config, r_a0_csv_path=args.r_a0_csv,
        r_a0_metadata_path=args.r_a0_metadata, environment_path_override=args.environment)
    profile = solve_speed_problem(problem)
    period = problem["r_a0_metadata"]["metrics"]["centerline"]["length"]
    dense = validate_dense(profile, period, int(problem["optimizer_data"]["dense_samples_per_segment"]))
    centerline, centerline_hash = centerline_comparator(problem)
    comparator = {"centerline_sha256": centerline_hash, **_summary(centerline)}
    csv_path, metadata_path, metadata = write_speed_artifacts(
        args.output_dir, profile, problem, dense, comparator
    )
    svg_hash = write_speed_svg(args.svg, profile) if args.svg else None
    print(json.dumps({
        "csv": str(csv_path),
        "elapsed_s": perf_counter() - started,
        "iterations": profile.iterations,
        "metadata": str(metadata_path),
        "offline_envelope_predicted_lap_time_s": profile.predicted_lap_time,
        "output_sha256": metadata["output_csv_sha256"],
        "svg_sha256": svg_hash,
        "valid": True,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
