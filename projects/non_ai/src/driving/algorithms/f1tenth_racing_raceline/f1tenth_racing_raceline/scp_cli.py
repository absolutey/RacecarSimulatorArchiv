import argparse
import json
from pathlib import Path

from .scp_adapter import load_scp_problem, solve_scp_problem, write_scp_candidate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--baseline-csv", type=Path)
    parser.add_argument("--baseline-metadata", type=Path)
    parser.add_argument("--environment", type=Path)
    parser.add_argument("--r-a0-csv", type=Path)
    parser.add_argument("--r-a0-metadata", type=Path)
    args = parser.parse_args()
    trajectory, result = solve_scp_problem(load_scp_problem(
        args.config, baseline_csv=args.baseline_csv, baseline_metadata=args.baseline_metadata,
        environment_path=args.environment, r_a0_csv=args.r_a0_csv,
        r_a0_metadata=args.r_a0_metadata))
    csv_path, metadata_path, written = write_scp_candidate(args.output_dir, trajectory)
    print(json.dumps({
        "accepted_steps": result.accepted_steps,
        "csv": str(csv_path),
        "final_predicted_time_s": result.final_predicted_time_s,
        "initial_predicted_time_s": result.initial_predicted_time_s,
        "metadata": str(metadata_path),
        "scp_iterations": result.scp_iterations,
        "termination_reason": result.termination_reason,
        "trajectory_id": written.trajectory_id,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
