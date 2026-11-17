import argparse
import json
from pathlib import Path
from time import perf_counter

from .artifacts import write_artifacts, write_svg
from .foundation_adapter import load_problem, solve_problem


def main():
    parser = argparse.ArgumentParser(description="Generate the deterministic R-A0 ICCAS2025 baseline racing line")
    parser.add_argument("--output-dir", type=Path, default=Path.cwd())
    parser.add_argument("--config", type=Path)
    parser.add_argument("--environment", type=Path)
    parser.add_argument("--svg", type=Path)
    args = parser.parse_args()
    started = perf_counter()
    problem = load_problem(args.config, environment_path_override=args.environment)
    line, validation, centerline_metrics, line_metrics = solve_problem(problem)
    csv_path, metadata_path, metadata = write_artifacts(
        args.output_dir, line, problem, validation, centerline_metrics, line_metrics
    )
    svg_hash = write_svg(args.svg, problem["corridor"], line) if args.svg else None
    print(json.dumps({
        "csv": str(csv_path),
        "elapsed_s": perf_counter() - started,
        "final_objective": line.final_objective,
        "initial_objective": line.initial_objective,
        "metadata": str(metadata_path),
        "output_sha256": metadata["output_csv_sha256"],
        "svg_sha256": svg_hash,
        "termination_reason": line.termination_reason,
        "valid": True,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
