import argparse
import inspect
import json
from pathlib import Path
from time import perf_counter

from ament_index_python.packages import get_package_share_directory

from . import trajectory as trajectory_module
from .trajectory import compose_racing_trajectory, sha256, validate_racing_trajectory, write_racing_trajectory


def main():
    parser = argparse.ArgumentParser(description="Build the deterministic baseline RacingTrajectory product")
    parser.add_argument("--output-dir", type=Path, default=Path.cwd())
    parser.add_argument("--r-a0-csv", type=Path)
    parser.add_argument("--r-a0-metadata", type=Path)
    parser.add_argument("--r-a1-csv", type=Path)
    parser.add_argument("--r-a1-metadata", type=Path)
    parser.add_argument("--track-id", default="iccas2025")
    args = parser.parse_args()
    started = perf_counter()
    data = Path(get_package_share_directory("f1tenth_racing_raceline")) / "data"
    trajectory = compose_racing_trajectory(
        args.r_a0_csv or data / f"{args.track_id}_r_a0_baseline.csv",
        args.r_a0_metadata or data / f"{args.track_id}_r_a0_baseline.json",
        args.r_a1_csv or data / f"{args.track_id}_r_a1_speed_profile.csv",
        args.r_a1_metadata or data / f"{args.track_id}_r_a1_speed_profile.json",
        sha256(inspect.getsourcefile(trajectory_module)),
        expected_track_id=args.track_id,
    )
    csv_path, metadata_path, trajectory = write_racing_trajectory(
        args.output_dir, trajectory, stem=f"{args.track_id}_r_a2_racing_trajectory")
    validation = validate_racing_trajectory(trajectory, csv_path)
    print(json.dumps({
        "csv": str(csv_path),
        "elapsed_s": perf_counter() - started,
        "metadata": str(metadata_path),
        "product_valid": validation["product_valid"],
        "runtime_tracking_qualified": trajectory.metadata["runtime_tracking_qualified"],
        "trajectory_id": trajectory.trajectory_id,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
