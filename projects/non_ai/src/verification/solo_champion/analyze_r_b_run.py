#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path
import sqlite3
import statistics


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * fraction
    low = int(position)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def distribution(values):
    values = [value for value in values if value is not None and math.isfinite(value)]
    if not values:
        return None
    return {
        "mean": statistics.fmean(values), "median": statistics.median(values),
        "p95": percentile(values, 0.95), "max": max(values),
    }


def periodic_between(value, start, end, period):
    return (value - start) % period <= (end - start) % period


def aggregate(rows, field, function):
    values = [row[field] for row in rows if row[field] is not None]
    return function(values) if values else None


def corner_segments(trajectory):
    """Signed curvature lobes above 0.15 1/m; independent of controller results."""
    active = [abs(float(row["curvature_per_m"])) >= 0.15 for row in trajectory]
    signs = [1 if float(row["curvature_per_m"]) >= 0.0 else -1 for row in trajectory]
    groups = []
    current = []
    for index in range(len(trajectory)):
        if active[index] and (not current or signs[index] == signs[current[-1]]):
            current.append(index)
        elif active[index]:
            groups.append(current); current = [index]
        elif current:
            groups.append(current); current = []
    if current:
        groups.append(current)
    return [{
        "corner_id": index + 1,
        "direction": "LEFT" if signs[group[0]] > 0 else "RIGHT",
        "start_s_m": float(trajectory[group[0]]["s_m"]),
        "end_s_m": float(trajectory[group[-1]]["s_m"]),
        "apex_s_m": float(trajectory[max(
            group, key=lambda item: abs(float(trajectory[item]["curvature_per_m"]))
        )]["s_m"]),
    } for index, group in enumerate(groups)]


def analyze(run_dir):
    run_dir = Path(run_dir)
    manifest = json.loads((run_dir / "manifest.json").read_text())
    bak = next(run_dir.glob("*_phase_review.sqlite.bak"))
    database = sqlite3.connect(f"file:{bak}?mode=ro", uri=True)
    database.row_factory = sqlite3.Row
    core = list(database.execute(
        "SELECT t.*, r.* FROM telemetry t JOIN r_b_runtime r USING(row_no) ORDER BY row_no"
    ))
    solver = list(database.execute("SELECT * FROM r_b_solver ORDER BY row_no"))
    integrity = database.execute("PRAGMA integrity_check").fetchone()[0]
    database.close()
    trajectory_path = next(
        Path(item["path"]) for item in manifest["inputs"] if item["path"].endswith(".csv")
    )
    trajectory = list(csv.DictReader(trajectory_path.open()))
    period = sum(float(row["ds_to_next_m"]) for row in trajectory)
    segments = []
    for segment in corner_segments(trajectory):
        rows = [row for row in core if periodic_between(
            row["blue_s_m"], segment["start_s_m"], segment["end_s_m"], period
        )]
        length = (segment["end_s_m"] - segment["start_s_m"]) % period
        entry = [row for row in rows if (row["blue_s_m"] - segment["start_s_m"]) % period <= .1 * length]
        exit_rows = [row for row in rows if (row["blue_s_m"] - segment["start_s_m"]) % period >= .9 * length]
        apex = [row for row in rows if abs(
            ((row["blue_s_m"] - segment["apex_s_m"] + period / 2) % period) - period / 2
        ) <= 0.5]
        before = [row for row in core if 0.0 <= (segment["start_s_m"] - row["blue_s_m"]) % period <= 2.0]
        after = [row for row in core if 0.0 <= (row["blue_s_m"] - segment["apex_s_m"]) % period <= 2.0]
        segment.update({
            "entry_speed_mps": aggregate(entry, "actual_speed_mps", statistics.fmean),
            "apex_speed_mps": aggregate(apex, "actual_speed_mps", min),
            "exit_speed_mps": aggregate(exit_rows, "actual_speed_mps", statistics.fmean),
            "peak_abs_slip_rad": aggregate(rows, "slip_angle_rad",
                                            lambda values: max(map(abs, values))),
            "peak_abs_lateral_error_m": aggregate(rows, "lateral_error_m",
                                                    lambda values: max(map(abs, values))),
            "peak_abs_steering_rad": aggregate(rows, "steering_command_rad",
                                                lambda values: max(map(abs, values))),
            "braking_onset_s_m": next((row["blue_s_m"] for row in before
                                        if row["acceleration_command_mps2"] < -0.1), None),
            "acceleration_resume_s_m": next((row["blue_s_m"] for row in after
                                               if row["acceleration_command_mps2"] > 0.1), None),
        })
        segments.append(segment)
    lap_times, previous = [], None
    for row in core:
        lap_time = row["lap_time_s"]
        if lap_time and lap_time != previous:
            lap_times.append(lap_time); previous = lap_time
    local_ratio = [row["local_target_speed_mps"] / row["r_a_target_speed_mps"]
                   for row in core if row["r_a_target_speed_mps"] > 0.0]
    return {
        "run_id": manifest["run_id"], "candidate": manifest["controller"]["candidate_id"],
        "profile": manifest["controller"]["controller_profile"],
        "config_hash": manifest["controller_config_hash"], "integrity": integrity,
        "valid": all(row["race_valid"] for row in core),
        "walls": max(row["wall_collisions"] for row in core),
        "car_collision": any(row["car_collision"] for row in core),
        "flying_lap_times_s": lap_times,
        "lap_time_s": distribution(lap_times),
        "actual_speed_mps": distribution([row["actual_speed_mps"] for row in core]),
        "r_a_speed_utilization": distribution(local_ratio),
        "lateral_error_abs_m": distribution([abs(row["lateral_error_m"]) for row in core]),
        "heading_error_abs_rad": distribution([abs(row["heading_error_rad"]) for row in core]),
        "slip_abs_rad": distribution([abs(row["slip_angle_rad"]) for row in core]),
        "yaw_rate_abs_rad_s": distribution([abs(row["yaw_rate_rad_s"]) for row in core]),
        "compute_us": distribution([row["controller_compute_us"] for row in core]),
        "fallback_rows": sum(row["fallback_active"] for row in core),
        "recovery_rows": sum(row["recovery_active"] for row in core),
        "deadline_misses": sum(row["deadline_met"] == 0 for row in solver),
        "solver_rows": len(solver), "corners": segments,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs="+")
    args = parser.parse_args()
    print(json.dumps([analyze(path) for path in args.run_dirs], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
