#!/usr/bin/env python3
"""Replay D226 truth trajectories into the Foundation LiDAR model for R-C profiling.

Truth is used only to reconstruct recorded sensor input offline; this script is not
installed and has no product-control path.
"""

import argparse
import csv
import gc
import json
import math
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from f1tenth_racing_perception import (
    CvKalmanTracker, Pose2D, TrackCorridor, extract_measurements,
)
from f1tenth_racing_perception.expected_scan import StaticExpectedScan
from f1tenth_racing_sim.sensing.lidar import scan_occupancy
from f1tenth_racing_sim.sensing.raycast import ray_box_ranges
from f1tenth_racing_sim.world.map import OccupancyMap


def nearest(rows, stamp):
    return min(rows, key=lambda row: abs(row[0] - stamp))


def distribution(values):
    ordered = sorted(values)
    pick = lambda fraction: ordered[round((len(ordered) - 1) * fraction)]
    return {
        "mean": sum(ordered) / len(ordered), "median": pick(0.5),
        "p95": pick(0.95), "p99": pick(0.99), "max": ordered[-1],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("raw_runtime", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    raw = json.loads(args.raw_runtime.read_text())
    sim_share = Path(get_package_share_directory("f1tenth_racing_sim"))
    perception_share = Path(get_package_share_directory("f1tenth_racing_perception"))
    environment = yaml.safe_load((sim_share / "config/competition_environment.yaml").read_text())
    config = yaml.safe_load((perception_share / "config/perception.yaml").read_text())
    occupancy = OccupancyMap(sim_share / environment["map"]["image_file"], environment["map"])
    lidar, mount = environment["lidar"], environment["vehicle"]["description"]
    projector = StaticExpectedScan(
        occupancy, lidar["angle_min_rad"], lidar["angle_increment_rad"],
        lidar["beam_count"], lidar["range_min_m"], lidar["range_max_m"],
    )
    with (sim_share / environment["map"]["centerline_file"]).open(newline="") as stream:
        corridor = TrackCorridor(list(csv.DictReader(stream)), 0.5 * mount["chassis_width_m"])
    tracker = CvKalmanTracker(config, corridor.geometry)
    radius = 0.5 * math.hypot(mount["chassis_length_m"], mount["chassis_width_m"])
    blue, red = raw["odom0"], raw["odom1"]
    start = raw["race_start_time_s"]
    end = raw.get("result", {}).get("collision_time_s") or blue[-1][0]
    samples, next_stamp = [], start
    for row in blue:
        if row[0] + 1e-9 < next_stamp:
            continue
        if row[0] > end:
            break
        samples.append(row)
        next_stamp += 0.1

    scans = []
    relative_angles = lidar["angle_min_rad"] + np.arange(lidar["beam_count"]) * lidar["angle_increment_rad"]
    for current in samples:
        _, x, y, yaw = current
        other = nearest(red, current[0])
        sensor_x = x + mount["lidar_x_m"] * math.cos(yaw) - mount["lidar_y_m"] * math.sin(yaw)
        sensor_y = y + mount["lidar_x_m"] * math.sin(yaw) + mount["lidar_y_m"] * math.cos(yaw)
        angles = yaw + relative_angles
        static = scan_occupancy(
            occupancy, sensor_x, sensor_y, angles,
            lidar["range_min_m"], lidar["range_max_m"],
        )
        dynamic = ray_box_ranges(
            sensor_x, sensor_y, angles,
            SimpleNamespace(x=other[1], y=other[2], yaw=other[3]),
            environment["collision"]["footprint_relative_to_base_link"],
            lidar["range_max_m"],
        )
        scans.append(np.minimum(static, dynamic))
    gc.collect()

    headers = (
        "sequence", "scan_time_s", "pose_candidates", "finite_beams", "residual_beams",
        "measurements", "pose_score_ms", "expected_scan_ms", "residual_ms",
        "dynamic_filter_ms", "clustering_ms", "candidate_filter_ms", "tracking_ms",
        "total_wall_ms", "total_cpu_ms",
    )
    records = []
    for sequence, (current, observed) in enumerate(zip(samples, scans), 1):
        stamp, x, y, yaw = current
        poses = [Pose2D(row[0], row[1], row[2], row[3]) for row in blue
                 if 0.0 <= stamp - row[0] <= config["maximum_pose_alignment_s"]]
        started_wall, started_cpu = time.perf_counter_ns(), time.process_time_ns()
        ranges = np.asarray(observed)
        indices = np.flatnonzero(np.isfinite(ranges) & (ranges < lidar["range_max_m"] - 1e-6))
        def score(pose):
            sx = pose.x_m + mount["lidar_x_m"] * math.cos(pose.yaw_rad) - mount["lidar_y_m"] * math.sin(pose.yaw_rad)
            sy = pose.y_m + mount["lidar_x_m"] * math.sin(pose.yaw_rad) + mount["lidar_y_m"] * math.cos(pose.yaw_rad)
            angles = pose.yaw_rad + lidar["angle_min_rad"] + indices * lidar["angle_increment_rad"]
            xs, ys = sx + ranges[indices] * np.cos(angles), sy + ranges[indices] * np.sin(angles)
            return sum(occupancy.occupied(float(px), float(py)) for px, py in zip(xs, ys))
        pose = max(poses, key=score)
        pose_done = time.perf_counter_ns()
        sensor_x = pose.x_m + mount["lidar_x_m"] * math.cos(pose.yaw_rad) - mount["lidar_y_m"] * math.sin(pose.yaw_rad)
        sensor_y = pose.y_m + mount["lidar_x_m"] * math.sin(pose.yaw_rad) + mount["lidar_y_m"] * math.cos(pose.yaw_rad)
        expected = projector.scan(sensor_x, sensor_y, pose.yaw_rad)
        expected_done = time.perf_counter_ns()
        marks = {}
        measurements, residual_count = extract_measurements(
            observed, expected, lidar["angle_min_rad"], lidar["angle_increment_rad"], pose,
            mount["lidar_x_m"], mount["lidar_y_m"], config, occupancy, corridor, radius,
            stage_marks=marks,
        )
        extraction_done = time.perf_counter_ns()
        tracker.update(measurements, stamp)
        finished_wall, finished_cpu = time.perf_counter_ns(), time.process_time_ns()
        records.append((
            sequence, stamp, len(poses), len(indices), residual_count, len(measurements),
            (pose_done - started_wall) / 1e6,
            (expected_done - pose_done) / 1e6,
            (marks["residual"][0] - expected_done) / 1e6,
            (marks["dynamic"][0] - marks["residual"][0]) / 1e6,
            (marks["clustering"][0] - marks["dynamic"][0]) / 1e6,
            (marks["candidates"][0] - marks["clustering"][0]) / 1e6,
            (finished_wall - extraction_done) / 1e6,
            (finished_wall - started_wall) / 1e6,
            (finished_cpu - started_cpu) / 1e6,
        ))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as stream:
        writer = csv.writer(stream, delimiter="\t"); writer.writerow(headers); writer.writerows(records)
    columns = list(zip(*records))
    summary = {headers[index]: distribution(list(columns[index])) for index in range(6, len(headers))}
    summary.update({"samples": len(records), "duration_s": end - start})
    args.output.with_suffix(".json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
