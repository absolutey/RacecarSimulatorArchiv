#!/usr/bin/env python3
import argparse
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import tempfile
import time
from types import SimpleNamespace

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import ControllerStatus, OpponentPrediction, OpponentState
from f1tenth_racing_sim.sensing.lidar import scan_occupancy
from f1tenth_racing_sim.sensing.raycast import ray_box_ranges
from f1tenth_racing_sim.world.map import OccupancyMap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def distribution(values):
    if not values:
        return None
    ordered = sorted(values)
    at = lambda q: ordered[min(len(ordered) - 1, round(q * (len(ordered) - 1)))]
    return {"mean": statistics.fmean(values), "p95": at(0.95), "p99": at(0.99), "max": max(values)}


def nearest(history, target, tolerance=0.06):
    if not history:
        return None
    best = min(history, key=lambda item: abs(item[0] - target))
    return best if abs(best[0] - target) <= tolerance else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--tracker", choices=("cv", "ukf"), default="ukf")
    parser.add_argument("--solo", action="store_true")
    parser.add_argument("--laps", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--blue-cap", type=float, default=2.0)
    parser.add_argument("--allow-known-overlap-watch", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--predictor", choices=("track_cv", "imm"))
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    command = [
        "ros2", "launch", "f1tenth_racing_bringup", "race_foundation.launch.py",
        "use_rviz:=false", "use_r_b0:=true", f"solo:={'true' if args.solo else 'false'}",
        f"speed_cap_mps:={args.blue_cap}", "use_opponent_perception:=true",
        f"opponent_tracker:={args.tracker}",
    ]
    if args.predictor:
        command += ["use_opponent_prediction:=true", f"opponent_predictor:={args.predictor}"]
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node(f"perception_truth_evaluator_{args.domain}")
        laps, walls, valid = [None], [None], [None]
        scans, estimates, predictions, controls = [], [], [], []
        odom0, odom1 = [], []
        node.create_subscription(LaserScan, "scan0", scans.append, 10)
        node.create_subscription(OpponentState, "opponent_state", estimates.append, 10)
        node.create_subscription(OpponentPrediction, "opponent_prediction", predictions.append, 10)
        node.create_subscription(ControllerStatus, "controller_status0", controls.append, 10)
        node.create_subscription(Int32, "lap_count0", lambda m: laps.__setitem__(0, m.data), 1)
        node.create_subscription(Int32, "wall_collision_count0", lambda m: walls.__setitem__(0, m.data), 1)
        node.create_subscription(Bool, "race_valid", lambda m: valid.__setitem__(0, m.data), 1)

        def receive_odom(history, message):
            pose, twist = message.pose.pose, message.twist.twist
            yaw = math.atan2(
                2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2),
            )
            vx = twist.linear.x * math.cos(yaw) - twist.linear.y * math.sin(yaw)
            vy = twist.linear.x * math.sin(yaw) + twist.linear.y * math.cos(yaw)
            history.append((stamp_s(message.header.stamp), pose.position.x, pose.position.y,
                            vx, vy, yaw))

        node.create_subscription(Odometry, "odom0", lambda m: receive_odom(odom0, m), 10)
        if not args.solo:
            node.create_subscription(Odometry, "odom1", lambda m: receive_odom(odom1, m), 10)
        client = node.create_client(Trigger, "race_start")
        started = time.monotonic()
        try:
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError("race_start unavailable")
            future = client.call_async(Trigger.Request())
            start_deadline = time.monotonic() + 15.0
            while not future.done():
                if time.monotonic() >= start_deadline:
                    raise RuntimeError("race_start response timeout")
                rclpy.spin_once(node, timeout_sec=0.1)
            deadline = time.monotonic() + args.timeout
            observation_deadline = time.monotonic() + args.duration if args.duration else None
            while ((observation_deadline is not None and time.monotonic() < observation_deadline) or
                   (observation_deadline is None and (laps[0] is None or laps[0] < args.laps))):
                if process.poll() is not None:
                    raise RuntimeError(f"launch exited early: {process.returncode}")
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"timeout laps={laps[0]}")
                rclpy.spin_once(node, timeout_sec=0.1)
                if (valid[0] is False and not args.allow_known_overlap_watch) or walls[0] not in (None, 0):
                    raise RuntimeError(f"invalid run valid={valid[0]} walls={walls[0]}")
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
            if not scans or not estimates or not controls:
                raise RuntimeError("incomplete product observations")
            position_errors, velocity_errors, large_errors, aligned = [], [], [], 0
            for estimate in estimates:
                if not estimate.valid or args.solo:
                    continue
                truth = nearest(odom1, stamp_s(estimate.header.stamp))
                if truth is None:
                    continue
                aligned += 1
                position_error = math.hypot(estimate.x_m - truth[1], estimate.y_m - truth[2])
                position_errors.append(position_error)
                if position_error > 1.0:
                    large_errors.append([
                        round(estimate.x_m, 3), round(estimate.y_m, 3),
                        round(truth[1], 3), round(truth[2], 3),
                        round(estimate.measured_extent_m, 3), estimate.hit_count,
                    ])
                velocity_errors.append(math.hypot(estimate.vx_mps - truth[3], estimate.vy_mps - truth[4]))
            scan_times = [stamp_s(scan.header.stamp) for scan in scans]
            estimate_times = [stamp_s(item.header.stamp) for item in estimates]
            prediction_errors = {0.25: [], 0.5: [], 1.0: []}
            prediction_coverage = {0.25: [], 0.5: [], 1.0: []}
            for prediction in predictions:
                if not prediction.valid or args.solo:
                    continue
                source_time = stamp_s(prediction.source_observation_time)
                for horizon in prediction_errors:
                    sample = min(prediction.samples, key=lambda item: abs(item.dt_s - horizon))
                    if abs(sample.dt_s - horizon) > 0.051:
                        continue
                    truth = nearest(odom1, source_time + sample.dt_s)
                    if truth is None:
                        continue
                    dx, dy = truth[1] - sample.x_m, truth[2] - sample.y_m
                    prediction_errors[horizon].append(math.hypot(dx, dy))
                    along = dx * math.cos(sample.heading_rad) + dy * math.sin(sample.heading_rad)
                    across = -dx * math.sin(sample.heading_rad) + dy * math.cos(sample.heading_rad)
                    prediction_coverage[horizon].append(
                        abs(along) <= sample.occupancy_half_length_m and
                        abs(across) <= sample.occupancy_half_width_m
                    )
            visible_opportunities = detected_visible = 0
            if not args.solo:
                sim_share = Path(get_package_share_directory("f1tenth_racing_sim"))
                environment = yaml.safe_load((sim_share / "config/competition_environment.yaml").read_text())
                occupancy = OccupancyMap(sim_share / environment["map"]["image_file"], environment["map"])
                mount = environment["vehicle"]["description"]
                footprint = environment["collision"]["footprint_relative_to_base_link"]
                timed_estimates = [(stamp_s(item.header.stamp), item) for item in estimates]
                for scan in scans:
                    scan_time = stamp_s(scan.header.stamp)
                    blue, red = nearest(odom0, scan_time), nearest(odom1, scan_time)
                    if blue is None or red is None:
                        continue
                    sensor_x = blue[1] + mount["lidar_x_m"] * math.cos(blue[5])
                    sensor_y = blue[2] + mount["lidar_x_m"] * math.sin(blue[5])
                    angle = math.atan2(red[2] - sensor_y, red[1] - sensor_x)
                    relative = math.atan2(math.sin(angle - blue[5]), math.cos(angle - blue[5]))
                    if not scan.angle_min <= relative <= scan.angle_max:
                        continue
                    static_range = scan_occupancy(
                        occupancy, sensor_x, sensor_y, [angle], scan.range_min, scan.range_max,
                    )[0]
                    target_range = ray_box_ranges(
                        sensor_x, sensor_y, np.array([angle]),
                        SimpleNamespace(x=red[1], y=red[2], yaw=red[5]), footprint, scan.range_max,
                    )[0]
                    if target_range >= static_range or target_range >= scan.range_max:
                        continue
                    visible_opportunities += 1
                    estimate = nearest(timed_estimates, scan_time, tolerance=0.15)
                    detected_visible += bool(estimate is not None and estimate[1].valid)
            control_times = [item.receipt_time_ns * 1e-9 for item in controls]
            result = {
                "domain": args.domain,
                "tracker": args.tracker,
                "solo": args.solo,
                "laps": laps[0],
                "valid": valid[0],
                "walls": walls[0],
                "scan_count": len(scans),
                "estimate_count": len(estimates),
                "scan_hz": (len(scan_times) - 1) / (scan_times[-1] - scan_times[0]),
                "opponent_state_hz": (len(estimate_times) - 1) / (estimate_times[-1] - estimate_times[0]),
                "valid_state_count": sum(item.valid for item in estimates),
                "false_valid_state_count": sum(item.valid for item in estimates) if args.solo else None,
                "false_valid_positions": [
                    [round(item.x_m, 3), round(item.y_m, 3)]
                    for item in estimates if args.solo and item.valid and item.miss_count == 0
                ],
                "residual_points": distribution([item.residual_point_count for item in estimates]),
                "candidate_count": distribution([item.candidate_count for item in estimates]),
                "measured_extent_m": distribution([
                    item.measured_extent_m for item in estimates if item.valid and item.miss_count == 0
                ]),
                "position_error_m": distribution(position_errors),
                "position_rmse_m": math.sqrt(statistics.fmean([value * value for value in position_errors])) if position_errors else None,
                "large_error_tracks": large_errors[:20],
                "velocity_error_mps": distribution(velocity_errors),
                "velocity_rmse_mps": math.sqrt(statistics.fmean([value * value for value in velocity_errors])) if velocity_errors else None,
                "truth_aligned_valid_states": aligned,
                "visible_opportunities": visible_opportunities,
                "visible_detection_recall": (
                    detected_visible / visible_opportunities if visible_opportunities else None
                ),
                "compute_us": distribution([item.compute_time_us for item in estimates]),
                "prediction_count": len(predictions),
                "valid_prediction_count": sum(item.valid for item in predictions),
                "prediction_compute_us": distribution([item.compute_time_us for item in predictions if item.valid]),
                "prediction_error_m": {
                    str(horizon): distribution(values) for horizon, values in prediction_errors.items()
                },
                "prediction_coverage": {
                    str(horizon): (sum(values) / len(values) if values else None)
                    for horizon, values in prediction_coverage.items()
                },
                "control_compute_us": distribution([item.compute_time_us for item in controls]),
                "control_hz": ((len(control_times) - 1) / (control_times[-1] - control_times[0])
                               if len(control_times) > 1 else None),
                "state_hz": ((len(odom0) - 1) / (odom0[-1][0] - odom0[0][0])
                             if len(odom0) > 1 else None),
                "control_degradation_count": sum(item.degradation_active for item in controls),
                "product_truth_subscriptions": 0,
                "wall_elapsed_s": time.monotonic() - started,
            }
        except Exception:
            log.seek(0); print(log.read()[-5000:]); raise
        finally:
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            if process.poll() is None: os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=5.0)
        result["launch_exit_code"] = process.returncode
        result["residual_processes"] = 0 if process.poll() is not None else 1
        print(json.dumps(result, sort_keys=True))
        if process.returncode != 0 or result["residual_processes"]:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
