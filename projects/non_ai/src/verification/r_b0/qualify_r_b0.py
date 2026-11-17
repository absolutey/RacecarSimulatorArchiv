#!/usr/bin/env python3
import argparse
import json
import os
import signal
import statistics
import subprocess
import tempfile
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from control_msgs.msg import CarState, ControllerStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger


def summary(values):
    ordered = sorted(values)
    pick = lambda q: ordered[min(len(ordered) - 1, round(q * (len(ordered) - 1)))]
    return {
        "mean": statistics.fmean(values),
        "p95": pick(0.95),
        "p99": pick(0.99),
        "max": max(values),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--laps", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    command = [
        "ros2", "launch", "f1tenth_racing_bringup", "race_foundation.launch.py",
        "use_rviz:=false",
    ]
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
        )
        rclpy.init()
        node = Node(f"r_b0_qualification_{args.domain}")
        laps = [None, None]
        walls = [None, None]
        valid = [None]
        statuses = []
        commands = []
        state_times = []
        command_times = []
        node.create_subscription(Int32, "lap_count0", lambda m: laps.__setitem__(0, m.data), 1)
        node.create_subscription(Int32, "lap_count1", lambda m: laps.__setitem__(1, m.data), 1)
        node.create_subscription(Int32, "wall_collision_count0", lambda m: walls.__setitem__(0, m.data), 1)
        node.create_subscription(Int32, "wall_collision_count1", lambda m: walls.__setitem__(1, m.data), 1)
        node.create_subscription(Bool, "race_valid", lambda m: valid.__setitem__(0, m.data), 1)
        node.create_subscription(ControllerStatus, "controller_status0", statuses.append, 10)
        node.create_subscription(
            CarState, "state0", lambda m: state_times.append(time.perf_counter()), 10
        )
        def receive_command(message):
            commands.append(message)
            command_times.append(time.perf_counter())
        command_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(
            AckermannDriveStamped, "ackermann_cmd0", receive_command, command_qos
        )
        client = node.create_client(Trigger, "race_start")
        started = time.monotonic()
        try:
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError("race_start unavailable")
            deadline = time.monotonic() + 10.0
            while not node.get_publishers_info_by_topic("/active_target_trajectory"):
                if time.monotonic() >= deadline:
                    raise RuntimeError("S0-R publisher unavailable")
                rclpy.spin_once(node, timeout_sec=0.1)
            blue_publishers = []
            while not blue_publishers and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                blue_publishers = node.get_publishers_info_by_topic("/ackermann_cmd0")
            if len(blue_publishers) != 1 or blue_publishers[0].node_name != "r_b0_racing_pursuit":
                raise RuntimeError(f"unexpected Blue command owners: {blue_publishers}")
            future = client.call_async(Trigger.Request())
            while not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
            deadline = time.monotonic() + args.timeout
            while any(value is None or value < args.laps for value in laps):
                if process.poll() is not None:
                    raise RuntimeError(f"launch exited early: {process.returncode}")
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"timeout laps={laps}")
                rclpy.spin_once(node, timeout_sec=0.1)
                if valid[0] is False or any(value not in (None, 0) for value in walls):
                    raise RuntimeError(f"invalid race valid={valid[0]} walls={walls}")
            if not statuses or not commands:
                raise RuntimeError("controller status/command not observed")
            result = {
                "domain": args.domain,
                "laps": laps,
                "walls": walls,
                "valid": valid[0],
                "sole_blue_command_publisher": blue_publishers[0].node_name,
                "source_reference_id": statuses[-1].active_reference_id,
                "reference_revision": statuses[-1].active_revision,
                "status_count": len(statuses),
                "command_count": len(commands),
                "state_count": len(state_times),
                "local_cap_count": sum(s.qualification_cap_active for s in statuses),
                "degradation_count": sum(s.degradation_active for s in statuses),
                "invalid_mode_count": sum(s.mode.startswith("INVALID") or s.mode == "STALE_STATE" for s in statuses),
                "lateral_abs_m": summary([abs(s.lateral_error_m) for s in statuses]),
                "heading_abs_rad": summary([abs(s.heading_error_rad) for s in statuses]),
                "speed_error_abs_mps": summary([abs(s.target_speed_mps - s.actual_speed_mps) for s in statuses]),
                "steering_abs_rad": summary([abs(s.steering_command_rad) for s in statuses]),
                "acceleration_abs_mps2": summary([abs(s.acceleration_command_mps2) for s in statuses]),
                "compute_us": summary([s.compute_time_us for s in statuses]),
                "state_hz": (len(state_times) - 1) / (state_times[-1] - state_times[0]),
                "command_hz": (len(command_times) - 1) / (command_times[-1] - command_times[0]),
                "drive_speed_nonzero_count": sum(abs(c.drive.speed) > 1e-12 for c in commands),
                "steering_limit_excess_count": sum(abs(c.drive.steering_angle) > 4.0 for c in commands),
                "acceleration_limit_excess_count": sum(abs(c.drive.acceleration) > 40.0 for c in commands),
                "wall_elapsed_s": time.monotonic() - started,
            }
        except Exception:
            log.seek(0)
            print(log.read()[-5000:])
            raise
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5.0)
        result["launch_exit_code"] = process.returncode
        result["residual_processes"] = 0 if process.poll() is not None else 1
        print(json.dumps(result, sort_keys=True))
        if process.returncode != 0 or result["residual_processes"] != 0:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
