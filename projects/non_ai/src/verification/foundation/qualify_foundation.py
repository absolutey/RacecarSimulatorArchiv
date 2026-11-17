#!/usr/bin/env python3
import argparse
import json
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Int32
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--no-rviz", action="store_true")
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    command = [
        "ros2", "launch", "f1tenth_racing_bringup", "race_foundation.launch.py",
        "use_r_b0:=false",
    ]
    if args.no_rviz:
        command.append("use_rviz:=false")
    started = time.monotonic()
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init(); node = Node(f"foundation_qualification_{args.domain}")
        laps = [None, None]; speeds = [None, None]; valid = [None]; walls = [None, None]; elapsed = [None]
        node.create_subscription(Int32, "lap_count0", lambda m: laps.__setitem__(0, m.data), 1)
        node.create_subscription(Int32, "lap_count1", lambda m: laps.__setitem__(1, m.data), 1)
        node.create_subscription(Float64, "speed0", lambda m: speeds.__setitem__(0, m.data), 1)
        node.create_subscription(Float64, "speed1", lambda m: speeds.__setitem__(1, m.data), 1)
        node.create_subscription(Bool, "race_valid", lambda m: valid.__setitem__(0, m.data), 1)
        node.create_subscription(Int32, "wall_collision_count0", lambda m: walls.__setitem__(0, m.data), 1)
        node.create_subscription(Int32, "wall_collision_count1", lambda m: walls.__setitem__(1, m.data), 1)
        node.create_subscription(Float64, "race_elapsed_time", lambda m: elapsed.__setitem__(0, m.data), 1)
        client = node.create_client(Trigger, "race_start")
        try:
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError("race_start service unavailable")
            while any(value is None for value in speeds):
                rclpy.spin_once(node, timeout_sec=0.1)
            waiting_speeds = list(speeds)
            if any(abs(value) > 1.0e-9 for value in waiting_speeds):
                raise RuntimeError(f"non-stationary WAITING state: {waiting_speeds}")
            future = client.call_async(Trigger.Request())
            while not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
            if not future.result().success:
                raise RuntimeError("race_start rejected")
            deadline = time.monotonic() + args.timeout
            next_progress = time.monotonic() + 10.0
            while any(value is None or value < 3 for value in laps):
                if process.poll() is not None:
                    raise RuntimeError(f"launch exited early: {process.returncode}")
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"qualification timeout: laps={laps}")
                rclpy.spin_once(node, timeout_sec=0.2)
                if time.monotonic() >= next_progress:
                    print(f"domain={args.domain} laps={laps} elapsed={elapsed[0]}", flush=True)
                    next_progress += 10.0
                if valid[0] is False or any(value not in (None, 0) for value in walls):
                    raise RuntimeError(f"invalid run: valid={valid[0]} walls={walls}")
            result = {"domain": args.domain, "waiting_speeds": waiting_speeds, "start_time": future.result().message, "laps": laps, "speeds": speeds, "walls": walls, "valid": valid[0], "race_elapsed_s": elapsed[0], "wall_elapsed_s": time.monotonic() - started}
        except Exception:
            log.seek(0); print(log.read()[-4000:]); raise
        finally:
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=5.0)
        result["launch_exit_code"] = process.returncode
        result["residual_processes"] = 0 if process.poll() is not None else 1
        print(json.dumps(result, sort_keys=True))
        if process.returncode != 0 or result["residual_processes"] != 0:
            raise SystemExit(1)


if __name__ == "__main__": main()
