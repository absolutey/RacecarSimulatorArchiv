#!/usr/bin/env python3
import argparse
from dataclasses import fields
import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node

from control_msgs.msg import ActiveTargetTrajectory
from f1tenth_racing_raceline import load_racing_trajectory
from f1tenth_racing_reference.node import ACTIVE_REFERENCE_QOS


def verify_message(message, expected):
    assert message.source_kind == "NOMINAL"
    assert message.source_trajectory_id == expected.trajectory_id
    assert message.source_qualification == "OFFLINE_BASELINE_QUALIFIED"
    assert message.source_product_valid is True
    assert message.source_runtime_tracking_qualified is False
    assert message.active_reference_valid is True
    assert message.track_id == expected.metadata["track_id"]
    assert message.frame_id == expected.metadata["frame_id"]
    assert message.sample_count == len(expected.samples) == len(message.samples)
    assert message.station_period_m == expected.metadata["station_period_m"]
    assert message.path_length_m == expected.metadata["path_length_m"]
    for source, transmitted in zip(expected.samples, message.samples):
        for field in fields(source):
            assert getattr(transmitted, field.name) == getattr(source, field.name)


def receive_late_joined(domain, run):
    process = subprocess.Popen(
        ["ros2", "run", "f1tenth_racing_reference", "nominal_active_reference"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        time.sleep(1.0)
        if process.poll() is not None:
            raise RuntimeError(f"S0-R exited before observer: {process.stdout.read()}")
        node = Node(f"s0_r_qualification_{domain}_{run}")
        received = []
        node.create_subscription(
            ActiveTargetTrajectory,
            "active_target_trajectory",
            received.append,
            ACTIVE_REFERENCE_QOS,
        )
        started = time.perf_counter()
        deadline = started + 10.0
        while not received and time.perf_counter() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.perf_counter() - started
        node.destroy_node()
        if not received:
            raise RuntimeError("late subscriber did not receive retained active reference")
        return received[0], elapsed
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10.0)
        if process.returncode != 0:
            raise RuntimeError(f"S0-R did not exit cleanly: {process.returncode}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, required=True)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    expected = load_racing_trajectory()
    rclpy.init()
    try:
        first, first_latency = receive_late_joined(args.domain, 1)
        second, second_latency = receive_late_joined(args.domain, 2)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    verify_message(first, expected)
    verify_message(second, expected)
    assert first.active_reference_id == second.active_reference_id
    assert first.active_revision == second.active_revision == 1
    print(json.dumps({
        "active_reference_id": first.active_reference_id,
        "active_revision": first.active_revision,
        "domain": args.domain,
        "first_late_join_latency_s": first_latency,
        "restart_identity_stable": True,
        "sample_count": first.sample_count,
        "second_late_join_latency_s": second_latency,
        "source_trajectory_id": first.source_trajectory_id,
        "transport_round_trip": "PASS",
    }, sort_keys=True))


if __name__ == "__main__":
    main()
