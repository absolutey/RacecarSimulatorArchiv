#!/usr/bin/env python3
import argparse
import json
import math
import os
import signal
import subprocess
import time

import rclpy
from builtin_interfaces.msg import Time
from control_msgs.msg import ActiveTargetTrajectory, CarState, ControllerStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from f1tenth_racing_raceline import load_racing_trajectory
from f1tenth_racing_reference import invalid_active_reference, select_nominal
from f1tenth_racing_reference.node import ACTIVE_REFERENCE_QOS
from f1tenth_racing_reference.transport import to_message


def spin_until(node, statuses, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if statuses and predicate(statuses[-1]):
            return statuses[-1]
    raise RuntimeError(f"expected controller mode not observed: {[s.mode for s in statuses[-10:]]}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, required=True)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    process = subprocess.Popen(
        ["ros2", "run", "f1tenth_racing_control", "r_b0_controller"],
        start_new_session=True,
    )
    rclpy.init()
    node = Node(f"r_b0_degradation_{args.domain}")
    statuses = []
    active_pub = node.create_publisher(
        ActiveTargetTrajectory, "active_target_trajectory", ACTIVE_REFERENCE_QOS
    )
    state_pub = node.create_publisher(CarState, "state0", 10)
    start_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    start_pub = node.create_publisher(Time, "race_start_time", start_qos)
    node.create_subscription(ControllerStatus, "controller_status0", statuses.append, 10)
    source = load_racing_trajectory()
    valid = to_message(select_nominal(source))
    invalid = to_message(invalid_active_reference("INJECTED_INVALID_REFERENCE"))
    state = CarState(px=source.samples[0].x_m, py=source.samples[0].y_m,
                     yaw=source.samples[0].heading_rad, v=1.0)
    try:
        time.sleep(0.8)
        now = node.get_clock().now().nanoseconds
        start_pub.publish(Time(sec=now // 1_000_000_000, nanosec=now % 1_000_000_000))
        active_pub.publish(valid)
        for _ in range(5):
            state_pub.publish(state)
            rclpy.spin_once(node, timeout_sec=0.05)
        normal = spin_until(node, statuses, lambda s: s.mode == "LOCAL_SPEED_CAPPED")
        active_pub.publish(invalid)
        for _ in range(3):
            state_pub.publish(state)
            rclpy.spin_once(node, timeout_sec=0.05)
        invalid_status = spin_until(node, statuses, lambda s: s.mode == "INVALID_REFERENCE")
        active_pub.publish(valid)
        for _ in range(5):
            state_pub.publish(state)
            rclpy.spin_once(node, timeout_sec=0.05)
        recovered = spin_until(node, statuses, lambda s: s.mode == "LOCAL_SPEED_CAPPED")
        stale = spin_until(node, statuses, lambda s: s.mode == "STALE_STATE")
        for status in (invalid_status, stale):
            assert status.degradation_active
            assert status.acceleration_command_mps2 == -2.0
            assert math.isfinite(status.steering_command_rad)
            assert math.isfinite(status.acceleration_command_mps2)
        print(json.dumps({
            "domain": args.domain,
            "invalid_reference_mode": invalid_status.mode,
            "normal_mode": normal.mode,
            "recovery_mode": recovered.mode,
            "stale_state_mode": stale.mode,
            "degraded_acceleration_mps2": stale.acceleration_command_mps2,
            "finite_commands": True,
        }, sort_keys=True))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10.0)
    if process.returncode != 0:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
