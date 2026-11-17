#!/usr/bin/env python3
import argparse
import copy
import json
import os
import signal
import subprocess
import time

import rclpy
from control_msgs.msg import ActiveTargetTrajectory, OpponentPrediction, OvertakingPlan
from rclpy.node import Node

from f1tenth_racing_reference.core import overtaking_plan_identity, prediction_identity
from f1tenth_racing_reference.node import ACTIVE_REFERENCE_QOS


def set_time(target, nanoseconds):
    target.sec, target.nanosec = divmod(nanoseconds, 1_000_000_000)


def spin(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, required=True)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    process = subprocess.Popen(
        ["ros2", "run", "f1tenth_racing_reference", "nominal_active_reference"],
        start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
    )
    rclpy.init()
    node = Node(f"h2h_repair_fault_{args.domain}")
    active = []
    node.create_subscription(ActiveTargetTrajectory, "active_target_trajectory", active.append, ACTIVE_REFERENCE_QOS)
    prediction_pub = node.create_publisher(OpponentPrediction, "opponent_prediction", 10)
    plan_pub = node.create_publisher(OvertakingPlan, "overtaking_plan", 10)
    try:
        spin(node, 2.0)
        if not active or not active[-1].active_reference_valid:
            raise RuntimeError("nominal reference unavailable")
        nominal = active[-1]

        def prediction(age_s=0.0, revision=7):
            message = OpponentPrediction()
            now_ns = node.get_clock().now().nanoseconds - round(age_s * 1e9)
            set_time(message.header.stamp, now_ns)
            set_time(message.source_observation_time, now_ns)
            message.schema_version = 1
            message.source_opponent_id = "opponent0"
            message.source_track_revision = revision
            message.source_lifecycle = "TRACKED"
            message.source_confidence = 0.9
            message.source_hit_count = 8
            message.valid = True
            message.predictor = "TRACK_CV"
            message.track_id = nominal.track_id
            message.frame_id = nominal.frame_id
            return message

        def plan(source, revision=1):
            message = OvertakingPlan()
            now_ns = node.get_clock().now().nanoseconds
            set_time(message.header.stamp, now_ns)
            message.header.frame_id = nominal.frame_id
            message.schema_version = 2
            message.plan_revision = revision
            message.source_prediction_id = prediction_identity(source)
            message.source_track_revision = source.source_track_revision
            message.source_observation_time = source.source_observation_time
            set_time(message.valid_until, now_ns + 500_000_000)
            set_time(message.continuation_until, now_ns + 2_000_000_000)
            message.valid = True
            message.maneuver_state = "COMMITTED_PASS"
            message.passing_side = "LEFT"
            message.safety_state = "TEMPORAL_SAFE"
            message.status_reason = "FAULT_TEST_VALID"
            message.track_id, message.frame_id = nominal.track_id, nominal.frame_id
            message.map_image_sha256 = nominal.map_image_sha256
            message.start_progress_m = 0.0
            message.commit_end_progress_m = 5.0
            message.pass_end_progress_m = 25.0
            message.merge_end_progress_m = 30.0
            message.samples = list(nominal.samples)
            message.sample_count = len(message.samples)
            message.station_period_m = nominal.station_period_m
            message.path_length_m = nominal.path_length_m
            message.plan_id = overtaking_plan_identity(message)
            return message

        source = prediction()
        prediction_pub.publish(source)
        spin(node, 0.2)
        empty = plan(source)
        empty.source_prediction_id = ""
        plan_pub.publish(empty)
        spin(node, 0.3)
        empty_rejected = active[-1].source_kind == "NOMINAL"

        forged = plan(source)
        forged.source_prediction_id = "sha256:forged"
        plan_pub.publish(forged)
        spin(node, 0.3)
        forged_rejected = active[-1].source_kind == "NOMINAL"

        old = prediction(age_s=1.0, revision=8)
        prediction_pub.publish(old)
        spin(node, 0.1)
        plan_pub.publish(plan(old))
        spin(node, 0.3)
        stale_rejected = active[-1].source_kind == "NOMINAL"

        original_body = prediction(revision=10)
        altered_body = copy.deepcopy(original_body)
        altered_body.source_confidence = 0.1
        prediction_pub.publish(altered_body)
        spin(node, 0.1)
        plan_pub.publish(plan(original_body))
        spin(node, 0.3)
        forged_body_rejected = active[-1].source_kind == "NOMINAL"

        fresh = prediction(revision=9)
        prediction_pub.publish(fresh)
        spin(node, 0.1)
        valid = plan(fresh, revision=2)
        plan_pub.publish(valid)
        spin(node, 0.3)
        coherent_accepted = active[-1].source_kind == "OVERTAKE_OVERRIDE"
        spin(node, 0.5)
        committed_continued = active[-1].source_kind == "OVERTAKE_OVERRIDE"
        spin(node, 1.5)
        bounded_expiry_degraded = not active[-1].active_reference_valid

        recovery = prediction(revision=11)
        prediction_pub.publish(recovery)
        spin(node, 0.1)
        committed = plan(recovery, revision=3)
        plan_pub.publish(committed)
        spin(node, 0.2)
        abort = copy.deepcopy(committed)
        abort.valid = False
        abort.maneuver_state = "ABORT"
        abort.safety_state = "NOT_ACTIVE"
        plan_pub.publish(abort)
        spin(node, 0.2)
        invalid_abort_preserved = (active[-1].active_reference_valid and
                                   active[-1].source_kind == "OVERTAKE_OVERRIDE" and
                                   active[-1].source_trajectory_id == committed.plan_id)
        complete = copy.deepcopy(abort)
        complete.maneuver_state = "COMPLETE"
        plan_pub.publish(complete)
        spin(node, 0.2)
        complete_released = active[-1].source_kind == "NOMINAL" and active[-1].active_reference_valid

        result = {
            "empty_rejected": empty_rejected,
            "forged_rejected": forged_rejected,
            "stale_rejected": stale_rejected,
            "forged_prediction_body_rejected": forged_body_rejected,
            "coherent_accepted": coherent_accepted,
            "committed_continued_after_refresh_expiry": committed_continued,
            "bounded_expiry_degraded_not_nominal": bounded_expiry_degraded,
            "invalid_abort_preserved_committed_reference": invalid_abort_preserved,
            "complete_released_nominal": complete_released,
        }
        print(json.dumps(result, sort_keys=True))
        if not all(result.values()):
            raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10)


if __name__ == "__main__":
    main()
