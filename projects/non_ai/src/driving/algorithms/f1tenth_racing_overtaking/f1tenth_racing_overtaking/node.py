import csv
from dataclasses import fields
import json
import math
from pathlib import Path
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import ActiveTargetTrajectory, CarState, OpponentPrediction, OvertakingPlan, RacingTrajectorySample
from rclpy.node import Node

from f1tenth_racing_reference.core import overtaking_plan_identity, prediction_identity
from f1tenth_racing_reference.node import ACTIVE_REFERENCE_QOS

from .core import (OvertakingConfig, OvertakingSupervisor, Prediction, PredictionSample,
                   TrajectorySample, continuation_deadline_s)
from .runtime_environment import load_verified_runtime
from .state_input_contract import admit_ego_state


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class OvertakingPlannerNode(Node):
    def __init__(self):
        super().__init__("r_e_overtaking_planner")
        self.declare_parameter("qualified_speed_cap_mps", 3.0)
        self.declare_parameter("racing_speed_cap_mps", 3.0)
        share = Path(get_package_share_directory("f1tenth_racing_overtaking"))
        sim_share = Path(get_package_share_directory("f1tenth_racing_sim"))
        settings = yaml.safe_load((share / "config/overtaking.yaml").read_text())
        # All H2H planners must bind to the launch-selected environment;
        # hardcoding ICCAS while launch selects Spielberg is unsafe.
        self.declare_parameter('environment_file',
                               str(sim_share/'config/competition_environment.yaml'))
        self.declare_parameter('centerline_file','')
        self.declare_parameter('expected_track_id','')
        self.declare_parameter('expected_map_sha256','')
        environment,rows=load_verified_runtime(
            self.get_parameter('environment_file').value,
            self.get_parameter('centerline_file').value,sim_share,
            expected_track_id=self.get_parameter('expected_track_id').value,
            expected_map_sha256=self.get_parameter('expected_map_sha256').value)
        config_fields = {field.name for field in fields(OvertakingConfig)}
        dynamics = environment["dynamics"]
        config_values = {key: value for key, value in settings.items() if key in config_fields}
        config_values.update({
            "footprint_half_length_m": max(
                abs(environment["collision"]["footprint_relative_to_base_link"]["min_x_m"]),
                abs(environment["collision"]["footprint_relative_to_base_link"]["max_x_m"]),
            ),
            "footprint_half_width_m": max(
                abs(environment["collision"]["footprint_relative_to_base_link"]["min_y_m"]),
                abs(environment["collision"]["footprint_relative_to_base_link"]["max_y_m"]),
            ),
            "wheelbase_m": dynamics["dynamic_wheelbase_m"],
            "steering_angle_max_rad": dynamics["limits"]["steering_angle_abs_max_rad"],
            "steering_rate_max_rad_s": dynamics["limits"]["steering_rate_abs_max_rad_s"],
            "candidate_speed_cap_mps": self.get_parameter("qualified_speed_cap_mps").value,
            "racing_speed_cap_mps": self.get_parameter("racing_speed_cap_mps").value,
        })
        config = OvertakingConfig(**config_values)
        self.supervisor = OvertakingSupervisor(
            config,
            [(float(row["x_m"]), float(row["y_m"])) for row in rows],
            [float(row["w_tr_left_m"]) for row in rows],
            [float(row["w_tr_right_m"]) for row in rows],
        )
        self.settings, self.environment = settings, environment
        self.nominal = self.prediction = self.state = None
        self._last_state_sequence = -1
        self._last_state_stamp_ns = -1
        self.publisher = self.create_publisher(OvertakingPlan, "overtaking_plan", 1)
        self.create_subscription(ActiveTargetTrajectory, "active_target_trajectory", self.receive_reference, ACTIVE_REFERENCE_QOS)
        self.create_subscription(OpponentPrediction, "opponent_prediction", self.receive_prediction, 10)
        self.create_subscription(CarState, "state0", self.receive_state, 1)
        self.create_timer(1.0 / settings["planning_rate_hz"], self.plan)

    def receive_reference(self, message):
        if (message.active_reference_valid and message.source_kind == "NOMINAL"
                and message.sample_count == len(message.samples)
                and message.track_id == self.environment['environment_identity']['active_track']
                and message.frame_id == 'map'
                and message.map_image_sha256 == self.environment['map']['image_sha256']):
            self.nominal = tuple(TrajectorySample(**{
                field.name: getattr(sample, field.name) for field in fields(TrajectorySample)
            }) for sample in message.samples)

    def receive_prediction(self, message):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        identity = prediction_identity(message)
        self.prediction = Prediction(
            identity, message.source_track_revision, stamp_s(message.source_observation_time),
            message.source_observation_time.sec * 1_000_000_000 + message.source_observation_time.nanosec,
            now_s, message.valid, message.source_lifecycle,
            message.source_confidence, message.source_hit_count, message.source_miss_count,
            message.track_id, message.frame_id,
            tuple(PredictionSample(
                sample.dt_s, sample.x_m, sample.y_m, sample.heading_rad, sample.s_m, sample.d_m,
                tuple(sample.covariance_xy), sample.occupancy_half_length_m, sample.occupancy_half_width_m,
            ) for sample in message.samples),
        )

    def receive_state(self, message):
        check=admit_ego_state(message,now_ns=self.get_clock().now().nanoseconds,
                              last_stamp_ns=self._last_state_stamp_ns,
                              last_sequence=self._last_state_sequence)
        if not check.fresh_identity:
            return
        self._last_state_sequence=check.source_sequence
        self._last_state_stamp_ns=check.source_stamp_ns
        self.state=message if check.admissible else None

    def plan(self):
        if (self.nominal is None or self.state is None or
                not 0<=self.get_clock().now().nanoseconds-self._last_state_stamp_ns<=75_000_000):
            self.state=None
            return
        started = time.perf_counter()
        now_s = self.get_clock().now().nanoseconds * 1e-9
        plan = self.supervisor.update(
            self.nominal, self.state.px, self.state.py, self.state.v, self.prediction, now_s,
            blue_yaw=self.state.yaw,
        )
        message = OvertakingPlan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.schema_version = 2
        message.plan_id, message.plan_revision = plan.plan_id, plan.revision
        message.source_prediction_id = plan.source_prediction_id
        message.source_track_revision = plan.source_revision
        if plan.source_time_ns:
            message.source_observation_time.sec = plan.source_time_ns // 1_000_000_000
            message.source_observation_time.nanosec = plan.source_time_ns % 1_000_000_000
        valid_until = now_s + self.settings["plan_validity_s"]
        message.valid_until.sec = int(valid_until)
        message.valid_until.nanosec = int((valid_until % 1.0) * 1e9)
        continuation_until = continuation_deadline_s(
            plan.state, plan.reason, plan.committed_at_s, now_s, valid_until,
            self.settings["committed_continuation_timeout_s"],
        )
        if plan.committed_at_s:
            message.continuation_until.sec = int(continuation_until)
            message.continuation_until.nanosec = int((continuation_until % 1.0) * 1e9)
        message.valid, message.maneuver_state, message.passing_side = plan.valid, plan.state, plan.side
        message.safety_state = ("FAIL_CLOSED" if plan.state in ("ABORT", "BLOCKING_HOLD") else
                                "TEMPORAL_SAFE" if plan.valid else "NOT_ACTIVE")
        message.status_reason = plan.reason
        message.track_id = self.environment["environment_identity"]["active_track"]
        message.frame_id = "map"
        message.map_image_sha256 = self.environment["map"]["image_sha256"]
        message.start_progress_m, message.commit_end_progress_m = plan.start_s, plan.commit_end_s
        message.pass_end_progress_m, message.merge_end_progress_m = plan.pass_end_s, plan.merge_end_s
        message.minimum_predicted_separation_m = plan.minimum_predicted_separation_m
        message.minimum_corridor_clearance_m = plan.minimum_corridor_clearance_m
        message.sample_count = len(plan.samples)
        message.station_period_m = self.supervisor.geometry.length
        message.path_length_m = float(sum(sample.ds_to_next_m for sample in plan.samples))
        for source in plan.samples:
            sample = RacingTrajectorySample()
            for field in fields(source):
                setattr(sample, field.name, getattr(source, field.name))
            message.samples.append(sample)
        if plan.plan_id:
            message.plan_id = overtaking_plan_identity(message)
        message.compute_time_us = (time.perf_counter() - started) * 1e6
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = OvertakingPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
