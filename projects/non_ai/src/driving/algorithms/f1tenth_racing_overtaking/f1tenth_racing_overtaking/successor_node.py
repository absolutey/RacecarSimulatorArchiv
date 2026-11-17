"""Successor R-E v0 ROS runtime adapter.

This adapter preserves the existing R-E -> S0-R -> R-B boundary:
- consumes nominal ActiveTargetTrajectory, R-D OpponentPrediction, Blue state0;
- publishes schema-2 OvertakingPlan only;
- never publishes actuator commands;
- never reads Red truth;
- derives physical authority from frozen Foundation/runtime owners.
"""

import csv
from dataclasses import fields
import math
from pathlib import Path
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import (
    ActiveTargetTrajectory,
    CarState,
    OpponentPrediction,
    OvertakingPlan,
    RacingTrajectorySample,
)
from rclpy.node import Node

from f1tenth_racing_raceline.speed_adapter import maximum_tire_force
from f1tenth_racing_reference.core import (
    overtaking_plan_identity,
    prediction_identity,
)
from f1tenth_racing_reference.node import ACTIVE_REFERENCE_QOS

from .runtime_environment import load_verified_runtime
from .state_input_contract import admit_ego_state
from .post_abort_freshness import accept_post_abort_red_observation
from .successor_core import (
    CommittedSpatiotemporalFrenetPlanner,
    InternalState,
    Prediction,
    PredictionSample,
    SuccessorConfig,
    TrajectorySample,
    WireState,
)


OVERTAKING_PLAN_TOPIC = "overtaking_plan"
ACTIVE_TARGET_TRAJECTORY_TOPIC = "active_target_trajectory"
OPPONENT_PREDICTION_TOPIC = "opponent_prediction"
BLUE_STATE_TOPIC = "state0"

WIRE_STATES_BY_INTERNAL = {
    InternalState.PASS_COMMIT: frozenset({
        WireState.COMMITTED_PASS,
    }),
    InternalState.FROZEN_PASS: frozenset({
        WireState.COMMITTED_PASS,
        WireState.PASSING,
        WireState.ABORT,
    }),
    InternalState.PASS_CONFIRMED: frozenset({
        WireState.MERGE,
    }),
    InternalState.FROZEN_REJOIN: frozenset({
        WireState.MERGE,
        WireState.ABORT,
    }),
    InternalState.NOMINAL: frozenset({
        WireState.COMPLETE,
    }),
}

NON_OVERRIDE_INTERNAL_STATES = frozenset({
    InternalState.TRAIL,
})


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def split_time(value_s):
    sec = math.floor(value_s)
    nanosec = round((value_s - sec) * 1_000_000_000)
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return int(sec), int(nanosec)


def runtime_deadlines(
    plan,
    action,
    now_s,
    plan_validity_s,
    continuation_timeout_s,
    planning_period_s,
    pass_timeout_s=None,
):
    # Return bounded valid/continuation deadlines for one wire publication.
    if action == "PUBLISH_ABORT_DEGRADE":
        deadline = now_s + planning_period_s
        return deadline, deadline

    if plan.committed_at_s > 0.0:
        continuation = plan.committed_at_s + continuation_timeout_s

        if (
            pass_timeout_s is not None
            and getattr(plan, "state", None) in (
                WireState.COMMITTED_PASS.value,
                WireState.PASSING.value,
            )
        ):
            continuation = min(
                continuation,
                plan.committed_at_s + pass_timeout_s,
            )

        if continuation <= now_s:
            return None
        return min(now_s + plan_validity_s, continuation), continuation

    valid_until = now_s + plan_validity_s
    return valid_until, valid_until

def build_successor_config(settings, environment, execution_speed_cap_mps):
    seeds = settings["q0_seed_parameters"]
    dynamics = environment["dynamics"]
    limits = dynamics["limits"]
    footprint = environment["collision"]["footprint_relative_to_base_link"]

    front_force, _ = maximum_tire_force(dynamics["front_tire"])
    rear_force, _ = maximum_tire_force(dynamics["rear_tire"])
    lateral_acceleration = (
        front_force + rear_force
    ) / float(dynamics["mass_kg"])

    # This is a planner limit, not actuation approval. Historical 5.0m/s
    # qualification must not silently replace the simulator's physics bound.
    # H2H actuation remains blocked until the independent dynamic Guard is
    # explicitly qualified; never lift that Guard interlock here.
    if not (math.isfinite(execution_speed_cap_mps) and 0.0 < execution_speed_cap_mps <=
            float(limits['speed_abs_max_m_s'])):
        raise ValueError('SUCCESSOR_CAP_EXCEEDS_PHYSICS_BOUND')

    return SuccessorConfig(
        prediction_max_age_s=float(seeds["prediction_max_age_s"]),
        minimum_track_confidence=float(seeds["minimum_track_confidence"]),
        minimum_confirmed_hits=int(seeds["minimum_confirmed_hits"]),
        maximum_source_misses=int(seeds["maximum_source_misses"]),
        maximum_covariance_eigenvalue_m2=float(
            seeds["maximum_covariance_eigenvalue_m2"]
        ),
        eligibility_ahead_min_m=float(seeds["eligibility_ahead_min_m"]),
        eligibility_ahead_max_m=float(seeds["eligibility_ahead_max_m"]),
        candidate_lateral_offset_m=float(
            seeds["candidate_lateral_offset_m"]
        ),
        entry_length_early_m=float(seeds["entry_length_early_m"]),
        entry_length_late_m=float(seeds["entry_length_late_m"]),
        rejoin_lengths_m=tuple(
            float(value) for value in seeds["rejoin_lengths_m"]
        ),
        corridor_clearance_m=float(seeds["corridor_clearance_m"]),
        blue_tracking_uncertainty_m=float(
            seeds["blue_tracking_uncertainty_m"]
        ),
        qualification_margin_m=float(seeds["qualification_margin_m"]),
        pass_completion_ahead_m=float(seeds["pass_completion_ahead_m"]),
        pass_timeout_s=float(seeds["pass_timeout_s"]),
        delta_v_pass_mps=float(seeds["delta_v_pass_mps"]),
        footprint_half_length_m=max(
            abs(float(footprint["min_x_m"])),
            abs(float(footprint["max_x_m"])),
        ),
        footprint_half_width_m=max(
            abs(float(footprint["min_y_m"])),
            abs(float(footprint["max_y_m"])),
        ),
        wheelbase_m=float(dynamics["dynamic_wheelbase_m"]),
        steering_angle_max_rad=float(
            limits["steering_angle_abs_max_rad"]
        ),
        steering_rate_max_rad_s=float(
            limits["steering_rate_abs_max_rad_s"]
        ),
        lateral_acceleration_max_mps2=float(lateral_acceleration),
        execution_speed_cap_mps=float(execution_speed_cap_mps),
        safe_approach_max_closing_speed_mps=float(
            seeds.get("safe_approach_max_closing_speed_mps", 0.0)),
        longitudinal_acceleration_max_mps2=float(limits["acceleration_max_m_s2"]),
        longitudinal_deceleration_max_mps2=float(limits["deceleration_max_m_s2"]),
        emergency_rejoin_enabled=seeds.get('emergency_rejoin_enabled', False),
    )


class SuccessorOvertakingPlannerNode(Node):
    def __init__(self):
        super().__init__("r_e_successor_overtaking_planner")

        share = Path(
            get_package_share_directory("f1tenth_racing_overtaking")
        )
        sim_share = Path(
            get_package_share_directory("f1tenth_racing_sim")
        )
        settings = yaml.safe_load(
            (share / "config/successor_v0.yaml").read_text()
        )
        self.declare_parameter('environment_file',
                               str(sim_share / 'config/competition_environment.yaml'))
        self.declare_parameter('centerline_file','')
        self.declare_parameter('expected_track_id','')
        self.declare_parameter('expected_map_sha256','')
        environment,rows=load_verified_runtime(
            self.get_parameter('environment_file').value,
            self.get_parameter('centerline_file').value,sim_share,
            expected_track_id=self.get_parameter('expected_track_id').value,
            expected_map_sha256=self.get_parameter('expected_map_sha256').value)
        runtime = settings["runtime_adapter"]

        standalone_cap=runtime["standalone_default_speed_cap_mps"]
        standalone_cap=(float(environment['dynamics']['limits']['speed_abs_max_m_s'])
                        if str(standalone_cap).upper()=='AUTO' else float(standalone_cap))
        self.declare_parameter("qualified_speed_cap_mps", standalone_cap)
        execution_speed_cap = float(
            self.get_parameter("qualified_speed_cap_mps").value
        )
        config = build_successor_config(
            settings,
            environment,
            execution_speed_cap,
        )



        self.environment = environment
        self.settings = settings
        self.planning_rate_hz = float(runtime["planning_rate_hz"])
        self.planning_period_s = 1.0 / self.planning_rate_hz
        self.plan_validity_s = float(runtime["plan_validity_s"])
        self.continuation_timeout_s = float(
            runtime["committed_continuation_timeout_s"]
        )

        self.planner = CommittedSpatiotemporalFrenetPlanner(
            config,
            [(float(row["x_m"]), float(row["y_m"])) for row in rows],
            [float(row["w_tr_left_m"]) for row in rows],
            [float(row["w_tr_right_m"]) for row in rows],
            track_id=environment["environment_identity"]["active_track"],
            frame_id="map",
            map_image_sha256=environment["map"]["image_sha256"],
        )

        self.nominal = None
        self.prediction = None
        self.state = None
        self._state_sequence = -1
        self._state_stamp_ns = -1
        self.awaiting_nominal_release = False
        self.awaiting_abort_release = False
        self.abort_release_started_ns = -1
        self.required_fresh_red_after_abort_ns = None

        self.publisher = self.create_publisher(
            OvertakingPlan,
            OVERTAKING_PLAN_TOPIC,
            1,
        )
        self.create_subscription(
            ActiveTargetTrajectory,
            ACTIVE_TARGET_TRAJECTORY_TOPIC,
            self.receive_reference,
            ACTIVE_REFERENCE_QOS,
        )
        self.create_subscription(
            OpponentPrediction,
            OPPONENT_PREDICTION_TOPIC,
            self.receive_prediction,
            10,
        )
        self.create_subscription(
            CarState,
            BLUE_STATE_TOPIC,
            self.receive_state,
            1,
        )
        self.create_timer(self.planning_period_s, self.plan)

        self.get_logger().info(
            "successor R-E runtime adapter ready; "
            f"qualification_cap={execution_speed_cap:.3f}m/s"
        )

    def receive_reference(self, message):
        if not message.active_reference_valid:
            # A withdrawn S0-R reference must not leave a cached nominal
            # path available for a brand-new H2H commitment. A later fresh
            # qualified NOMINAL publication re-establishes that input.
            self.nominal = None
            return
        if (
            message.active_reference_valid
            and message.source_kind == "NOMINAL"
            and message.sample_count == len(message.samples)
            and message.track_id == self.planner.track_id
            and message.frame_id == self.planner.frame_id
            and message.map_image_sha256 == self.planner.map_image_sha256
        ):
            self.nominal = tuple(
                TrajectorySample(**{
                    field.name: getattr(sample, field.name)
                    for field in fields(TrajectorySample)
                })
                for sample in message.samples
            )
            if self.awaiting_nominal_release:
                if (
                    self.planner.plan is not None
                    and self.planner.plan.state == WireState.COMPLETE.value
                ):
                    self.planner.acknowledge_complete()
                self.awaiting_nominal_release = False
            if (self.awaiting_abort_release and
                    self.planner.plan is not None and
                    message.status_reason == (
                        'H2H_ABORT_RECOVERY_STATIONARY_REJOIN:' +
                        self.planner.plan.identity_digest + ':' +
                        str(self.planner.plan.revision)) and
                    self.planner.plan.state == WireState.ABORT.value):
                # The nominal reference is a fresh, explicit S0-R recovery
                # publication, never the original TRANSIENT_LOCAL replay.
                self.planner.acknowledge_abort_recovery()
                # Never reuse a prediction that existed before ABORT.
                self.prediction = None
                # The new epoch fence survives subsequent Red latched/topic
                # replay; only a measurement genuinely newer than recovery
                # may initiate the next pass opportunity.
                self.required_fresh_red_after_abort_ns = self.get_clock().now().nanoseconds
                self.awaiting_abort_release = False
                self.abort_release_started_ns = -1

    def receive_prediction(self, message):
        source_ns=(int(message.source_observation_time.sec)*1_000_000_000+
                   int(message.source_observation_time.nanosec))
        if not accept_post_abort_red_observation(
                source_ns,self.required_fresh_red_after_abort_ns,
                self.get_clock().now().nanoseconds):
            self.prediction = None
            return
        identity = prediction_identity(message)
        self.prediction = Prediction(
            identity=identity,
            source_revision=message.source_track_revision,
            source_time_s=stamp_s(message.source_observation_time),
            source_time_ns=(
                message.source_observation_time.sec * 1_000_000_000
                + message.source_observation_time.nanosec
            ),
            valid=message.valid,
            lifecycle=message.source_lifecycle,
            confidence=message.source_confidence,
            hit_count=message.source_hit_count,
            miss_count=message.source_miss_count,
            track_id=message.track_id,
            frame_id=message.frame_id,
            horizon_s=message.horizon_s,
            timestep_s=message.timestep_s,
            samples=tuple(
                PredictionSample(
                    dt_s=sample.dt_s,
                    x_m=sample.x_m,
                    y_m=sample.y_m,
                    vx_mps=sample.vx_mps,
                    vy_mps=sample.vy_mps,
                    heading_rad=sample.heading_rad,
                    s_m=sample.s_m,
                    d_m=sample.d_m,
                    covariance_xy=tuple(sample.covariance_xy),
                    occupancy_half_length_m=sample.occupancy_half_length_m,
                    occupancy_half_width_m=sample.occupancy_half_width_m,
                )
                for sample in message.samples
            ),
        )

    def receive_state(self, message):
        # Refuse stale/reordered states before ANY H2H trajectory proposal.
        # A bad newer source invalidates the planner input instead of allowing
        # an indefinitely cached state to drive a new overtaking proposal.
        check=admit_ego_state(message,now_ns=self.get_clock().now().nanoseconds,
                              last_stamp_ns=self._state_stamp_ns,
                              last_sequence=self._state_sequence)
        if not check.fresh_identity:
            return
        self._state_sequence=check.source_sequence
        self._state_stamp_ns=check.source_stamp_ns
        self.state=message if check.admissible else None

    def _inactive_status_message(self, output, now_s, started):
        """Publish planner presence without claiming overtaking authority.

        KEEP_NOMINAL means R-E is alive and intentionally leaves S0-R on the
        nominal reference.  This mirrors the existing R-E wire contract:
        invalid NOMINAL is planner status, not an override payload.
        """
        message = OvertakingPlan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.planner.frame_id
        message.schema_version = 2

        # Do not create or advance a committed-plan identity/revision here.
        message.plan_id = ""
        message.plan_revision = self.planner.revision

        valid_sec, valid_nsec = split_time(
            now_s + self.plan_validity_s
        )
        message.valid_until.sec = valid_sec
        message.valid_until.nanosec = valid_nsec

        message.valid = False
        message.maneuver_state = "NOMINAL"
        message.passing_side = "NONE"
        message.safety_state = "NOT_ACTIVE"
        message.status_reason = output.reason

        message.track_id = self.planner.track_id
        message.frame_id = self.planner.frame_id
        message.map_image_sha256 = self.planner.map_image_sha256

        blue_s = self.planner.geometry.project(
            self.state.px,
            self.state.py,
        ).s
        message.start_progress_m = blue_s
        message.commit_end_progress_m = blue_s
        message.pass_end_progress_m = blue_s
        message.merge_end_progress_m = blue_s

        message.minimum_predicted_separation_m = float("inf")
        message.minimum_corridor_clearance_m = float("inf")
        message.sample_count = 0
        message.station_period_m = self.planner.geometry.length
        message.path_length_m = 0.0
        message.compute_time_us = (
            time.perf_counter() - started
        ) * 1e6
        return message

    def _to_message(self, output, now_s, started):
        plan = output.plan
        deadlines = runtime_deadlines(
            plan,
            output.action,
            now_s,
            self.plan_validity_s,
            self.continuation_timeout_s,
            self.planning_period_s,
            self.planner.config.pass_timeout_s,
        )
        if deadlines is None:
            return None
        valid_until, continuation_until = deadlines

        message = OvertakingPlan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.planner.frame_id
        message.schema_version = 2

        message.plan_revision = plan.revision
        message.source_prediction_id = plan.source_prediction_id
        message.source_track_revision = plan.source_revision
        if plan.source_time_ns:
            message.source_observation_time.sec = (
                plan.source_time_ns // 1_000_000_000
            )
            message.source_observation_time.nanosec = (
                plan.source_time_ns % 1_000_000_000
            )

        valid_sec, valid_nsec = split_time(valid_until)
        message.valid_until.sec = valid_sec
        message.valid_until.nanosec = valid_nsec
        continuation_sec, continuation_nsec = split_time(
            continuation_until
        )
        message.continuation_until.sec = continuation_sec
        message.continuation_until.nanosec = continuation_nsec

        message.valid = plan.valid
        message.maneuver_state = plan.state
        message.passing_side = plan.side
        message.safety_state = (
            "FAIL_CLOSED"
            if plan.state in (
                WireState.ABORT.value,
                WireState.BLOCKING_HOLD.value,
            )
            else "TEMPORAL_SAFE"
            if plan.valid
            else "NOT_ACTIVE"
        )
        message.status_reason = plan.reason
        message.track_id = plan.track_id
        message.frame_id = plan.frame_id
        message.map_image_sha256 = plan.map_image_sha256
        message.start_progress_m = plan.start_s
        message.commit_end_progress_m = plan.commit_end_s
        message.pass_end_progress_m = plan.pass_end_s
        message.merge_end_progress_m = plan.merge_end_s
        message.minimum_predicted_separation_m = (
            plan.minimum_predicted_separation_m
        )
        message.minimum_corridor_clearance_m = (
            plan.minimum_corridor_clearance_m
        )
        message.sample_count = len(plan.samples)
        message.station_period_m = self.planner.geometry.length
        message.path_length_m = float(
            sum(sample.ds_to_next_m for sample in plan.samples)
        )

        for source in plan.samples:
            sample = RacingTrajectorySample()
            for field in fields(TrajectorySample):
                setattr(
                    sample,
                    field.name,
                    getattr(source, field.name),
                )
            message.samples.append(sample)

        message.plan_id = overtaking_plan_identity(message)
        message.compute_time_us = (
            time.perf_counter() - started
        ) * 1e6
        return message

    def plan(self):
        if (
            self.nominal is None
            or self.state is None
            or self.awaiting_nominal_release
            or self.awaiting_abort_release
        ):
            return

        now_ns = self.get_clock().now().nanoseconds
        if (self._state_stamp_ns<=0 or
                not 0<=now_ns-self._state_stamp_ns<=75_000_000):
            self.state=None
            return  # S0-R and Guard expire unsafe authority independently.
        started = time.perf_counter()
        now_s = now_ns * 1e-9
        output = self.planner.update(
            self.nominal,
            self.state.px,
            self.state.py,
            self.state.v,
            self.prediction,
            now_s,
            blue_yaw=self.state.yaw,
        )

        if output.plan is None:
            if output.action == "KEEP_NOMINAL":
                self.publisher.publish(
                    self._inactive_status_message(
                        output,
                        now_s,
                        started,
                    )
                )
            else:
                self.get_logger().warning(
                    "successor returned no plan for action: "
                    + output.action
                )
            return
        if output.action in (
            "CONTINUE_FROZEN",
            "ABORT_LATCHED_NO_REPUBLISH",
            "AWAIT_COMPLETE_PUBLICATION",
        ):
            return
        if output.action not in (
            "PUBLISH_OVERRIDE",
            "PUBLISH_ABORT_DEGRADE",
            "RELEASE_TO_NOMINAL",
        ):
            self.get_logger().warning(
                f"successor output action not publishable: {output.action}"
            )
            return

        message = self._to_message(
            output,
            now_s,
            started,
        )
        if message is None:
            self.get_logger().warning(
                "successor committed continuation deadline reached"
            )
            return

        self.publisher.publish(message)

        if output.action == "PUBLISH_ABORT_DEGRADE":
            self.awaiting_abort_release = True
            self.abort_release_started_ns = self.get_clock().now().nanoseconds
        if output.action == "RELEASE_TO_NOMINAL":
            self.awaiting_nominal_release = True


def main():
    rclpy.init()
    node = SuccessorOvertakingPlannerNode()
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
