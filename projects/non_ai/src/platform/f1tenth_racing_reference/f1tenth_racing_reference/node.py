import json
import math
import time
from dataclasses import replace
from pathlib import Path
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

from control_msgs.msg import ActiveTargetTrajectory, OpponentPrediction, OvertakingPlan, CarState
from std_msgs.msg import Bool
from f1tenth_racing_raceline import load_racing_trajectory

from .core import (invalid_active_reference, matching_complete,
                   override_expiry_action, prediction_identity,
                   select_nominal, select_override)
from .transport import to_message
from .contract_guard import load_nominal_policy
from .abort_recovery_gate import evaluate_abort_recovery


ACTIVE_REFERENCE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class NominalActiveReferenceNode(Node):
    def __init__(self):
        super().__init__("s0_active_reference")
        self.declare_parameter("trajectory_candidate", "baseline")
        self.declare_parameter("environment_file", "")
        self.declare_parameter("allow_preliminary_spielberg", False)
        self.declare_parameter("nominal_policy_file", "")
        self.declare_parameter("raceline_data_dir", "")
        self.declare_parameter("expected_track_id", "iccas2025")
        self.declare_parameter("expected_map_sha256", "")
        self.declare_parameter("prediction_max_age_s", 0.5)
        self.declare_parameter("committed_continuation_max_s", 15.0)
        # An explicit launch-time veto. Observer-only R-E messages never
        # gain active-reference authority. Normal profiles preserve old behavior.
        self.declare_parameter("accept_overtaking_plans", True)
        self._accept_overtaking_plans = self.get_parameter("accept_overtaking_plans").value is True
        publisher = self.create_publisher(
            ActiveTargetTrajectory,
            "active_target_trajectory",
            ACTIVE_REFERENCE_QOS,
        )
        try:
            configured_policy = self.get_parameter("nominal_policy_file").value
            policy_path = (Path(configured_policy) if configured_policy else Path(
                get_package_share_directory("f1tenth_racing_reference")
            ).joinpath("config", "nominal_reference.json"))
            configured_data = self.get_parameter("raceline_data_dir").value
            raceline_data = (Path(configured_data) if configured_data else Path(
                get_package_share_directory("f1tenth_racing_raceline")
            ).joinpath("data"))
            # Verify policy path, every referenced artifact, and declared track/map SHA.
            expected_track = self.get_parameter("expected_track_id").value
            expected_map = self.get_parameter("expected_map_sha256").value
            raw_policy = json.loads(policy_path.read_text())
            if not expected_map:
                expected_map = raw_policy["map_image_sha256"]
            policy = load_nominal_policy(
                policy_path, track_id=expected_track,
                map_sha256=expected_map, data_dir=raceline_data)
            candidate_name = self.get_parameter("trajectory_candidate").value
            candidate = policy["candidates"][candidate_name]
            active = select_nominal(
                load_racing_trajectory(
                    raceline_data / candidate["csv"],
                    raceline_data / candidate["metadata"],
                ),
                expected_track_id=policy["track_id"],
                expected_frame_id=policy["frame_id"],
                expected_map_image_sha256=policy["map_image_sha256"],
                expected_source_trajectory_id=candidate["source_trajectory_id"],
                allow_preliminary_spielberg=bool(self.get_parameter(
                    "allow_preliminary_spielberg").value),
            )
        except Exception as error:  # Startup boundary: publish invalid status, never payload.
            active = invalid_active_reference(f"NOMINAL_LOAD_FAILED: {error}")
        self._publisher = publisher
        self._nominal = self._active = active
        self._plan = None
        self._expired_plan = None
        self._predictions = {}
        # Only the accepted ABORT path may use this recovery gate. Never apply
        # nominal while moving/outside its corridor or without fresh Red proof.
        self._recovery_state = None
        self._recovery_collision_ego = None
        self._recovery_collision_red = None
        self._recovery_collision_ego_steady_ns = None
        self._recovery_collision_red_steady_ns = None
        self._recovery_prediction = None
        self._recovery_last_sequence = -1
        self._recovery_good_ticks = 0
        self._recovery_evidence = "NOT_ATTEMPTED"
        try:
            sim_share = Path(get_package_share_directory('f1tenth_racing_sim'))
            env_path = self.get_parameter('environment_file').value or str(
                sim_share / 'config/competition_environment.yaml')
            env = yaml.safe_load(Path(env_path).read_text())
            if (env['environment_identity']['active_track'] != expected_track or
                    env['map']['image_sha256'] != expected_map):
                raise ValueError('ABORT_RECOVERY_ENVIRONMENT_MISMATCH')
            fp = env['collision']['footprint_relative_to_base_link']
            self._recovery_radius = max(
                math.hypot(x,y) for x in (float(fp['min_x_m']),float(fp['max_x_m']))
                for y in (float(fp['min_y_m']),float(fp['max_y_m']))) + 0.03
        except Exception as error:
            self._recovery_radius = None
            self.get_logger().warning('ABORT_RECOVERY_UNAVAILABLE: '+str(error))
        self.create_subscription(CarState, 'state0', self.receive_recovery_state, 1)
        self.create_subscription(Bool, 'collision0', self.receive_ego_collision, 10)
        self.create_subscription(Bool, 'vehicle_collision', self.receive_red_collision, 10)
        self.create_subscription(OpponentPrediction, "opponent_prediction", self.receive_prediction, 10)
        self.create_subscription(OvertakingPlan, "overtaking_plan", self.receive_plan, 10)
        self.create_timer(0.1, self.expire_override)
        self.publish(active, force=True)

    def publish(self, active, force=False):
        if not force and (active.active_reference_id, active.active_revision) == (
                self._active.active_reference_id, self._active.active_revision):
            return
        self._active = active
        self._publisher.publish(to_message(active))
        self.get_logger().info(json.dumps({
            "active_reference_id": active.active_reference_id,
            "active_revision": active.active_revision,
            "sample_count": active.sample_count,
            "source_kind": active.source_kind,
            "source_trajectory_id": active.source_trajectory_id,
            "status_reason": active.status_reason,
            "valid": active.active_reference_valid,
        }, sort_keys=True))

    def receive_plan(self, plan):
        # Deliberately do not even evaluate CANCEL/COMPLETE in observer mode;
        # R-E is visible on its topic but has zero S0-R actuation authority.
        if not self._accept_overtaking_plans:
            return
        now_ns = self.get_clock().now().nanoseconds
        # A cancellation/COMPLETE is governed by matched plan identity, not
        # by an active lease. Do not read a valid_until field on this branch.
        if not plan.valid:
            accepted = self._plan or self._expired_plan
            if matching_complete(plan, accepted):
                # COMPLETE is *not* a bypass around the ABORT fail-closed
                # recovery gate. A corresponding COMPLETE from R-E only
                # acknowledges the old plan, not safe vehicle alignment or
                # Red swept occupancy. Keep the expired ABORT identity so
                # the parked S0-R gate can prove a new nominal publication.
                if accepted.maneuver_state == 'ABORT':
                    self._plan = None
                    self._expired_plan = accepted
                    self._recovery_good_ticks = 0
                    self._recovery_last_sequence = -1
                    self.publish(invalid_active_reference(
                        'H2H_ABORT_COMPLETE_NOT_AUTHORIZED_FOR_NOMINAL'))
                elif accepted.maneuver_state == 'BLOCKING_HOLD':
                    self._plan = None
                    self._expired_plan = accepted
                    self.publish(invalid_active_reference(
                        'H2H_BLOCKING_HOLD_COMPLETE_REQUIRES_NEW_SAFETY_PLAN'))
                else:
                    self._plan = None
                    self._expired_plan = None
                    self.publish(self._nominal)
            return
        # An active override alone requires a live lease. Expired plans must
        # never reach override selection or gain actuation authority.
        valid_until_ns = plan.valid_until.sec * 1_000_000_000 + plan.valid_until.nanosec
        if valid_until_ns <= now_ns:
            return
        if plan.maneuver_state == "ABORT" and self._plan is None:
            return
        if (self._expired_plan is not None and
                self._expired_plan.maneuver_state == 'ABORT'):
            self.get_logger().warning('ABORT_LATCHED_NEW_PLAN_REQUIRES_PARKED_RECOVERY')
            return
        active = select_override(
            self._nominal, plan, self._predictions.get(plan.source_prediction_id), now_ns * 1e-9,
            self.get_parameter("prediction_max_age_s").value,
            self.get_parameter("committed_continuation_max_s").value,
            self._plan,
        )
        if active.active_reference_valid:
            self._plan = plan
            self._expired_plan = None
            self.publish(active)
        else:
            self.get_logger().warning(active.status_reason)

    def receive_prediction(self, prediction):
        now_ns = self.get_clock().now().nanoseconds
        maximum_age_ns = round(self.get_parameter("prediction_max_age_s").value * 1_000_000_000)
        source_ns = (prediction.source_observation_time.sec * 1_000_000_000 +
                     prediction.source_observation_time.nanosec)
        if (not 0 <= now_ns - source_ns <= maximum_age_ns or
                prediction.track_id != self._nominal.track_id or
                prediction.frame_id != self._nominal.frame_id):
            # A future sample must not survive even briefly in the override
            # cache, and must not replace the last fresh Red recovery witness.
            self._predictions = {
                identity: message for identity, message in self._predictions.items()
                if 0 <= now_ns - (message.source_observation_time.sec*1_000_000_000 +
                         message.source_observation_time.nanosec) <= maximum_age_ns}
            return
        self._recovery_prediction = prediction
        self._predictions[prediction_identity(prediction)] = prediction
        self._predictions = {
            identity: message for identity, message in self._predictions.items()
            if 0 <= now_ns - (message.source_observation_time.sec * 1_000_000_000 +
                         message.source_observation_time.nanosec) <= maximum_age_ns
        }

    def receive_recovery_state(self, message):
        # The last two samples must have monotonically increasing identity.
        if (self._recovery_state is None or
                (int(message.physics_sequence) > int(self._recovery_state.physics_sequence) and
                 (message.physics_stamp.sec, message.physics_stamp.nanosec) >
                 (self._recovery_state.physics_stamp.sec,
                  self._recovery_state.physics_stamp.nanosec))):
            self._recovery_state = message

    def receive_ego_collision(self, message):
        self._recovery_collision_ego_steady_ns = time.monotonic_ns()
        self._recovery_collision_ego = (
            bool(message.data) or bool(self._recovery_collision_ego))

    def receive_red_collision(self, message):
        self._recovery_collision_red_steady_ns = time.monotonic_ns()
        self._recovery_collision_red = (
            bool(message.data) or bool(self._recovery_collision_red))

    def _attempt_abort_recovery(self):
        # S0-R cannot issue braking or an actuation certificate. It may
        # re-publish nominal ONLY after the independent parked-state/Red gate.
        if (self._expired_plan is None or
                self._expired_plan.maneuver_state != 'ABORT' or
                self._active.active_reference_valid or
                self._recovery_radius is None or
                self._nominal.trajectory is None):
            return
        state = self._recovery_state
        red = self._recovery_prediction
        if (red is not None and
                (red.track_id != self._nominal.track_id or
                 red.frame_id != self._nominal.frame_id)):
            red = None
        steady_now = time.monotonic_ns()
        collision_fresh = (
            self._recovery_collision_ego_steady_ns is not None and
            self._recovery_collision_red_steady_ns is not None and
            0 <= steady_now - self._recovery_collision_ego_steady_ns <= 250_000_000 and
            0 <= steady_now - self._recovery_collision_red_steady_ns <= 250_000_000)
        result = evaluate_abort_recovery(
            self._nominal.trajectory.samples, state, red,
            now_ns=self.get_clock().now().nanoseconds,
            ego_radius_m=self._recovery_radius,
            collision_known=(collision_fresh and
                             self._recovery_collision_ego is not None and
                             self._recovery_collision_red is not None),
            collision_active=bool(self._recovery_collision_ego or
                                  self._recovery_collision_red),
        )
        self._recovery_evidence = result.reason
        if not result.allowed:
            self._recovery_good_ticks = 0
            return
        sequence = int(state.physics_sequence)
        if sequence <= self._recovery_last_sequence:
            return
        self._recovery_last_sequence = sequence
        self._recovery_good_ticks += 1
        # Three distinct, increasing state ticks after ABORT expiration.
        if self._recovery_good_ticks < 3:
            return
        abort_plan = self._expired_plan
        self._expired_plan = None
        self._recovery_good_ticks = 0
        self.publish(replace(
            self._nominal,
            status_reason=('H2H_ABORT_RECOVERY_STATIONARY_REJOIN:' +
                           abort_plan.plan_id + ':' + str(abort_plan.plan_revision)),
        ))
        self.get_logger().warning(
            'H2H_ABORT_RECOVERY_NOMINAL_PROPOSAL_ONLY: '
            'independent Guard must certify all subsequent commands')

    def expire_override(self):
        if self._plan is not None and self._active.source_kind != "NOMINAL":
            action = override_expiry_action(
                self._plan, self.get_clock().now().nanoseconds)
            if action == "DEGRADE":
                self._expired_plan = self._plan
                self._plan = None
                self._recovery_last_sequence = -1
                self._recovery_good_ticks = 0
                self.publish(invalid_active_reference(
                    ('H2H_ABORT_CONTINUATION_EXPIRED:' + self._expired_plan.plan_id)
                    if self._expired_plan.maneuver_state == 'ABORT'
                    else 'COMMITTED_CONTINUATION_EXPIRED'))
        self._attempt_abort_recovery()


def main():
    rclpy.init()
    node = NominalActiveReferenceNode()
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
