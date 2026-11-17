import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from control_msgs.msg import (ActuatorIntent, ActuatorGuardStatus, ActiveTargetTrajectory, CarState, ControllerStatus,
                              OpponentPrediction, OpponentState, OvertakingPlan)
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, Int32, String

from f1tenth_racing_geometry import ClosedTrackGeometry, interpolate_periodic_scalar
from f1tenth_racing_raceline import load_racing_trajectory

from .storage import RunWriter
from .review_validity import LapCollisionReview


BAG_TOPICS = (
    "state0", "state1", "ackermann_cmd0", "active_target_trajectory", "controller_status0",
    "opponent_state", "opponent_prediction", "overtaking_plan", "race_readiness",
    "race_start_time", "lap_count0", "lap_time0", "race_valid",
    "wall_collision_count0", "collision0", "vehicle_collision",
    "actuator_deadman0", "actuator_deadman1", "controller_execution_receipt",
    "actuator_intent0", "actuator_guard_status0", "actuator_applied_receipt0",
    "actuator_applied_receipt1",
)


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def git_identity(cwd):
    try:
        head = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=cwd, text=True, stderr=subprocess.DEVNULL,
        ).strip()
        tree = subprocess.check_output(
            ["git", "rev-parse", "HEAD^{tree}"], cwd=cwd, text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
        return head, tree
    except (OSError, subprocess.CalledProcessError):
        return "UNAVAILABLE", "UNAVAILABLE"


def file_identity(path):
    path = Path(path)
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def r_b_dynamic_fields(state, steering_command, acceleration_command,
                       steering_limit, acceleration_limit, braking_limit):
    return {
        "blue_body_vx_mps": state.vx,
        "blue_body_vy_mps": state.vy,
        "blue_yaw_rate_rad_s": state.omega,
        "blue_slip_angle_rad": state.slip_angle,
        "blue_longitudinal_acceleration_mps2": state.ax,
        "blue_lateral_acceleration_mps2": state.ay,
        "blue_steering_angle_rad": state.steer,
        "blue_steering_rate_rad_s": state.steer_vel,
        "r_b_steering_saturated": math.isclose(
            abs(steering_command), steering_limit, rel_tol=0.0, abs_tol=1e-9,
        ),
        "r_b_acceleration_saturated": (
            math.isclose(acceleration_command, acceleration_limit,
                         rel_tol=0.0, abs_tol=1e-9) or
            math.isclose(acceleration_command, -braking_limit,
                         rel_tol=0.0, abs_tol=1e-9)
        ),
    }


def stop_child(process):
    if process is None or process.poll() is not None:
        return None if process is None else process.returncode
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10.0)
    except ProcessLookupError:
        pass
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)
    return process.poll()


class FlightRecorder(Node):
    """Passive subscriber. It intentionally creates no ROS publisher or service."""

    def __init__(self):
        super().__init__("h2h_flight_recorder")
        self.declare_parameter("run_root", "reports/runs")
        self.declare_parameter("run_id", "")
        self.declare_parameter("trajectory_candidate", "baseline")
        self.declare_parameter("nominal_policy_file", "")
        self.declare_parameter("raceline_data_dir", "")
        self.declare_parameter("environment_file", "")
        self.declare_parameter("expected_track_id", "iccas2025")
        self.declare_parameter("expected_map_sha256", "")
        self.declare_parameter("record_rosbag", True)
        self.declare_parameter("review_phase", "FOUNDATION")
        self.declare_parameter("execution_speed_cap_mps", 2.0)
        self.declare_parameter("fallback_speed_cap_mps", 5.6)
        self.declare_parameter("controller_kind", "pursuit")
        self.declare_parameter("controller_profile", "default")
        self.declare_parameter("candidate_id", "B0")
        self.declare_parameter("controller_config_sha256", "")
        self.declare_parameter("solver_formulation_id", "NONE")
        self.declare_parameter("solver_horizon_steps", 0)
        self.declare_parameter("solver_timestep_s", 0.0)
        self.declare_parameter("solver_deadline_s", 0.0)
        review_phase = self.get_parameter("review_phase").value.upper()
        if review_phase not in ("FOUNDATION", "R-A", "R-B", "R-C", "R-D"):
            raise ValueError("invalid review_phase")
        now_ns = time.time_ns()
        run_id = self.get_parameter("run_id").value or (
            f"{review_phase}-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime(now_ns / 1e9))}"
            f"-{now_ns % 1_000_000_000:09d}"
        )
        run_dir = Path(self.get_parameter("run_root").value).resolve() / run_id

        reference_share = Path(get_package_share_directory("f1tenth_racing_reference"))
        raceline_share = Path(get_package_share_directory("f1tenth_racing_raceline"))
        sim_share = Path(get_package_share_directory("f1tenth_racing_sim"))
        policy_path = (Path(self.get_parameter("nominal_policy_file").value)
                       if self.get_parameter("nominal_policy_file").value
                       else reference_share / "config/nominal_reference.json")
        policy = json.loads(policy_path.read_text())
        if (policy.get("track_id") != self.get_parameter("expected_track_id").value or
            (self.get_parameter("expected_map_sha256").value and
             policy.get("map_image_sha256") != self.get_parameter("expected_map_sha256").value)):
            raise ValueError("Recorder reference map/track identity mismatch")
        candidate_name = self.get_parameter("trajectory_candidate").value
        candidate = policy["candidates"][candidate_name]
        data_dir = (Path(self.get_parameter("raceline_data_dir").value)
                    if self.get_parameter("raceline_data_dir").value else raceline_share / "data")
        csv_path = data_dir / candidate["csv"]
        metadata_path = data_dir / candidate["metadata"]
        environment_path = (Path(self.get_parameter("environment_file").value)
            if self.get_parameter("environment_file").value
            else sim_share / "config/competition_environment.yaml")
        environment = yaml.safe_load(environment_path.read_text())
        limits = environment["dynamics"]["limits"]
        controller_metadata = {
            "controller_kind": self.get_parameter("controller_kind").value,
            "controller_profile": self.get_parameter("controller_profile").value,
            "candidate_id": self.get_parameter("candidate_id").value,
            "source_config_sha256": self.get_parameter("controller_config_sha256").value,
            "execution_speed_cap_mps": self.get_parameter("execution_speed_cap_mps").value,
            "fallback_speed_cap_mps": self.get_parameter("fallback_speed_cap_mps").value,
            "solver_formulation_id": self.get_parameter("solver_formulation_id").value,
            "solver_horizon_steps": self.get_parameter("solver_horizon_steps").value,
            "solver_timestep_s": self.get_parameter("solver_timestep_s").value,
            "solver_deadline_s": self.get_parameter("solver_deadline_s").value,
        }
        controller_config_hash = hashlib.sha256(
            json.dumps(controller_metadata, sort_keys=True).encode()
        ).hexdigest()
        self.nominal = load_racing_trajectory(csv_path, metadata_path)
        self.track = ClosedTrackGeometry([(sample.x_m, sample.y_m) for sample in self.nominal.samples])
        self.nominal_s = tuple(sample.s_m for sample in self.nominal.samples)
        self.nominal_speed = tuple(sample.target_speed_mps for sample in self.nominal.samples)
        self.nominal_accel = tuple(sample.target_accel_mps2 for sample in self.nominal.samples)
        head, tree = git_identity(Path.cwd())
        self.writer = RunWriter(run_dir, {
            "schema_version": 1,
            "run_id": run_id,
            "created_unix_s": time.time(),
            "pid": os.getpid(),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            "head": head,
            "tree": tree,
            "trajectory_candidate": candidate_name,
            "review_phase": review_phase,
            "lifecycle": "INITIALIZING",
            "legacy_r_e_status": "HISTORICAL_DISABLED",
            "trajectory_id": self.nominal.trajectory_id,
            "truth_boundary": "state1 fields are EVALUATOR_ONLY_TRUTH",
            "control_publications": [],
            "controller": controller_metadata,
            "controller_config_hash": controller_config_hash,
            "inputs": [file_identity(path) for path in (
                policy_path, csv_path, metadata_path,
                environment_path,
            )],
            "bag_topics": BAG_TOPICS,
        })
        self.run_dir = run_dir
        self.review_phase = review_phase
        self.execution_speed_cap_mps = self.get_parameter("execution_speed_cap_mps").value
        self.controller_kind = controller_metadata["controller_kind"]
        self.solver_formulation_id = controller_metadata["solver_formulation_id"]
        self.solver_horizon_steps = controller_metadata["solver_horizon_steps"]
        self.solver_timestep_s = controller_metadata["solver_timestep_s"]
        self.solver_deadline_s = controller_metadata["solver_deadline_s"]
        self.steering_limit_rad = limits["steering_angle_abs_max_rad"]
        self.acceleration_limit_mps2 = limits["acceleration_max_m_s2"]
        self.braking_limit_mps2 = limits["deceleration_max_m_s2"]
        if self.review_phase == "R-A":
            self.writer.r_a_trajectory(self.nominal)
        self.bag = None
        if self.get_parameter("record_rosbag").value:
            self.bag = subprocess.Popen(
                ["ros2", "bag", "record", "-o", str(run_dir / "rosbag2"), *BAG_TOPICS],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )

        self.blue = self.red = self.active = self.opponent = self.prediction = self.plan = None
        self.blue_projection = self.red_projection = self.opponent_s = None
        self.last_control_time = None
        self.last_plan_event = self.last_source = self.last_readiness = None
        self.last_prediction_event = None
        self.last_qualified_opponent_relation = "UNKNOWN"
        self.follow_episode_started_s = None
        self.last_fallback = False
        self.command_gap_count = 0
        self.was_behind = False
        self.pass_count = self.lap = self.wall_collisions = 0
        self.last_lap_time = 0.0
        self.race_valid = True
        self.car_collision = False
        self.review_validity = LapCollisionReview()
        self.collision_episodes = []
        self.start_time = None
        self.recording_started = False
        self.controller_runtime_recorded = False
        self.closed = False
        transient = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # R-B is BEST_EFFORT; an integer-depth subscription requests RELIABLE
        # and cannot connect to this publisher. Recorder does not actuate.
        intent_qos = QoSProfile(depth=50,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(ActuatorIntent,"actuator_intent0",
                                 self.receive_intent,intent_qos)
        self.create_subscription(ActuatorGuardStatus,"actuator_guard_status0",self.receive_guard,10)
        self.create_subscription(String,"actuator_applied_receipt0",self.receive_applied,10)
        self.create_subscription(CarState, "state0", self.receive_blue, 1)
        self.create_subscription(CarState, "state1", self.receive_red, 1)
        self.create_subscription(ActiveTargetTrajectory, "active_target_trajectory", self.receive_active, transient)
        self.create_subscription(ControllerStatus, "controller_status0", self.receive_control, 20)
        self.create_subscription(String, "controller_execution_receipt",
                                 self.receive_execution_receipt, transient)
        self.create_subscription(OpponentState, "opponent_state", self.receive_opponent, 10)
        self.create_subscription(OpponentPrediction, "opponent_prediction", self.receive_prediction, 10)
        self.create_subscription(OvertakingPlan, "overtaking_plan", self.receive_plan, 10)
        self.create_subscription(String, "race_readiness", self.receive_readiness, transient)
        self.create_subscription(Time, "race_start_time", self.receive_start, transient)
        self.create_subscription(Int32, "lap_count0", self.receive_lap, 1)
        self.create_subscription(Float64, "lap_time0", self.receive_lap_time, 10)
        self.create_subscription(Bool, "race_valid", self.receive_valid, 1)
        self.create_subscription(Int32, "wall_collision_count0", self.receive_wall, 1)
        self.create_subscription(Bool, "collision0", self.receive_wall_level, 1)
        self.create_subscription(Bool, "vehicle_collision", self.receive_collision, 1)
        self.writer.set_prearmed()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def receive_intent(self, msg):
        self.emit("ACTUATOR","INTENT",f"{msg.source_id}:{msg.sequence}:{msg.candidate_mode}",
                  msg.active_reference_id,msg.active_revision)

    def receive_guard(self, msg):
        if not msg.ready or msg.emergency_unrecoverable:
            self.emit("ACTUATOR_GUARD",msg.mode,msg.reason,msg.active_reference_id)

    def receive_applied(self,msg):
        self.emit("SIMULATOR","COMMAND_APPLIED",msg.data)

    def emit(self, owner, event, reason="", plan_id="", revision=0, timestamp_s=None):
        timestamp_s = self.now() if timestamp_s is None else timestamp_s
        if self.start_time is None or timestamp_s < self.start_time:
            return
        self.writer.event(
            timestamp_s, timestamp_s - self.start_time,
            owner, event, reason, plan_id, revision,
        )

    def receive_blue(self, message):
        self.blue = message
        self.blue_projection = self.track.project(message.px, message.py)
        self.check_pass()
        if self.review_phase == "FOUNDATION":
            now_s = self.now()
            if self.start_time is not None and now_s >= self.start_time:
                self.writer.telemetry({
                    "timestamp_s": now_s,
                    "run_elapsed_s": now_s - self.start_time,
                    "blue_x_m": message.px,
                    "blue_y_m": message.py,
                    "blue_yaw_rad": message.yaw,
                    "blue_s_m": self.blue_projection.s,
                    "blue_d_m": self.blue_projection.d,
                    "blue_actual_speed_mps": message.v,
                    "red_truth_x_m": self.red.px if self.red else math.nan,
                    "red_truth_y_m": self.red.py if self.red else math.nan,
                    "red_truth_speed_mps": self.red.v if self.red else math.nan,
                    "lap": self.lap,
                    "lap_time_s": self.last_lap_time,
                    "race_valid": self.race_valid,
                    "wall_collisions": self.wall_collisions,
                    "car_collision": self.car_collision,
                })

    def receive_red(self, message):
        self.red = message
        self.red_projection = self.track.project(message.px, message.py)
        self.check_pass()

    def receive_execution_receipt(self, message):
        try:
            data=json.loads(message.data)
            if data.get("schema_version") != 1:
                return
            self.emit("R-B", "EXECUTION_PROVENANCE", data.get("event", "UNKNOWN"),
                      data.get("active_reference_id", ""),
                      int(data.get("active_revision", 0)))
        except (ValueError,TypeError,KeyError):
            return

    def receive_active(self, message):
        previous = self.last_source
        self.active = message
        self.last_source = message.source_kind
        if message.source_kind != previous:
            if message.source_kind == "OVERTAKE_OVERRIDE":
                self.emit("S0-R", "INTERACTION_CAP_ENGAGED", message.status_reason,
                          message.active_reference_id, message.active_revision)
            elif previous == "OVERTAKE_OVERRIDE" and message.source_kind == "NOMINAL":
                self.emit("S0-R", "INTERACTION_CAP_RELEASED", message.status_reason,
                          message.active_reference_id, message.active_revision)

    def receive_opponent(self, message):
        self.opponent = message
        self.opponent_s = self.track.project(message.x_m, message.y_m).s

    def receive_prediction(self, message):
        self.prediction = message
        event = (message.valid, message.status_reason)
        if not message.valid and event != self.last_prediction_event:
            self.emit("R-D", "PREDICTION_INVALID", message.status_reason)
        self.last_prediction_event = event

    def receive_plan(self, message):
        previous_state = self.plan.maneuver_state if self.plan else ""
        self.plan = message
        event = (message.maneuver_state, message.status_reason, message.plan_id, message.plan_revision)
        if event == self.last_plan_event:
            return
        self.last_plan_event = event
        mapped = {
            "PREPARE": "PREPARE", "COMMITTED_PASS": "COMMIT", "PASSING": "PASSING",
            "MERGE": "MERGE", "COMPLETE": "COMPLETE", "ABORT": "ABORT",
            "BLOCKING_HOLD": "MOVING_CONTAINMENT",
        }.get(message.maneuver_state)
        if mapped:
            self.emit("R-E", mapped, message.status_reason, message.plan_id, message.plan_revision)
        if message.maneuver_state == "BLOCKING_HOLD" and previous_state != "BLOCKING_HOLD":
            self.follow_episode_started_s = self.now()
            self.emit("R-E", "FOLLOW_EPISODE_START", message.status_reason,
                      message.plan_id, message.plan_revision)
        elif previous_state == "BLOCKING_HOLD" and message.maneuver_state != "BLOCKING_HOLD":
            duration = (0.0 if self.follow_episode_started_s is None else
                        self.now() - self.follow_episode_started_s)
            self.emit("R-E", "FOLLOW_EPISODE_END", f"{duration:.6f}s",
                      message.plan_id, message.plan_revision)
            if message.maneuver_state == "COMMITTED_PASS":
                self.emit("R-E", "PASS_COMMIT_AFTER_FOLLOW", f"{duration:.6f}s",
                          message.plan_id, message.plan_revision)
            self.follow_episode_started_s = None
        if "NO_SAFE_CANDIDATE" in message.status_reason:
            self.emit("R-E", "NO_SAFE_CANDIDATE", message.status_reason,
                      message.plan_id, message.plan_revision)

    def receive_readiness(self, message):
        if message.data != self.last_readiness:
            self.last_readiness = message.data
            if message.data == "READY":
                self.emit("COORDINATOR", "READY", message.data)

    def receive_start(self, message):
        self.start_time = stamp_s(message)
        self.recording_started = False
        self.last_control_time = None
        self.last_fallback = False
        self.follow_episode_started_s = None
        self.command_gap_count = 0
        self.was_behind = False
        self.pass_count = self.lap = self.wall_collisions = 0
        self.last_lap_time = 0.0
        self.race_valid = True
        self.car_collision = False
        self.review_validity = LapCollisionReview()
        self.collision_episodes = []
        self.writer.set_start_time(self.start_time)
        self.emit("COORDINATOR", "START", "SYNCHRONIZED_RELEASE",
                  timestamp_s=self.start_time)

    def receive_lap_time(self, message):
        self.last_lap_time = message.data

    def receive_lap(self, message):
        self.lap = message.data
        self.review_validity.set_lap(message.data)

    def receive_valid(self, message):
        if self.race_valid and not message.data:
            self.emit("EVALUATOR", "RACE_INVALID", "EVALUATOR_ONLY_TRUTH")
        self.race_valid = message.data

    def receive_wall(self, message):
        if message.data > self.wall_collisions:
            self.emit("EVALUATOR", "WALL_COLLISION", "EVALUATOR_ONLY_TRUTH")
        self.wall_collisions = message.data

    def receive_wall_level(self, message):
        self.update_collision_episode(wall=message.data)

    def receive_collision(self, message):
        self.update_collision_episode(vehicle=message.data)
        if message.data and not self.car_collision:
            self.emit("EVALUATOR", "CAR_COLLISION", "EVALUATOR_ONLY_TRUTH")
        self.car_collision = self.car_collision or message.data

    def update_collision_episode(self, **level):
        if self.start_time is None:
            return
        transition = self.review_validity.update(**level)
        now_s = self.now()
        if transition is not None and transition.kind == "ENTER":
            self.collision_episodes.append({
                "episode_id": transition.episode_id,
                "enter_timestamp_s": now_s,
                "enter_run_elapsed_s": max(0.0, now_s - self.start_time),
                "lap": self.lap,
                "wall_collision": transition.wall,
                "vehicle_collision": transition.vehicle,
                "cumulative_wall_episodes": self.review_validity.wall_episodes,
                "cumulative_vehicle_episodes": self.review_validity.vehicle_episodes,
                "cumulative_total_episodes": self.review_validity.episode_id,
            })
        elif self.collision_episodes:
            episode = self.collision_episodes[-1]
            episode["wall_collision"] = self.review_validity.episode_wall
            episode["vehicle_collision"] = self.review_validity.episode_vehicle
            episode["cumulative_wall_episodes"] = self.review_validity.wall_episodes
            episode["cumulative_vehicle_episodes"] = self.review_validity.vehicle_episodes
            if transition is not None and transition.kind == "EXIT":
                episode["exit_timestamp_s"] = now_s
                episode["exit_run_elapsed_s"] = max(0.0, now_s - self.start_time)

    def check_pass(self):
        if self.blue_projection is None or self.red_projection is None:
            return
        delta = self.track.signed_delta_s(self.red_projection.s, self.blue_projection.s)
        if delta < -1.0:
            self.was_behind = True
        elif self.was_behind and delta > 1.0:
            self.was_behind = False
            self.pass_count += 1
            self.emit("EVALUATOR", "PHYSICAL_PASS", "EVALUATOR_ONLY_TRUTH")

    def scalar(self, message, field, station):
        if message is None or not message.samples:
            return math.nan
        return interpolate_periodic_scalar(
            tuple(sample.s_m for sample in message.samples),
            tuple(getattr(sample, field) for sample in message.samples),
            station, message.station_period_m,
        )

    def receive_control(self, message):
        now_s = message.receipt_time_ns * 1e-9
        if self.start_time is None or now_s < self.start_time:
            return
        self.recording_started = True
        if not self.controller_runtime_recorded:
            self.writer.manifest["controller_runtime"] = {
                "controller_kind": message.controller_kind,
                "solver_formulation_id": message.solver_formulation_id,
                "horizon_steps": message.horizon_steps,
                "solver_timestep_s": message.solver_timestep_s,
            }
            self.writer._write_manifest()
            self.controller_runtime_recorded = True
        gap = math.nan if self.last_control_time is None else now_s - self.last_control_time
        self.last_control_time = now_s
        if math.isfinite(gap) and gap > 0.05:
            self.command_gap_count += 1
            self.emit("R-B", "COMMAND_GAP", f"{gap:.6f}s")
        if message.fallback_active and not self.last_fallback:
            self.emit("R-B", "FALLBACK", message.reason, message.active_reference_id,
                      message.active_revision)
        self.last_fallback = message.fallback_active
        if self.blue is None:
            return
        projection = self.blue_projection
        station = message.target_progress_m
        nominal_speed = interpolate_periodic_scalar(
            self.nominal_s, self.nominal_speed, station, self.nominal.station_period_m,
        )
        nominal_accel = interpolate_periodic_scalar(
            self.nominal_s, self.nominal_accel, station, self.nominal.station_period_m,
        )
        opponent_s = self.opponent_s if self.opponent_s is not None else math.nan
        prediction_first = (self.prediction.samples[0]
                            if self.prediction is not None and self.prediction.samples else None)
        prediction_time = (stamp_s(self.prediction.source_observation_time)
                           if self.prediction is not None else math.nan)
        covariance = (max(prediction_first.covariance_xy[0], prediction_first.covariance_xy[3])
                      if prediction_first is not None else math.nan)
        prediction_id = (f"{self.prediction.source_track_revision}:{prediction_time:.9f}"
                         if self.prediction is not None else "")
        relative_progress = (
            self.track.signed_delta_s(prediction_first.s_m, projection.s)
            if prediction_first is not None and self.prediction.valid else math.nan
        )
        reason = self.plan.status_reason if self.plan else ""
        state = self.plan.maneuver_state if self.plan else ""
        parity_reached = state == "MERGE" or reason.startswith((
            "POST_PARITY_", "REJOIN_PENDING:", "PASS_CONFIRMED",
        ))
        safe_ahead = state == "MERGE" or reason.startswith((
            "REJOIN_PENDING:", "PASS_CONFIRMED",
        ))
        maneuver_relation = (
            "SAFE_AHEAD" if safe_ahead else
            "POST_PARITY" if parity_reached else
            "PRE_PARITY" if state in ("COMMITTED_PASS", "PASSING") else
            "RED_AHEAD" if state == "BLOCKING_HOLD" else "NOMINAL_CLEAR"
        )
        if maneuver_relation in ("RED_AHEAD", "POST_PARITY", "SAFE_AHEAD"):
            self.last_qualified_opponent_relation = maneuver_relation
        speed_authority_mode = (
            "SAFETY_LIMIT" if state == "ABORT" else
            "INTERACTION" if state in ("BLOCKING_HOLD", "COMMITTED_PASS", "PASSING")
            and not parity_reached else "RACING"
        )
        merge_progress = (
            self.track.forward_delta_s(self.plan.start_progress_m, projection.s)
            if self.plan is not None and state == "MERGE" else math.nan
        )
        selected_speed = self.scalar(self.active, "target_speed_mps", station)
        target_curvature = self.scalar(self.active, "curvature_per_m", station)
        follow_duration = (
            0.0 if self.follow_episode_started_s is None else
            max(0.0, now_s - self.follow_episode_started_s)
        )
        diagnostics = {
            item.split("=", 1)[0]: item.split("=", 1)[1]
            for item in reason.split(":") if "=" in item
        }
        row = {
            "timestamp_s": now_s,
            "run_elapsed_s": now_s - self.start_time,
            "blue_x_m": self.blue.px, "blue_y_m": self.blue.py, "blue_yaw_rad": self.blue.yaw,
            "blue_s_m": projection.s, "blue_d_m": projection.d,
            "blue_actual_speed_mps": message.actual_speed_mps,
            "red_truth_x_m": self.red.px if self.red else math.nan,
            "red_truth_y_m": self.red.py if self.red else math.nan,
            "red_truth_speed_mps": self.red.v if self.red else math.nan,
            "r_a_trajectory_id": self.nominal.trajectory_id,
            "r_a_nominal_speed_mps": nominal_speed, "r_a_nominal_accel_mps2": nominal_accel,
            "s0_r_source": self.active.source_kind if self.active else "",
            "s0_r_reference_id": self.active.active_reference_id if self.active else "",
            "s0_r_revision": self.active.active_revision if self.active else 0,
            "s0_r_target_speed_mps": selected_speed,
            "s0_r_target_accel_mps2": self.scalar(self.active, "target_accel_mps2", station),
            "r_b_requested_speed_mps": message.target_speed_mps,
            "r_b_target_progress_m": message.target_progress_m,
            "r_b_qualification_cap_active": message.qualification_cap_active,
            "r_b_steering_rad": message.steering_command_rad,
            "r_b_acceleration_mps2": message.acceleration_command_mps2,
            "r_b_lateral_error_m": message.lateral_error_m,
            "r_b_heading_error_rad": message.heading_error_rad,
            "r_b_mode": message.mode, "r_b_fallback": message.fallback_active,
            "r_b_active_reference_id": message.active_reference_id,
            "r_b_command_gap_s": gap, "r_b_compute_us": message.compute_time_us,
            "r_c_lifecycle": self.opponent.lifecycle if self.opponent else "",
            "r_c_confidence": self.opponent.confidence if self.opponent else math.nan,
            "r_c_compute_us": self.opponent.compute_time_us if self.opponent else math.nan,
            "r_c_opponent_s_m": opponent_s,
            "r_c_opponent_speed_mps": self.opponent.speed_mps if self.opponent else math.nan,
            "r_c_state_age_s": self.opponent.measurement_age_s if self.opponent else math.nan,
            "r_d_prediction_id": prediction_id,
            "r_d_prediction_age_s": self.prediction.source_age_s if self.prediction else math.nan,
            "r_d_prediction_valid": self.prediction.valid if self.prediction else False,
            "r_d_opponent_s_m": prediction_first.s_m if prediction_first else math.nan,
            "r_d_opponent_speed_mps": (math.hypot(prediction_first.vx_mps, prediction_first.vy_mps)
                                       if prediction_first else math.nan),
            "r_d_max_covariance_m2": covariance,
            "r_d_compute_us": self.prediction.compute_time_us if self.prediction else math.nan,
            "r_e_state": self.plan.maneuver_state if self.plan else "",
            "r_e_reason": self.plan.status_reason if self.plan else "",
            "r_e_side": self.plan.passing_side if self.plan else "",
            "r_e_plan_id": self.plan.plan_id if self.plan else "",
            "r_e_revision": self.plan.plan_revision if self.plan else 0,
            "r_e_compute_us": self.plan.compute_time_us if self.plan else math.nan,
            "r_e_min_predicted_clearance_m": (self.plan.minimum_predicted_separation_m
                                               if self.plan else math.nan),
            "relative_progress_m": relative_progress,
            "maneuver_relation": maneuver_relation,
            "speed_authority_mode": speed_authority_mode,
            "parity_reached": parity_reached,
            "safe_ahead": safe_ahead,
            "merge_feasible": state == "MERGE",
            "merge_phase": reason,
            "merge_progress_m": merge_progress,
            "selected_speed_authority_mps": selected_speed,
            "last_qualified_opponent_relation": self.last_qualified_opponent_relation,
            "same_strategy_follow_duration_s": follow_duration,
            "left_candidate_result": diagnostics.get("EVAL_LEFT", ""),
            "right_candidate_result": diagnostics.get("EVAL_RIGHT", ""),
            "lap": self.lap, "lap_time_s": self.last_lap_time,
            "race_valid": self.race_valid, "wall_collisions": self.wall_collisions,
            "car_collision": self.car_collision,
            "execution_speed_cap_mps": self.execution_speed_cap_mps,
            "controller_kind": message.controller_kind,
            "r_b_target_curvature_per_m": target_curvature,
            "r_a_target_speed_mps": nominal_speed,
            "r_b_local_target_speed_mps": message.local_speed_limit_mps,
            "r_b_local_speed_reason": message.local_speed_limit_reason,
            "r_b_raw_dynamic_limit_mps": message.raw_dynamic_limit_mps,
            "r_b_backward_reachable_limit_mps": message.backward_reachable_limit_mps,
            "r_b_execution_envelope_mps": message.execution_envelope_mps,
            "r_b_effective_preview_distance_m": message.effective_preview_distance_m,
            "r_b_required_braking_distance_m": message.required_braking_distance_m,
            "r_b_fallback_cap_mps": message.fallback_cap_mps,
            "r_b_bounded_fallback_target_mps": message.bounded_fallback_target_mps,
            "r_b_primary_admission_state": message.primary_admission_state,
            "r_b_feasibility_duration_s": message.feasibility_duration_s,
            "r_b_predicted_terminal_recoverable": message.predicted_terminal_recoverable,
            "r_b_recoverability_state": message.recoverability_state,
            "r_b_recoverability_reason": message.recoverability_reason,
            "r_b_recoverability_margin": message.recoverability_margin,
            "r_b_terminal_recoverability_margin": message.terminal_recoverability_margin,
            "r_b_predicted_terminal_x_m": message.predicted_terminal_x_m,
            "r_b_predicted_terminal_y_m": message.predicted_terminal_y_m,
            "r_b_predicted_terminal_yaw_rad": message.predicted_terminal_yaw_rad,
            "r_b_predicted_terminal_speed_mps": message.predicted_terminal_speed_mps,
            "r_b_predicted_terminal_slip_rad": message.predicted_terminal_slip_rad,
            "r_b_predicted_terminal_yaw_rate_rad_s": message.predicted_terminal_yaw_rate_rad_s,
            "r_b_hard_valid": self.race_valid and not self.car_collision and self.wall_collisions == 0,
            "r_b_fallback_reason": message.reason if message.fallback_active else "",
            "r_b_recovery_active": message.degradation_active,
            "r_b_recovery_reason": message.reason if message.degradation_active else "",
            "solver_formulation_id": message.solver_formulation_id,
            "solver_horizon_steps": message.horizon_steps,
            "solver_timestep_s": message.solver_timestep_s,
            "r_b_primary_status": message.primary_status,
            "r_b_solver_iterations": message.solver_iterations,
            "r_b_primary_solve_us": message.primary_solve_time_us,
            "r_b_deadline_met": (
                message.primary_status == "PRIMARY_VALID" and
                message.primary_solve_time_us <= self.solver_deadline_s * 1e6
            ),
            "r_b_solver_objective": message.solver_objective,
            "r_b_constraint_residual": message.max_constraint_residual,
        }
        row.update(r_b_dynamic_fields(
            self.blue, message.steering_command_rad, message.acceleration_command_mps2,
            self.steering_limit_rad, self.acceleration_limit_mps2,
            self.braking_limit_mps2,
        ))
        self.writer.telemetry(row)
        if self.review_phase == "R-A":
            self.writer.r_a_runtime(row)
        elif self.review_phase == "R-B":
            self.writer.r_b_runtime(row)
            self.writer.r_b_solver(row)

    def close(self, termination_reason="NORMAL_COMPLETION"):
        if self.closed:
            return
        self.emit("RECORDER", "SHUTDOWN", "FLUSH")
        try:
            stop_child(self.bag)
        finally:
            self.writer.collision_episodes(self.collision_episodes)
            finalized_unix_s = time.time()
            receipt = self.writer.close({
                "valid": self.race_valid and not self.car_collision and self.wall_collisions == 0,
                "laps": self.lap, "physical_passes": self.pass_count,
                "wall_collisions": self.wall_collisions, "car_collision": self.car_collision,
                "collision_episodes": self.review_validity.episode_id,
                "wall_collision_episodes": self.review_validity.wall_episodes,
                "vehicle_collision_episodes": self.review_validity.vehicle_episodes,
                "command_gaps_over_50ms": self.command_gap_count,
                "bag_exit_code": (self.bag.poll() if self.bag is not None else None),
                "review_phase": self.review_phase,
                "race_start_time_s": self.start_time,
                "race_duration_s": (None if self.start_time is None else
                                    max(0.0, self.now() - self.start_time)),
                "termination_reason": termination_reason,
                "finalized_unix_s": finalized_unix_s,
            })
            self.closed = True
            print("=" * 60, flush=True)
            print(" PHASE REVIEW EVIDENCE FINALIZED", flush=True)
            print("=" * 60, flush=True)
            print(f"PHASE={self.review_phase}", flush=True)
            print(f"RUN_ID={self.run_dir.name}", flush=True)
            print(f"TERMINATION={termination_reason}", flush=True)
            print(f"TELEMETRY={receipt['telemetry_rows']}", flush=True)
            print(f"EVENTS={receipt['event_rows']}", flush=True)
            print(f"RUN_DIR={self.run_dir}", flush=True)
            print(f"SQLITE_BAK={receipt['sqlite_path']}", flush=True)
            print(f"INTEGRITY={'PASS' if receipt['sqlite_integrity'] == 'ok' else 'FAIL'}", flush=True)
            print(f"SHA256={receipt['sha256_status']}", flush=True)
            print("=" * 60, flush=True)


def main():
    rclpy.init()
    node = FlightRecorder()
    termination_reason = "NORMAL_COMPLETION"
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        termination_reason = "USER_CTRL_C"
    except ExternalShutdownException:
        termination_reason = "ROS_CONTEXT_SHUTDOWN"
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.close(termination_reason)
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
