import json
import math
import time
import threading
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from collections import deque

import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from control_msgs.msg import ActuatorIntent, ActiveTargetTrajectory, CarState, ControllerStatus
from f1tenth_racing_reference.contract_guard import validate_reference_message
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from f1tenth_racing_reference.node import ACTIVE_REFERENCE_QOS
from f1tenth_racing_sim.world.map import OccupancyMap
from f1tenth_racing_sim.safety.stopping_envelope import StoppingEnvelope
from .fast_stop_proof import CertifiedFreeStoppingEnvelope

from .core import ControlConfig, EgoState, ModelAidedPursuit, PursuitReference
from .mpc import (
    B0RecoverabilityGovernor,
    EnvelopeBoundReference,
    LtvDynamicBicycleMpc,
    NonlinearDynamicBicycleMpc,
    MpcConfig,
    MpcCycleBudgetExceeded,
    PrimaryAdoptionGate,
)
from .stability import StabilityRecoverySupervisor
from .fallback_safety import FallbackSafetyTransitionSupervisor
from .guard_ahead import GuardAheadAdmission
from .control_worker import LatestOnlyControlWorker
from .primary_worker import MpcPrimaryWorker
from .process_primary_worker import IsolatedMpcProcessWorker
from .execution_speed_authority import derive_closed_loop_execution_speed
from .final_preflight import admit_final_command
from .dynamic_admission import PostBootstrapDynamicAdmission
from .execution_contract import verify_run_contract, sha as sha_file
from .reference_admission import authorize_active_reference
from .environment_identity import read_environment, vehicle_contract_sha256
from std_msgs.msg import String


def build_prestart_status(now_ns, reference_id, reference_revision,
                          actual_speed_mps, controller_kind):
    status = ControllerStatus()
    status.receipt_time_ns = now_ns
    status.mode = "PRESTART_READY"
    status.reason = "WAITING_FOR_RACE_START"
    status.active_reference_id = reference_id
    status.active_revision = reference_revision
    status.actual_speed_mps = actual_speed_mps
    status.primary_status = "NOT_RUN"
    status.fallback_active = False
    status.controller_kind = controller_kind
    status.local_speed_limit_reason = "PRESTART_READY"
    return status


class RacingPursuitController(Node):
    def __init__(self):
        super().__init__("r_b0_racing_pursuit")
        # Serializes controller history, reference adoption, START and watchdog
        # against the off-executor computation worker. Receipt remains O(1).
        self._control_lock = threading.RLock()
        self._state_receipt_lock = threading.Lock()
        self._control_worker = None
        self._primary_worker = None
        control_share = Path(get_package_share_directory("f1tenth_racing_control"))
        sim_share = Path(get_package_share_directory("f1tenth_racing_sim"))
        settings = yaml.safe_load((control_share / "config/r_b0.yaml").read_text())
        mpc_settings = yaml.safe_load((control_share / "config/r_b3_mpc.yaml").read_text())
        self.declare_parameter("guard_run_epoch", "")
        self.intent_sequence = 0
        self.last_source_state_stamp = Time()
        self.last_source_state_sequence = 0
        self.declare_parameter("controller_kind", "pursuit")
        # Default: off-executor latest-only control computation. No realtime claim.
        self.declare_parameter("control_worker_enabled", True)
        self.declare_parameter("primary_process_enabled", False)
        self.declare_parameter("primary_worker_enabled", True)
        self.declare_parameter("controller_profile", "default")
        self.declare_parameter("mpc_formulation", "ltv")  # nmpc: explicit research opt-in
        self.declare_parameter("fallback_speed_cap_mps", 0.0)
        self.declare_parameter("diagnostic_withhold_primary", False)
        self.declare_parameter("sim_online_adaptive", False)
        # EXPERIMENTAL: C++ full stop rollout for R-B proposal screening only.
        # Guard final approval stays in the authoritative independent Python
        # process, even when the optional preflight prototype is used.
        self.declare_parameter("experimental_native_guardahead", False)
        self.declare_parameter("native_guardahead_library", "")
        self.declare_parameter("native_guardahead_receipt", "")
        self.declare_parameter("expected_track_id", "iccas2025")
        self.declare_parameter("expected_map_sha256", "")
        self.declare_parameter("high_speed_contract_file", "")
        self.declare_parameter("high_speed_trajectory_id", "")
        self.declare_parameter("high_speed_trusted_public_key_file", "")
        self.declare_parameter("high_speed_trusted_public_key_sha256", "")
        self.declare_parameter("environment_file", str(sim_share / "config/competition_environment.yaml"))
        self.declare_parameter("centerline_file", "")
        self.declare_parameter(
            "qualified_speed_cap_mps", settings["qualified_speed_cap_mps"]
        )
        environment_file=self.get_parameter("environment_file").value
        environment, self.environment_sha256 = read_environment(
            environment_file,
            track_id=self.get_parameter("expected_track_id").value,
            map_sha256=self.get_parameter("expected_map_sha256").value,
        )
        self.vehicle_contract_sha256=vehicle_contract_sha256(environment)
        self.execution_receipt_sha256=""
        self.high_speed_authorized=False
        selected_centerline=self.get_parameter("centerline_file").value or str(
            sim_share / environment["map"]["centerline_file"])
        centerline=Path(selected_centerline).resolve(strict=True)
        if (not centerline.is_file() or sha_file(centerline) !=
                environment["map"]["centerline_sha256"]):
            raise ValueError("selected controller centerline SHA mismatch")
        # The recovery corridor must use exactly the simulator's chosen asset.
        environment=dict(environment)
        environment["map"]=dict(environment["map"],centerline_file=str(centerline))
        dynamics = environment["dynamics"]
        limits = dynamics["limits"]
        self.controller_kind = self.get_parameter("controller_kind").value
        self.diagnostic_withhold_primary = self.get_parameter(
            "diagnostic_withhold_primary"
        ).value
        if self.controller_kind not in ("pursuit", "mpc"):
            raise ValueError("controller_kind must be pursuit or mpc")
        self.primary_speed_cap_mps = float(self.get_parameter("qualified_speed_cap_mps").value)
        self.sim_online_adaptive = bool(self.get_parameter("sim_online_adaptive").value)
        self.sim_online_speed_bound_mps = float(limits["speed_abs_max_m_s"])
        if not math.isfinite(self.primary_speed_cap_mps) or not (0 < self.primary_speed_cap_mps <= self.sim_online_speed_bound_mps):
            raise ValueError("REQUESTED_SPEED_EXCEEDS_SIM_PHYSICS_BOUND")
        if self.sim_online_adaptive:
            if not bool(self.get_parameter("expected_track_id").value) or not environment.get("simulation"):
                raise ValueError("SIM_ADAPTIVE_MODE_REQUIRES_VERIFIED_SIM_ENVIRONMENT")
            # Online Guard checks are NOT a previously qualified high-speed run.
            # Keep high_speed_authorized=False and record this distinction.
        if self.primary_speed_cap_mps > 5.8 and not self.sim_online_adaptive:
            receipt = self.get_parameter("high_speed_contract_file").value
            if not receipt:
                raise ValueError("unqualified high speed must be blocked")
            verify_run_contract(receipt,
                track_id=self.get_parameter("expected_track_id").value,
                map_sha=self.get_parameter("expected_map_sha256").value,
                trajectory_id=self.get_parameter("high_speed_trajectory_id").value,
                controller_kind=self.controller_kind, proposed_cap=self.primary_speed_cap_mps,
                trusted_public_key_file=self.get_parameter("high_speed_trusted_public_key_file").value,
                trusted_public_key_sha256=self.get_parameter("high_speed_trusted_public_key_sha256").value,
                environment_sha256=self.environment_sha256,
                vehicle_contract_sha256=self.vehicle_contract_sha256)
            self.execution_receipt_sha256=sha_file(receipt)
            self.high_speed_authorized=True
        configured_fallback=float(self.get_parameter("fallback_speed_cap_mps").value)
        if not math.isfinite(configured_fallback) or configured_fallback<0.0:
            raise ValueError('INVALID_FALLBACK_CAP')
        fallback_cap=(self.primary_speed_cap_mps if self.controller_kind=='pursuit' or
                      configured_fallback==0.0 else configured_fallback)
        self.controller = ModelAidedPursuit(ControlConfig(
            wheelbase_m=dynamics["dynamic_wheelbase_m"],
            lookahead_base_m=settings["lookahead_base_m"],
            lookahead_speed_gain_s=settings["lookahead_speed_gain_s"],
            lookahead_min_m=settings["lookahead_min_m"],
            lookahead_max_m=settings["lookahead_max_m"],
            speed_feedback_gain_per_s=settings["speed_feedback_gain_per_s"],
            qualified_speed_cap_mps=fallback_cap,
            steering_max_rad=limits["steering_angle_abs_max_rad"],
            steering_rate_max_rad_s=limits["steering_rate_abs_max_rad_s"],
            acceleration_max_mps2=limits["acceleration_max_m_s2"],
            braking_max_mps2=limits["deceleration_max_m_s2"],
            jerk_max_mps3=limits["jerk_abs_max_m_s3"],
            degraded_deceleration_mps2=settings["degraded_deceleration_mps2"],
        ))
        profile = self.get_parameter("controller_profile").value
        profiles = mpc_settings.pop("profiles")
        if profile not in profiles:
            raise ValueError(f"unknown controller_profile: {profile}")
        mpc_settings.update(profiles[profile])
        self.mpc_formulation = str(self.get_parameter('mpc_formulation').value)
        if self.mpc_formulation not in ('ltv', 'nmpc'):
            raise ValueError('MPC_FORMULATION_UNKNOWN')
        if self.mpc_formulation == 'nmpc' and (self.controller_kind != 'mpc' or
                not bool(self.get_parameter('primary_worker_enabled').value)):
            raise ValueError('NMPC_REQUIRES_INDEPENDENT_PRIMARY_WORKER')
        primary_type = (NonlinearDynamicBicycleMpc if self.mpc_formulation == 'nmpc'
                        else LtvDynamicBicycleMpc)
        self.primary = primary_type(
            dynamics,
            limits,
            MpcConfig(**mpc_settings),
        )
        # Isolate the MPC solver's warm-start, speed-envelope memoization and
        # mutable QP state from the fallback/recoverability worker. A slow
        # solver never owns the actuator and may not block the safety path.
        self.primary_worker_enabled = (self.controller_kind == 'mpc' and
            bool(self.get_parameter('primary_worker_enabled').value))
        self._primary_solver = (primary_type(
            dynamics, limits, MpcConfig(**mpc_settings))
            if self.primary_worker_enabled else self.primary)
        if bool(self.get_parameter("primary_process_enabled").value):
            if not self.primary_worker_enabled:
                raise ValueError('PROCESS_PRIMARY_REQUIRES_ENABLED_MPC_PRIMARY_WORKER')
            # The subprocess builds its OWN solver; no Python GIL or warm-start
            # is shared with the fallback. It never holds ROS publishing rights.
            self._primary_worker = IsolatedMpcProcessWorker(
                dynamics=dynamics, limits=limits,
                mpc_config=mpc_settings, formulation=self.mpc_formulation)
        elif self.primary_worker_enabled:
            self._primary_worker = MpcPrimaryWorker()
        self.primary_gate = PrimaryAdoptionGate(
            self.primary.config.horizon_steps * self.primary.config.timestep_s
        )
        map_selected=Path(sim_share/environment["map"]["image_file"])
        selected_map=Path(self.get_parameter("environment_file").value).resolve()
        # Use verified environment asset path; map file is independently
        # cross-checked by Launch before this controller starts.
        self.stopping_envelope=CertifiedFreeStoppingEnvelope(
            dynamics,OccupancyMap(map_selected,environment["map"]),
            environment["collision"]["footprint_relative_to_base_link"])
        preflight_stop=self.stopping_envelope
        self.experimental_native_guardahead=bool(self.get_parameter(
            "experimental_native_guardahead").value)
        if self.experimental_native_guardahead:
            if not self.sim_online_adaptive:
                raise ValueError("NATIVE_GUARDAHEAD_ONLY_IN_VERIFIED_SIM_MODE")
            lib_path=str(self.get_parameter("native_guardahead_library").value)
            if not lib_path or not Path(lib_path).is_file():
                raise ValueError("NATIVE_GUARDAHEAD_LIBRARY_NOT_FOUND")
            from f1tenth_racing_sim.safety.native_stop_proof import (
                ExperimentalNativeProofProxy,check_experimental_parity_receipt)
            receipt_path=str(self.get_parameter("native_guardahead_receipt").value)
            if not receipt_path:
                raise ValueError("NATIVE_GUARDAHEAD_PARITY_RECEIPT_MISSING")
            check_experimental_parity_receipt(lib_path,receipt_path,
                expected_map_sha256=environment["map"]["image_sha256"],
                active_map_file=map_selected)
            preflight_stop=ExperimentalNativeProofProxy(self.stopping_envelope,lib_path)
        self.guard_ahead = GuardAheadAdmission(preflight_stop, limits)
        # 40 Hz wall-clock admission, independent from simulated ROS time.
        # Reserve the back of the callback for independent stop pre-check,
        # dynamic/stability overrides, ROS publication and transport.
        self.mpc_cycle_slot_ns = 12_000_000
        # Time budget belongs to the entire 40Hz state callback, not just QP.
        # B0 must leave room for Fallback, Stability, stop preflight and ROS
        # Intent+status publication; a timed-out proof fails closed.
        self.b0_cycle_deadline_ns = 14_000_000
        # Stop costly fallback prediction before the 25 ms callback limit.
        # This is a failure deadline, NOT an abbreviated safety proof.
        self.fallback_preview_slot_ns = 15_000_000
        # The fresh-state fallback preview is completed BEFORE expensive
        # Primary/B0 work in established racing. A timed-out/unsafe preview
        # stays fail-closed; the independent Guard and final Preflight remain.
        self.fallback_first_slot_ns = 8_000_000
        self.stop_precheck_reserve_ns = 8_000_000
        self.stage_timing_pub = self.create_publisher(
            String, "r_b_compute_stages0",
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.recoverability_governor = B0RecoverabilityGovernor(
            self.primary, self.controller,
            stopping_checker=lambda state,steer,accel:
                self.stopping_envelope.evaluate(state,steer,accel).qualified,
        )
        self.stability_recovery = StabilityRecoverySupervisor(
            self.primary,
            self.recoverability_governor,
            dynamics,
            limits,
            environment,
            sim_share,
        )
        self.fallback_safety = FallbackSafetyTransitionSupervisor(
            self.primary,
            self.recoverability_governor,
            self.controller,
            self.stability_recovery,
        )
        # Recovery proof must predict the same startup fallback shaper.
        self.recoverability_governor.fallback_supervisor = self.fallback_safety
        self.dynamic_admission = PostBootstrapDynamicAdmission(
            self.stability_recovery,
            self.controller.config.degraded_deceleration_mps2,
            self.controller.config.jerk_max_mps3,
            self.fallback_safety.ACCEL_RELEASE_SPEED_MPS,
            self.fallback_safety.CORRIDOR_GUARD_M,
        )
        self.state_freshness_ns = round(settings["state_freshness_s"] * 1e9)
        self.reference = None
        self.reference_valid = False
        self.reference_id = ""
        self.reference_revision = 0
        self.start_ns = None
        # Latched exactly once after initial actual motion; low speeds in a
        # later corner must never retrigger STARTUP or erase Primary history.
        self.startup_primary_phase_complete = False
        self.last_state_ns = None
        self.last_command_ns = None
        self.last_actual_speed = 0.0
        self.previous_steering = 0.0
        self.previous_acceleration = 0.0
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.command_pub = self.create_publisher(
            ActuatorIntent, "actuator_intent0", command_qos
        )
        self.status_pub = self.create_publisher(ControllerStatus, "controller_status0", 10)
        execution_qos=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.execution_receipt_pub=self.create_publisher(
            String, "controller_execution_receipt", execution_qos)
        self.adopted_source_kind=""
        self.adopted_source_trajectory_id=""
        self.create_subscription(
            ActiveTargetTrajectory,
            "active_target_trajectory",
            self.receive_reference,
            ACTIVE_REFERENCE_QOS,
        )
        # Fresh-state mailbox decouples ROS receipt from expensive control work.
        # It is deliberately size one: never compute a command for a queued old
        # state after a newer one has arrived. The Guard remains independently
        # subscribed to every simulator state, and never trusts this mailbox.
        self._pending_state = None
        self._pending_state_steady_ns = None
        self._processed_state_sequence = -1
        self._received_state_sequence = -1
        self._last_received_physics_stamp_ns = -1
        # Latest-only arrival ages, recorded before asynchronous computation.
        # Pure pre-start evidence; never authorizes or refreshes a command.
        self._state_rx_recent_ages_ns = deque(maxlen=8)
        self._state_rx_receipt_count = 0
        self._state_rx_last_receipt_steady_ns = -1
        self._cycle_source_sequence = None
        self._cycle_source_stamp = None
        self._last_dropped_compute_reason = ""
        self._stage_telemetry_queue = deque(maxlen=64)
        self._stage_diagnostic_dropped = 0
        # Executing R-B latest-only state subscription: measured on the
        # identical Ubuntu host in a three-way DDS probe (2026-10-19).
        # RELIABLE/depth=1 lagged 357ms median; BEST_EFFORT/depth=1 was
        # 1.019ms median / 12.258ms maximum across 1,438 samples.
        # Transport cannot grant actuation: the ORIGINAL 75ms source-stamp
        # gate and the independent Guard's RELIABLE/depth=16 remain active.
        self.state_subscription_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CarState, "state0", self.receive_state_snapshot,
                                 self.state_subscription_qos)
        # Physics publishes every 25ms, but a matching 25ms timer can wait
        # almost another full physics tick depending on the two timer phases.
        # Poll the latest-only mailbox every 5ms instead. A state is consumed
        # at most once; this DOES NOT create extra controller commands or
        # guarantee scheduling under CPU contention.
        self.control_worker_enabled = bool(self.get_parameter(
            "control_worker_enabled").value)
        if not self.control_worker_enabled:
            # Legacy execution mode for controlled comparisons only.
            self.create_timer(0.005, self.run_latest_state_cycle)
        self.create_timer(0.01, self.flush_stage_diagnostics)
        self.worker_diagnostics_pub = self.create_publisher(
            String, 'control_worker_metrics', 10)
        self.create_timer(0.25, self.publish_worker_diagnostics)
        start_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Time, "race_start_time", self.receive_start, start_qos)
        self.create_timer(0.025, self.watchdog)
        if self.control_worker_enabled:
            self._control_worker = LatestOnlyControlWorker(
                self.run_latest_state_cycle,
                on_error=self._control_worker_error)

    def destroy_node(self):
        # Never destroy publishers while the dedicated worker can use them.
        primary_worker = getattr(self, '_primary_worker', None)
        if primary_worker is not None and not primary_worker.close(timeout_s=5.0):
            raise RuntimeError('PRIMARY_WORKER_SHUTDOWN_UNPROVEN')
        worker = getattr(self, '_control_worker', None)
        if worker is not None and not worker.close(timeout_s=5.0):
            # Fail safely: callers must not silently consider ROS shutdown clean
            # if the worker is still running. Guard/Simulator leases expire.
            raise RuntimeError('CONTROL_WORKER_SHUTDOWN_UNPROVEN')
        return super().destroy_node()

    def publish_worker_diagnostics(self):
        # Observability only. Never used to authorize any actuator command.
        worker = self._control_worker
        if worker is None:
            return
        from dataclasses import asdict
        facts = asdict(worker.counters())
        # Snapshot diagnostics independently from the hard 75ms execution gate.
        # The pre-START observer requires 8 fresh R-B *received* samples,
        # unlike the old observer which only saw the Guard's fresh samples.
        with self._state_receipt_lock:
            rx_ages = list(self._state_rx_recent_ages_ns)
            rx_receipt_count = self._state_rx_receipt_count
            rx_steady_ns = self._state_rx_last_receipt_steady_ns
            rx_sequence = self._received_state_sequence
        rx_recent_ns = (time.monotonic_ns() - rx_steady_ns
                        if rx_steady_ns >= 0 else -1)
        facts.update({
            'schema_version': 2,
            'state_rx_qos_reliability': 'BEST_EFFORT',
            'state_rx_qos_depth': 1,
            'state_rx_count': rx_receipt_count,
            'state_rx_last_sequence': rx_sequence,
            'state_rx_recent_ages_ns': rx_ages,
            'state_rx_last_receipt_steady_age_ns': rx_recent_ns,
            'state_rx_recent_8_fresh': (
                len(rx_ages) == 8 and
                all(0 <= x <= 75_000_000 for x in rx_ages) and
                0 <= rx_recent_ns <= 250_000_000),
            'type': 'R_B_CONTROL_WORKER_SCHEDULING',
            'receipt_time_ns': self.get_clock().now().nanoseconds,
            'threaded': True,
            'hard_realtime_qualified': False,
        })
        primary_worker = getattr(self, '_primary_worker', None)
        if primary_worker is not None:
            facts['primary_worker'] = asdict(primary_worker.counters())
            if isinstance(primary_worker, IsolatedMpcProcessWorker):
                facts['primary_process_recovery'] = primary_worker.recovery_status()
        payload = String()
        payload.data = json.dumps(facts, sort_keys=True)
        self.worker_diagnostics_pub.publish(payload)

    def _control_worker_error(self, error):
        # Do not synthesize an intent or a positive safety receipt on failure.
        self._last_dropped_compute_reason = (
            'WORKER_EXCEPTION_UNPROVEN:' + type(error).__name__)
        self.get_logger().error(self._last_dropped_compute_reason)

    def receive_reference(self, message):
        with self._control_lock:
            return self._receive_reference_locked(message)

    def _receive_reference_locked(self, message):
        try:
            if not message.active_reference_valid or message.source_kind not in ("NOMINAL", "OVERTAKE_OVERRIDE"):
                raise ValueError(message.status_reason or "invalid active reference")
            expected_map = self.get_parameter("expected_map_sha256").value
            if not expected_map:
                raise ValueError("missing expected map SHA; cannot authorize controller")
            validate_reference_message(message,
                expected_track_id=self.get_parameter("expected_track_id").value,
                expected_map_sha256=expected_map)
            authorize_active_reference(
                message,
                requested_cap_mps=self.primary_speed_cap_mps,
                qualified_trajectory_id=self.get_parameter("high_speed_trajectory_id").value,
                expected_track_id=self.get_parameter("expected_track_id").value,
                expected_map_sha256=expected_map,
                trusted_high_speed=self.high_speed_authorized,
                simulation_online_bound_mps=(self.sim_online_speed_bound_mps
                    if self.sim_online_adaptive else None))
            if message.sample_count != len(message.samples) or message.sample_count < 3:
                raise ValueError("active reference sample count mismatch")
            samples = tuple(SimpleNamespace(**{
                field: getattr(sample, field)
                for field in (
                    "sample_index", "s_m", "x_m", "y_m", "heading_rad",
                    "curvature_per_m", "lateral_offset_m", "target_speed_mps",
                    "target_accel_mps2", "ds_to_next_m",
                )
            }) for sample in message.samples)
            # R-A authored v(s) is retained in the accepted wire message;
            # locally limit only the execution proposal by map curvature,
            # vehicle reachability, and the Guard's finite stop horizon.
            # This never issues a certificate. Full current/N+1 GuardAhead
            # plus independent Guard authority are still mandatory.
            if self.sim_online_adaptive:
                samples, self.execution_speed_audit = derive_closed_loop_execution_speed(
                    samples, message.station_period_m,
                    vehicle_speed_bound_mps=self.sim_online_speed_bound_mps,
                    braking_mps2=self.stopping_envelope.brake,
                    acceleration_mps2=float(self.stopping_envelope.dynamics.c['limits']['acceleration_max_m_s2']),
                    proof_horizon_s=self.stopping_envelope.max_time,
                    reaction_s=self.stopping_envelope.reaction,
                    lateral_accel_bound_mps2=self.stopping_envelope.lat_limit,
                    physics_step_s=self.stopping_envelope.dt,
                    wheelbase_m=float(self.stopping_envelope.dynamics.c['dynamic_wheelbase_m']),
                    steering_rate_limit_rad_s=float(self.stopping_envelope.dynamics.c['limits']['steering_rate_abs_max_rad_s']),
                    jerk_limit_mps3=float(self.stopping_envelope.dynamics.c['limits']['jerk_abs_max_m_s3']))
            else:
                self.execution_speed_audit = None
            reference = PursuitReference(samples, message.station_period_m)
            if not math.isclose(reference.geometry.length, message.path_length_m, abs_tol=1e-9):
                raise ValueError("active reference path length mismatch")
            reference_changed = (
                message.active_reference_id != self.reference_id or
                message.active_revision != self.reference_revision
            )
            self.reference = reference
            self.reference_valid = True
            self.reference_id = message.active_reference_id
            self.reference_revision = message.active_revision
            self.adopted_source_kind=message.source_kind
            self.adopted_source_trajectory_id=message.source_trajectory_id
            self.publish_execution_provenance(now_ns=self.get_clock().now().nanoseconds,
                                             event="REFERENCE_ADOPTED")
            if reference_changed:
                self.primary_gate.reset()
                self.stability_recovery.reset()
            self.get_logger().info(json.dumps({
                "active_reference_id": self.reference_id,
                "active_revision": self.reference_revision,
                "samples": len(samples),
                "source_kind": message.source_kind,
            }, sort_keys=True))
        except (AttributeError, TypeError, ValueError) as error:
            self.reference = None
            self.reference_valid = False
            self.execution_speed_audit = None
            self.reference_id = message.active_reference_id
            self.reference_revision = message.active_revision
            self.adopted_source_kind=""
            self.adopted_source_trajectory_id=""
            self.publish_execution_provenance(now_ns=self.get_clock().now().nanoseconds,
                                             event="REFERENCE_REJECTED", reason=str(error))
            self.get_logger().error(f"active reference rejected: {error}")

    def publish_execution_provenance(self, *, now_ns, event, reason=""):
        # This is controller-self-reported provenance, NOT an external safety approval.
        data={"schema_version":1, "receipt_time_ns":int(now_ns),
              "event":event, "reason":reason,
              "track_id":self.get_parameter("expected_track_id").value,
              "map_image_sha256":self.get_parameter("expected_map_sha256").value,
              "environment_sha256":self.environment_sha256,
              "vehicle_contract_sha256":self.vehicle_contract_sha256,
              "execution_contract_sha256":self.execution_receipt_sha256,
              "active_reference_id":self.reference_id,
              "active_revision":self.reference_revision,
              "source_kind":self.adopted_source_kind,
              "source_trajectory_id":self.adopted_source_trajectory_id,
              "controller_kind":self.controller_kind,
              "qualified_speed_cap_mps":float(self.primary_speed_cap_mps),
              "high_speed_authorized":self.high_speed_authorized,
              "execution_speed_audit": (
                  vars(self.execution_speed_audit) if getattr(self,'execution_speed_audit',None)
                  is not None else None)}
        msg=String();msg.data=json.dumps(data,sort_keys=True)
        self.execution_receipt_pub.publish(msg)

    def receive_start(self, message):
        with self._control_lock:
            return self._receive_start_locked(message)

    def _receive_start_locked(self, message):
        next_start_ns = message.sec * 1_000_000_000 + message.nanosec
        if self.start_ns == next_start_ns:
            # TRANSIENT_LOCAL replay of the same START is not a new race.
            return
        if self.start_ns is not None:
            # No unrequested restart in an active uninterrupted race.
            self.get_logger().warning("IGNORED_SECOND_RACE_START_WHILE_ACTIVE")
            return
        self.start_ns = next_start_ns
        self.startup_primary_phase_complete = False
        self.primary_gate.reset()
        self.fallback_safety.reset()
        self.dynamic_admission.reset()

    def receive_state_snapshot(self, message):
        """Fast callback: validate monotonic identity, enqueue newest only.

        The expensive cycle runs on a dedicated thread when configured. A
        replaced state is never replayed. Guard independently sees all states.
        """
        seq = int(message.physics_sequence)
        stamp_ns = (int(message.physics_stamp.sec)*1_000_000_000
                    + int(message.physics_stamp.nanosec))
        with self._state_receipt_lock:
            if (seq <= self._received_state_sequence or
                    stamp_ns <= self._last_received_physics_stamp_ns):
                self._last_dropped_compute_reason = 'RECEIPT_NON_MONOTONIC_UNPROVEN'
                return
            self._received_state_sequence = seq
            self._last_received_physics_stamp_ns = stamp_ns
            received_ros_ns = self.get_clock().now().nanoseconds
            self.last_state_ns = received_ros_ns
            self._state_rx_recent_ages_ns.append(received_ros_ns - stamp_ns)
            self._state_rx_receipt_count += 1
            self._state_rx_last_receipt_steady_ns = time.monotonic_ns()
            if self.control_worker_enabled:
                if not self._control_worker.submit(seq, message):
                    self._last_dropped_compute_reason = 'WORKER_QUEUE_REJECTED'
            else:
                self._pending_state = message
                self._pending_state_steady_ns = time.monotonic_ns()

    def run_latest_state_cycle(self, message=None):
        """Run one expensive cycle; all mutable controller state is serialized."""
        if message is None:
            with self._state_receipt_lock:
                message = self._pending_state
                self._pending_state = None
        if message is None:
            return
        with self._control_lock:
            seq = int(message.physics_sequence)
            if seq <= self._processed_state_sequence:
                self._last_dropped_compute_reason = 'ALREADY_PROCESSED_UNPROVEN'
                return
            self._processed_state_sequence = seq
            now = self.get_clock().now().nanoseconds
            stamp_ns = (int(message.physics_stamp.sec)*1_000_000_000
                        + int(message.physics_stamp.nanosec))
            if (stamp_ns <= 0 or now < stamp_ns or
                    now - stamp_ns > min(self.state_freshness_ns, 75_000_000)):
                self._last_dropped_compute_reason = 'PRECOMPUTE_STATE_AGE_UNPROVEN'
                return
            self.receive_state(message)

    def flush_stage_diagnostics(self):
        """Keep optional JSON serialization outside the intent critical path."""
        # Bound work per callback so diagnostics cannot grow indefinitely.
        for _ in range(min(2, len(self._stage_telemetry_queue))):
            timing = String()
            timing.data = json.dumps(self._stage_telemetry_queue.popleft(),sort_keys=True)
            self.stage_timing_pub.publish(timing)

    def receive_state(self, message):
        cycle_started_ns = time.perf_counter_ns()
        now_ns = self.get_clock().now().nanoseconds
        self.last_source_state_stamp = message.physics_stamp
        self.last_source_state_sequence = message.physics_sequence
        self.last_state_ns = now_ns
        state = EgoState(message.px, message.py, message.yaw, message.v)
        self.last_actual_speed = state.speed_mps
        if self.start_ns is None or now_ns < self.start_ns:
            if self.reference_valid and state.finite:
                self.status_pub.publish(build_prestart_status(
                    now_ns,
                    self.reference_id,
                    self.reference_revision,
                    state.speed_mps,
                    self.controller_kind,
                ))
            return
        if (not self.startup_primary_phase_complete and state.finite and
                abs(float(state.speed_mps)) >= 0.50):
            self.startup_primary_phase_complete = True
        dt_s = 0.025 if self.last_command_ns is None else max(
            1e-6, (now_ns - self.last_command_ns) * 1e-9
        )
        stages_us = {}
        self.last_mpc_failure_stage = ""
        self.last_mpc_failure_elapsed_ns = 0
        if self.controller_kind == 'mpc':
            self.primary.last_microstage_times = {}
        def stamp_stage(name):
            stages_us[name] = (time.perf_counter_ns() - cycle_started_ns) / 1000.0
        station = None
        envelope_point = None
        envelope = None
        control_reference = self.reference if self.reference_valid else None
        diagnostic_withheld = (
            self.controller_kind == "mpc" and self.reference_valid and state.finite
            and self.diagnostic_withhold_primary
        )
        if self.controller_kind == "mpc" and self.reference_valid and state.finite and not diagnostic_withheld:
            projection = self.reference.geometry.project(state.x_m, state.y_m)
            track_sample = self.reference.geometry.interpolate(projection.s)
            station = self.reference.station_at_physical(track_sample)
            envelope = self.primary.speed_envelope(
                self.reference, self.primary_speed_cap_mps,
            )
            envelope_point = envelope.at(station)
            control_reference = EnvelopeBoundReference(self.reference, envelope)
        decision = self.controller.compute(
            control_reference, state, self.previous_steering,
            self.previous_acceleration, dt_s,
        )
        stamp_stage("fallback_base")
        bounded_fallback_target = decision.target_speed_mps
        primary_status, solver_iterations, primary_solve_us = "DISABLED", 0, 0.0
        solver_objective = max_constraint_residual = math.nan
        local_speed_limit, local_speed_reason = decision.target_speed_mps, decision.reason
        recoverability = None
        primary_b0_profile = {}  # capture immediately; Stability may call B0 later
        fallback_prepared = None
        fallback_first_us = 0.0
        primary_sequence = int(message.physics_sequence)
        primary_deadline_ns = cycle_started_ns + self.mpc_cycle_slot_ns
        primary_job_submitted = False
        if (self._primary_worker is not None and self.reference_valid and
                state.finite and not diagnostic_withheld and
                self.startup_primary_phase_complete):
            # Capture all the proposal inputs before yielding to the distinct
            # primary worker. Do not copy a mutable controller or query its
            # warm start from this thread.
            primary_ref = self.reference
            primary_stamp = message
            primary_station = station
            primary_previous = (self.previous_steering, self.previous_acceleration)
            primary_cap = self.primary_speed_cap_mps
            primary_solver = self._primary_solver
            if isinstance(self._primary_worker, IsolatedMpcProcessWorker):
                primary_job_submitted = self._primary_worker.submit_request(
                    primary_sequence, primary_ref, primary_stamp,
                    primary_station, primary_previous, primary_cap,
                    primary_deadline_ns)
            else:
                primary_job_submitted = self._primary_worker.submit(
                    primary_sequence,
                    lambda: primary_solver.solve(
                        primary_ref, primary_stamp, primary_station,
                        primary_previous, primary_cap,
                        deadline_ns=primary_deadline_ns),
                    primary_deadline_ns)
        # The V3 failure record proves that running Primary+B0 FIRST leaves
        # the safety fallback's +15 ms preview deadline already expired.
        # Pre-prove fresh fallback in the control worker while the independent
        # primary solver runs. A primary timeout never bypasses fallback or Guard.
        # Do not cache across callbacks, states, reference revisions or ticks.
        if (self.controller_kind == "mpc" and self.reference_valid and
                state.finite and envelope is not None and
                self.startup_primary_phase_complete and not diagnostic_withheld):
            t_fallback = time.perf_counter_ns()
            proposal = replace(
                decision, mode="FALLBACK_PREPARED_BEFORE_PRIMARY",
                reason="SAME_TICK_FRESH_STATE_FALLBACK_PROOF",
            )
            fallback_prepared = self._guard_fallback_recovery(
                proposal, message, state, dt_s,
                preview_deadline_ns=cycle_started_ns + self.fallback_first_slot_ns,
                already_computed_envelope=envelope,
            )
            fallback_first_us = (time.perf_counter_ns()-t_fallback)/1000.0
        if diagnostic_withheld:
            primary_status = "DIAGNOSTIC_WITHHELD"
            admission_state, feasible_duration_s = "FALLBACK_REQUIRED", 0.0
            self.primary_gate.reset()
            decision = replace(
                decision,
                mode="FALLBACK_DIAGNOSTIC_PRIMARY_WITHHELD",
                reason="DIAGNOSTIC_PRIMARY_WITHHELD",
            )
        elif (self.controller_kind == "mpc" and self.reference_valid and state.finite
              and not self.startup_primary_phase_complete
              and abs(state.speed_mps) < 0.50):
            # At rest, a full 44+-dynamics-call MPC is not the bootstrap
            # actuator. The existing Fallback/FinalPreflight/independent Guard
            # must still prove every positive motion command. No direct output.
            primary_status = "PRIMARY_STARTUP_DEFERRED"
            admission_state, feasible_duration_s = "FALLBACK_REQUIRED", 0.0
            # Gate reset occurs only at a new START/reference, never on each
            # low-speed tick. Keep initial fallback certification intact.
            decision = replace(decision, mode="FALLBACK_STARTUP_PRIMARY_DEFERRED",
                               reason="STARTUP_USE_GUARD_CERTIFIED_FALLBACK")
        elif (self.controller_kind == "mpc" and self.reference_valid and
              state.finite and fallback_prepared is not None and
              time.perf_counter_ns() - cycle_started_ns >= 9_000_000):
            # Heavy pre-proof or OS preemption consumed the Primary slot.
            # Do NOT run a late QP merely to lose the final Preflight budget.
            primary_status = "PRIMARY_SKIPPED_PREFLIGHT_TIME_RESERVED"
            _, admission_state, feasible_duration_s = self.primary_gate.update(
                False, now_ns * 1e-9,
            )
            decision = replace(decision, mode="FALLBACK_PRIMARY_TIME_RESERVED",
                               reason="FRESH_FALLBACK_READY_PRIMARY_TIME_EXHAUSTED")
        elif self.controller_kind == "mpc" and self.reference_valid and state.finite:
            mpc_started_ns = time.perf_counter_ns()
            mpc_call_running = True
            try:
                if self._primary_worker is not None:
                    if not primary_job_submitted:
                        raise RuntimeError('MPC_WORKER_BUSY_OR_DEADLINE')
                    primary = self._primary_worker.take(
                        primary_sequence,
                        deadline_ns=min(
                            primary_deadline_ns,
                            mpc_started_ns + int(self.primary.config.primary_deadline_s * 1e9),
                        ),
                    )
                else:
                    primary = self.primary.solve(
                        self.reference, message, station,
                        (self.previous_steering, self.previous_acceleration),
                        self.primary_speed_cap_mps,
                        deadline_ns=min(
                            primary_deadline_ns,
                            mpc_started_ns + int(self.primary.config.primary_deadline_s * 1e9),
                        ),
                    )
                mpc_call_running = False
                primary_status = primary.status
                solver_iterations = primary.iterations
                primary_solve_us = primary.solve_time_s * 1e6
                solver_objective = primary.objective
                max_constraint_residual = primary.max_constraint_residual
                local_speed_limit = primary.target_speed_mps
                local_speed_reason = primary.speed_limit_reason
                if primary.status == "PRIMARY_VALID":
                    recoverability = self.recoverability_governor.evaluate(
                        self.reference,
                        envelope,  # identical current-state authority and revision
                        message,
                        (primary.steering_rad, primary.acceleration_mps2),
                        deadline_ns=cycle_started_ns + self.b0_cycle_deadline_ns,
                    )
                    primary_b0_profile = dict(
                        getattr(self.recoverability_governor, 'last_profile', {}))
                    if not recoverability.recoverable:
                        primary_status = ("PRIMARY_B0_DEADLINE_UNPROVEN" if
                                          recoverability.reason == "B0_DEADLINE_UNPROVEN" else
                                          "PRIMARY_UNRECOVERABLE")
                adopt_primary, admission_state, feasible_duration_s = self.primary_gate.update(
                    primary.status == "PRIMARY_VALID" and
                    recoverability is not None and recoverability.recoverable,
                    now_ns * 1e-9,
                )
                if adopt_primary:
                    decision = replace(
                        decision,
                        steering_rad=primary.steering_rad,
                        acceleration_mps2=primary.acceleration_mps2,
                        mode="PRIMARY_VALID",
                        reason=f"MPC_VALID_ON_TIME:{primary.speed_limit_reason}",
                        target_speed_mps=primary.target_speed_mps,
                    )
                elif primary.status == "PRIMARY_VALID" and recoverability.recoverable:
                    decision = replace(
                        decision,
                        mode="FALLBACK_PRIMARY_ADMISSION",
                        reason="PRIMARY_ADMISSION_PENDING_HORIZON",
                    )
                elif primary.status == "PRIMARY_VALID":
                    decision = replace(
                        decision,
                        mode=f"FALLBACK_{admission_state}",
                        reason=f"PRIMARY_RECOVERABILITY_REJECTED:{recoverability.reason}",
                    )
                else:
                    decision = replace(
                        decision,
                        mode=f"FALLBACK_{admission_state}",
                        reason="MPC_LATE_REJECTED",
                    )
            except RuntimeError as error:
                # The duration is measured even when the solver aborts or
                # reaches its iteration limit; 0 us must never mask CPU cost.
                if mpc_call_running:
                    primary_solve_us = (time.perf_counter_ns() - mpc_started_ns) / 1000.0
                if isinstance(error, MpcCycleBudgetExceeded):
                    solver_iterations = error.iterations
                    self.last_mpc_failure_stage = error.stage
                    self.last_mpc_failure_elapsed_ns = error.elapsed_ns
                # "iteration limit" and "constraint residual" are not
                # proofs of QP infeasibility. Keep safety fallback, but
                # classify the reason precisely for forensic validation.
                failure = str(error)
                known = {
                    "MPC_QP_ITERATION_LIMIT": "PRIMARY_ITERATION_LIMIT",
                    "MPC_QP_CONSTRAINT_RESIDUAL": "PRIMARY_CONSTRAINT_REJECTED",
                    "MPC_CYCLE_BUDGET_EXCEEDED": "PRIMARY_BUDGET_EXHAUSTED",
                    "MPC_WORKER_BUSY_OR_DEADLINE": "PRIMARY_WORKER_BUSY",
                    "MPC_WORKER_RESULT_LATE": "PRIMARY_WORKER_DEADLINE",
                    "MPC_WORKER_RESULT_MISSING_OR_EXPIRED": "PRIMARY_WORKER_EXPIRED",
                    "MPC_WORKER_DEADLINE_EXPIRED": "PRIMARY_WORKER_DEADLINE",
                }
                primary_status = known.get(failure, "PRIMARY_INFEASIBLE")
                _, admission_state, feasible_duration_s = self.primary_gate.update(
                    False, now_ns * 1e-9,
                )
                decision = replace(
                    decision,
                    mode=f"FALLBACK_{admission_state}",
                    reason=(failure if failure in known
                            else "MPC_INFEASIBLE_REJECTED"),
                )
            except (TypeError, ValueError):
                if mpc_call_running:
                    primary_solve_us = (time.perf_counter_ns() - mpc_started_ns) / 1000.0
                primary_status = "PRIMARY_INVALID"
                _, admission_state, feasible_duration_s = self.primary_gate.update(
                    False, now_ns * 1e-9,
                )
                decision = replace(
                    decision,
                    mode=f"FALLBACK_{admission_state}",
                    reason="MPC_INVALID_REJECTED",
                )
        else:
            admission_state, feasible_duration_s = "NOT_APPLICABLE", 0.0
        stamp_stage("mpc_and_recoverability")
        fallback_assessment = None
        # STARTUP FAST PATH, strictly while stationary and nominal. The
        # fallback shaper still enforces steering slew, longitudinal jerk and
        # +4 m/s2 ceiling. Current slip/yaw/real corridor and stability trend
        # must be healthy. Predictive fallback preview is supplementary to,
        # never a replacement for, the full current+next stop proof below.
        # Unlike a generic bypass this path is disabled if any safety margin,
        # recovery state or expected fresh reference is not qualified.
        stationary_startup = False
        if (self.controller_kind == "mpc" and self.reference_valid and
                state.finite and envelope is not None and
                decision.mode.startswith("FALLBACK") and
                abs(float(message.v)) <= 0.025 and
                math.hypot(float(message.vx), float(message.vy)) <= 0.025 and
                abs(float(message.omega)) <= 0.05 and
                abs(float(message.slip_angle)) <= 0.05 and
                self.stability_recovery.mode == "NORMAL" and
                not self.fallback_safety.startup_complete):
            current_ok, margins = self.fallback_safety.current_safety(message)
            entry_trigger = (self.stability_recovery.entry.update(
                now_ns * 1e-9, margins[0], margins[1])
                if current_ok else True)
            if current_ok and not entry_trigger:
                decision, startup_phase = self.fallback_safety.shape(
                    message, decision,
                    (self.previous_steering, self.previous_acceleration), dt_s)
                stationary_startup = True
                # Keep the dynamics trend recorder coherent for the
                # subsequent transition to moving-vehicle control.
                self.dynamic_admission.history.append(
                    (now_ns * 1e-9, float(margins[0]), float(margins[1])))
                local_speed_reason = "STATIONARY_CURRENT_MARGINS_OK_GUARD_PROOF_PENDING"
        if not stationary_startup and decision.mode.startswith("FALLBACK"):
            if fallback_prepared is not None:
                # Same tick, same source CarState and reference; the prepared
                # fallback underwent the ORIGINAL predictive safety check.
                # Unsafe/time-expired preparation returns recovery braking,
                # never an uncertified positive acceleration. Any later
                # dynamic/stability mutation still faces FinalPreflight.
                decision, fallback_assessment = fallback_prepared
                local_speed_reason = "FALLBACK_PREPARED_FIRST:" + decision.reason
            else:
                decision, fallback_assessment = self._guard_fallback_recovery(
                    decision, message, state, dt_s,
                    preview_deadline_ns=cycle_started_ns + self.fallback_preview_slot_ns,
                    already_computed_envelope=envelope,
                )
            if recoverability is None and fallback_assessment is not None:
                recoverability = fallback_assessment

        stamp_stage("fallback_safety")
        if not stationary_startup:
            dynamic_admission = self.dynamic_admission.update(
                message, decision, self.previous_acceleration, now_ns * 1e-9, dt_s)
            if dynamic_admission.active:
                decision = dynamic_admission.decision
                local_speed_limit = decision.target_speed_mps
                local_speed_reason = dynamic_admission.reason
        stamp_stage("dynamic_admission")
        recovery_command = None
        if (not stationary_startup and
            self.controller_kind == "mpc"
            and self.reference_valid
            and state.finite
            and envelope is not None
            and station is not None
        ):
            recovery_command = self.stability_recovery.update(
                self.reference,
                envelope,
                message,
                (self.previous_steering, self.previous_acceleration),
                now_ns * 1e-9,
                dt_s,
            )
            if recovery_command.active:
                self.primary_gate.reset()
                decision = replace(
                    decision,
                    steering_rad=recovery_command.steering_rad,
                    acceleration_mps2=recovery_command.acceleration_mps2,
                    mode=recovery_command.mode,
                    reason=recovery_command.reason,
                    target_speed_mps=recovery_command.target_speed_mps,
                    degradation_active=True,
                )
                primary_status = "RECOVERY_OVERRIDE"
                admission_state = "RECOVERY_OVERRIDE"
                feasible_duration_s = 0.0
                local_speed_limit = recovery_command.target_speed_mps
                local_speed_reason = recovery_command.reason

        stamp_stage("stability_recovery")
        # The last decision mutation from Primary, Fallback, Dynamic Admission
        # or Stability Recovery happened above. All positive proposals now
        # use the current+N+1 StoppingEnvelope before publishing Intent.
        original_mode = str(decision.mode)
        preflight = admit_final_command(
            decision,
            message=message, state_finite=state.finite,
            actual_speed_mps=state.speed_mps,
            reference_valid=self.reference_valid,
            cycle_started_ns=cycle_started_ns,
            reserve_ns=self.stop_precheck_reserve_ns,
            previous_steering=self.previous_steering, dt_s=dt_s,
            guard_ahead=self.guard_ahead, controller=self.controller,
        )
        decision = preflight.decision
        final_preflight = preflight.reason
        if preflight.changed:
            local_speed_reason = decision.reason
            if preflight.rejected:
                local_speed_limit = 0.0
            if original_mode == "PRIMARY_VALID":
                primary_status = ("PRIMARY_STOP_PREFLIGHT_UNPROVEN" if
                                  preflight.rejected else
                                  "PRIMARY_FINAL_PREFLIGHT_MODERATED")
                self.primary_gate.reset()
                admission_state = ("FINAL_PREFLIGHT_UNPROVEN" if
                                   preflight.rejected else
                                   "FINAL_PREFLIGHT_MODERATED")
                feasible_duration_s = 0.0
        # No further mutation occurs before publish(). Independent Guard
        # remains the only final Ackermann authority and may still reject.
        stamp_stage("guard_ahead")
        # Per-tick deadlines concern wall time, not ROS simulation time.
        compute_us = (time.perf_counter_ns() - cycle_started_ns) / 1000.0
        # Freeze source identity at the beginning of publication. Do not
        # refresh a completed command's physics source stamp to a newer state.
        source_stamp_ns = int(message.physics_stamp.sec)*1_000_000_000 + int(message.physics_stamp.nanosec)
        publication_now_ns = self.get_clock().now().nanoseconds
        # With an off-executor worker, a newer physics receipt may supersede
        # this source while the long solve runs. Do not publish an old intent.
        # The last monotonic source check and the actual ROS publication are
        # ONE receipt-locked critical section. Otherwise a newer physics state
        # can arrive between the check and publish(), allowing the worker to
        # publish a superseded intent despite the existing V6 lease check.
        # Guard remains the *only* actuation authority. Holding this lock
        # serializes receipt only across the final bounded publication, never
        # across MPC, fallback, or GuardAhead computation.
        with self._state_receipt_lock:
            source_superseded = (int(message.physics_sequence) <
                                 self._received_state_sequence or
                                 source_stamp_ns < self._last_received_physics_stamp_ns)
            publication_now_ns = self.get_clock().now().nanoseconds
            if (source_superseded or publication_now_ns < source_stamp_ns or
                    publication_now_ns-source_stamp_ns > min(self.state_freshness_ns,75_000_000) or
                    time.perf_counter_ns()-cycle_started_ns > 25_000_000):
                self._last_dropped_compute_reason = 'POSTCOMPUTE_EXPIRED_UNPROVEN'
                # No intent may be published with a false current-state claim.
                # Guard's independent command expiry/deadman remains in force.
                return
            self.publish(
                decision, state.speed_mps, now_ns, compute_us,
                primary_status, solver_iterations, primary_solve_us,
                solver_objective, max_constraint_residual,
                local_speed_limit, local_speed_reason,
                envelope_point, bounded_fallback_target,
                admission_state, feasible_duration_s,
                recoverability,
            )
        # Diagnostics are explicitly NON-authoritative and bounded; ROS
        # intent publish has completed before any JSON encoding occurs.
        timing_data = {
            "physics_sequence": int(message.physics_sequence),
            "primary": primary_status,
            "controller_mode": decision.mode,
            "wall_compute_us": compute_us,
            "wall_after_intent_us": (time.perf_counter_ns() - cycle_started_ns) / 1000.0,
            "mpc_attempt_us": primary_solve_us,
            "mpc_failure_stage": self.last_mpc_failure_stage,
            "mpc_failure_elapsed_ns": self.last_mpc_failure_elapsed_ns,
            "mpc_microstages": (getattr(self._primary_solver, 'last_microstage_times', {})
                                if self.controller_kind == 'mpc' else {}),
            "stage_cumulative_us": stages_us,
            "deadline_us": 25000,
            "primary_deadline_us": int(self.primary.config.primary_deadline_s * 1e6),
            "final_preflight": final_preflight,
            "final_preflight_input_mode": original_mode,
            "final_preflight_changed": bool(preflight.changed),
            "final_preflight_rejected": bool(preflight.rejected),
            "guard_ahead_proof_timing_us": preflight.trace,
            "stop_corridor_proof_all_signs": True,
            "stationary_startup_fast_path": stationary_startup,
            "startup_primary_phase_complete": self.startup_primary_phase_complete,
            "fallback_first_attempted": fallback_prepared is not None,
            "fallback_first_us": fallback_first_us,
            "fallback_first_result": ("NOT_ATTEMPTED" if fallback_prepared is None else
                                      fallback_prepared[0].mode),
            "final_intent_acceleration_mps2": float(decision.acceleration_mps2),
            "final_intent_mode": decision.mode,
            "publication_stages_us": getattr(self, "last_publication_stages_us", {}),
            "b0_cycle_deadline_us": self.b0_cycle_deadline_ns / 1000.0,
            "b0_internal_profile": primary_b0_profile,
        }
        if len(self._stage_telemetry_queue) == self._stage_telemetry_queue.maxlen:
            self._stage_diagnostic_dropped += 1
        self._stage_telemetry_queue.append(timing_data)

    def _guard_fallback_recovery(self, decision, message, state, dt_s,
                                 preview_deadline_ns=None, already_computed_envelope=None):
        # Primary/MPC admission retains strict B0 in receive_state().
        # Fallback uses the separately proven startup + positive-reserve
        # predictive-safety contract from the closed R-B failure map.
        if (self.controller_kind != "mpc" or not self.reference_valid or
                not state.finite or not decision.mode.startswith("FALLBACK")):
            return decision, None
        envelope = already_computed_envelope
        try:
            if envelope is None:
                envelope = self.primary.speed_envelope(
                    self.reference, self.primary_speed_cap_mps)
            transition = self.fallback_safety.evaluate(
                self.reference,
                envelope,
                message,
                decision,
                (self.previous_steering, self.previous_acceleration),
                dt_s,
                deadline_ns=preview_deadline_ns,
            )
            if transition.assessment.recoverable:
                return transition.decision, transition.assessment
            assessment = transition.assessment
            reason = f"FALLBACK_SAFETY_REJECTED:{assessment.reason}"
        except (RuntimeError, TypeError, ValueError, FloatingPointError) as error:
            assessment = None
            reason = f"FALLBACK_SAFETY_EVAL_FAILED:{type(error).__name__}"

        braking = self.controller.degraded(
            self.previous_steering, reason, state.speed_mps, dt_s,
        )
        # Do not assume that freezing the previous steering angle is the
        # safest way to brake. Evaluate bounded candidate steering commands
        # against the *existing* physical corridor and dynamic margins.
        # This is only a short-horizon mitigation; no stop certificate or
        # primary-admission permission is conferred by its result.
        if envelope is None:
            steering, steering_reason = (
                self.previous_steering, "STEER_UNPROVEN_ENVELOPE_UNAVAILABLE"
            )
        elif (preview_deadline_ns is not None and
              time.perf_counter_ns() >= preview_deadline_ns):
            # Independent Guard certifies the brake/steer proposal. Never
            # spend another 8ms on a noncertifying steering mitigation when
            # the callback's reserved time is already exhausted.
            steering, steering_reason = (
                self.previous_steering, "STEER_UNPROVEN_PREVIEW_DEADLINE"
            )
        else:
            steering, steering_reason = self.fallback_safety.select_braking_steer(
                self.reference, envelope, message, decision.steering_rad,
                self.previous_steering, braking.acceleration_mps2, dt_s,
            )
        braking = replace(
            braking,
            steering_rad=steering,
            mode="FALLBACK_RECOVERY_BRAKE",
            reason=f"{reason}:{steering_reason}",
            target_progress_m=decision.target_progress_m,
            target_speed_mps=0.0,
            lateral_error_m=decision.lateral_error_m,
            heading_error_rad=decision.heading_error_rad,
        )
        return braking, assessment

    def watchdog(self):
        with self._control_lock:
            return self._watchdog_locked()

    def _watchdog_locked(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.start_ns is None or now_ns < self.start_ns:
            return
        if self.last_state_ns is None or now_ns - self.last_state_ns > self.state_freshness_ns:
            dt_s = 0.025 if self.last_command_ns is None else max(
                1e-6, (now_ns - self.last_command_ns) * 1e-9
            )
            decision = self.controller.degraded(
                self.previous_steering, "STALE_STATE", self.last_actual_speed, dt_s,
            )
            self.publish(decision, self.last_actual_speed, now_ns, 0.0)

    def publish(self, decision, actual_speed, now_ns, compute_us,
                primary_status="NOT_RUN", solver_iterations=0, primary_solve_us=0.0,
                solver_objective=math.nan, max_constraint_residual=math.nan,
                local_speed_limit=math.nan, local_speed_reason="",
                envelope_point=None, bounded_fallback_target=math.nan,
                admission_state="NOT_APPLICABLE", feasible_duration_s=0.0,
                recoverability=None):
        # Only independent Guard publishes the final Ackermann; this is an
        # expiring proposal and has no actuation authority.
        self.intent_sequence += 1
        intent = ActuatorIntent()
        intent.source_id = "R_B"
        intent.run_epoch = self.get_parameter("guard_run_epoch").value
        intent.sequence = self.intent_sequence
        issue_time = self.get_clock().now().to_msg()
        intent.created_ros_stamp = issue_time
        issue_ns = issue_time.sec*1000000000 + issue_time.nanosec
        expiry_ns = issue_ns + 75000000
        intent.expires_ros_stamp = Time(sec=expiry_ns//1000000000, nanosec=expiry_ns%1000000000)
        intent.source_state_stamp = self.last_source_state_stamp
        intent.source_state_sequence = self.last_source_state_sequence
        intent.active_reference_id = self.reference_id
        intent.active_revision = self.reference_revision
        intent.track_id = self.get_parameter("expected_track_id").value
        intent.map_sha256 = self.get_parameter("expected_map_sha256").value
        intent.candidate_mode = decision.mode
        intent.steering_rad = float(decision.steering_rad)
        intent.acceleration_mps2 = float(decision.acceleration_mps2)
        intent_pre_publish_ns = time.perf_counter_ns()
        self.command_pub.publish(intent)
        intent_post_publish_ns = time.perf_counter_ns()
        self.previous_steering = decision.steering_rad
        self.previous_acceleration = decision.acceleration_mps2
        self.last_command_ns = issue_ns
        status_started_ns = time.perf_counter_ns()
        status = ControllerStatus(
            receipt_time_ns=now_ns,
            mode=decision.mode,
            reason=decision.reason,
            active_reference_id=self.reference_id,
            active_revision=self.reference_revision,
            target_progress_m=decision.target_progress_m,
            target_speed_mps=decision.target_speed_mps,
            actual_speed_mps=actual_speed,
            lateral_error_m=decision.lateral_error_m,
            heading_error_rad=decision.heading_error_rad,
            steering_command_rad=decision.steering_rad,
            acceleration_command_mps2=decision.acceleration_mps2,
            qualification_cap_active=decision.qualification_cap_active,
            degradation_active=decision.degradation_active,
            compute_time_us=compute_us,
            primary_status=primary_status,
            solver_iterations=solver_iterations,
            primary_solve_time_us=primary_solve_us,
            fallback_active=decision.mode.startswith("FALLBACK"),
            controller_kind=self.controller_kind,
            solver_formulation_id=(
                ('NMPC_NONLINEAR_DYNAMICS_SLSQP_V1' if self.mpc_formulation == 'nmpc'
                 else 'LTV_PACEJKA_ADMM_V2')
                if self.controller_kind == 'mpc' else 'NONE'),
            horizon_steps=(self.primary.config.horizon_steps
                           if self.controller_kind == "mpc" else 0),
            solver_timestep_s=(self.primary.config.timestep_s
                               if self.controller_kind == "mpc" else 0.0),
            local_speed_limit_mps=(decision.target_speed_mps
                                   if not math.isfinite(local_speed_limit)
                                   else local_speed_limit),
            local_speed_limit_reason=local_speed_reason or decision.reason,
            solver_objective=solver_objective,
            max_constraint_residual=max_constraint_residual,
            raw_dynamic_limit_mps=(envelope_point.raw_dynamic_limit_mps
                                   if envelope_point else math.nan),
            backward_reachable_limit_mps=(envelope_point.backward_reachable_limit_mps
                                          if envelope_point else math.nan),
            execution_envelope_mps=(envelope_point.execution_envelope_mps
                                    if envelope_point else math.nan),
            effective_preview_distance_m=(envelope_point.effective_preview_distance_m
                                          if envelope_point else math.nan),
            required_braking_distance_m=(envelope_point.required_braking_distance_m
                                         if envelope_point else math.nan),
            fallback_cap_mps=(self.controller.config.qualified_speed_cap_mps
                              if self.controller_kind == "mpc" else math.nan),
            bounded_fallback_target_mps=bounded_fallback_target,
            primary_admission_state=admission_state,
            feasibility_duration_s=feasible_duration_s,
            predicted_terminal_recoverable=primary_status == "PRIMARY_VALID",
            recoverability_state=("NOT_EVALUATED" if recoverability is None else
                                  "RECOVERABLE" if recoverability.recoverable else
                                  "UNRECOVERABLE"),
            recoverability_reason=("" if recoverability is None else recoverability.reason),
            recoverability_margin=(math.nan if recoverability is None else
                                   recoverability.initial_margin),
            terminal_recoverability_margin=(math.nan if recoverability is None else
                                            recoverability.terminal_margin),
            predicted_terminal_x_m=(math.nan if recoverability is None else
                                    recoverability.terminal_state[0]),
            predicted_terminal_y_m=(math.nan if recoverability is None else
                                    recoverability.terminal_state[1]),
            predicted_terminal_yaw_rad=(math.nan if recoverability is None else
                                       recoverability.terminal_state[2]),
            predicted_terminal_speed_mps=(math.nan if recoverability is None else
                                         recoverability.terminal_state[3]),
            predicted_terminal_slip_rad=(math.nan if recoverability is None else
                                        recoverability.terminal_state[4]),
            predicted_terminal_yaw_rate_rad_s=(math.nan if recoverability is None else
                                              recoverability.terminal_state[5]),
        )
        status_constructed_ns = time.perf_counter_ns()
        self.status_pub.publish(status)
        status_published_ns = time.perf_counter_ns()
        # Diagnostic-only timing; no publication is skipped, moved off the
        # measured path, or declared complete before returning. These fields
        # do not affect Guard, controller or credential authority.
        self.last_publication_stages_us = {
            "intent_publish_us": (intent_post_publish_ns - intent_pre_publish_ns) / 1000.0,
            "status_construct_us": (status_constructed_ns - status_started_ns) / 1000.0,
            "status_publish_us": (status_published_ns - status_constructed_ns) / 1000.0,
            "intent_to_status_end_us": (status_published_ns - intent_post_publish_ns) / 1000.0,
        }


# === FORENSIC CALLBACK WRAPPERS: diagnostics only; no safety/control branch changes ===
from f1tenth_racing_control.forensic_trace import trace_rb as _trace

_rb_original_state_receipt = RacingPursuitController.receive_state_snapshot
def _rb_observed_receipt(self,msg):
    tick=time.monotonic_ns()
    _trace('RB_STATE_ENTER',physics_sequence=int(msg.physics_sequence),
           physics_stamp_ns=int(msg.physics_stamp.sec)*1000000000+int(msg.physics_stamp.nanosec),
           prior_sequence=int(getattr(self,'_received_state_sequence',-1)),
           controller_last_state_ns=getattr(self,'last_state_ns',None),
           start_ns=getattr(self,'start_ns',None))
    try:
        return _rb_original_state_receipt(self,msg)
    finally:
        _trace('RB_STATE_EXIT',physics_sequence=int(msg.physics_sequence),
               callback_elapsed_ns=time.monotonic_ns()-tick)
RacingPursuitController.receive_state_snapshot=_rb_observed_receipt

_rb_original_state = RacingPursuitController.receive_state
def _rb_observed_state(self, msg):
    import time
    tick = time.monotonic_ns()
    _trace('RB_CYCLE_ENTER', physics_sequence=int(msg.physics_sequence),
           physics_stamp_ns=int(msg.physics_stamp.sec)*1000000000+int(msg.physics_stamp.nanosec),
           prior_sequence=int(getattr(self,'last_source_state_sequence',-1)),
           controller_last_state_ns=getattr(self,'last_state_ns',None),
           start_ns=getattr(self,'start_ns',None))
    try:
        return _rb_original_state(self,msg)
    finally:
        _trace('RB_CYCLE_EXIT', physics_sequence=int(msg.physics_sequence),
               callback_elapsed_ns=time.monotonic_ns()-tick)
RacingPursuitController.receive_state = _rb_observed_state

_rb_original_watchdog = RacingPursuitController.watchdog
def _rb_observed_watchdog(self):
    import time
    s=getattr(self,'last_source_state_stamp',None)
    st=(int(s.sec)*1000000000+int(s.nanosec)) if s is not None else None
    now=self.get_clock().now().nanoseconds
    _trace('RB_WATCHDOG_ENTER', source_sequence=int(getattr(self,'last_source_state_sequence',-1)),
           last_state_ros_ns=getattr(self,'last_state_ns',None), source_stamp_ros_ns=st,
           ros_now_ns=now, start_ns=getattr(self,'start_ns',None))
    return _rb_original_watchdog(self)
RacingPursuitController.watchdog = _rb_observed_watchdog

_rb_original_reference = RacingPursuitController.receive_reference
def _rb_observed_reference(self,msg):
    import time
    t=time.monotonic_ns()
    _trace('RB_REFERENCE_ENTER', revision=int(getattr(msg,'active_revision',-1)),
           sample_count=int(getattr(msg,'sample_count',-1)))
    try:return _rb_original_reference(self,msg)
    finally:_trace('RB_REFERENCE_EXIT',callback_elapsed_ns=time.monotonic_ns()-t)
RacingPursuitController.receive_reference = _rb_observed_reference

_rb_original_start = RacingPursuitController.receive_start
def _rb_observed_start(self,msg):
    _trace('RB_START_ENTER', start_stamp_ns=int(msg.sec)*1000000000+int(msg.nanosec))
    return _rb_original_start(self,msg)
RacingPursuitController.receive_start = _rb_observed_start

_rb_original_publish = RacingPursuitController.publish
def _rb_observed_publish(self,decision,actual_speed,now_ns,compute_us,*args,**kwargs):
    s=getattr(self,'last_source_state_stamp',None)
    st=(int(s.sec)*1000000000+int(s.nanosec)) if s is not None else None
    _trace('RB_INTENT_EMIT',source_sequence=int(getattr(self,'last_source_state_sequence',-1)),
           physics_stamp_ros_ns=st, controller_mode=str(decision.mode),
           ros_now_ns=int(now_ns), compute_us=float(compute_us),
           source_age_ros_ns=(int(now_ns)-st) if st is not None else None)
    return _rb_original_publish(self,decision,actual_speed,now_ns,compute_us,*args,**kwargs)
RacingPursuitController.publish = _rb_observed_publish

def main():
    rclpy.init()
    node = RacingPursuitController()
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
