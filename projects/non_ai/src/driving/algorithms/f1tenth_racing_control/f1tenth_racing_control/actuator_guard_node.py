"""Independent, fail-closed Ego actuator arbiter.

No dynamic qualification is implied. Certificate calculation deliberately
refuses running when state/map/timing provenance is absent.
"""
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from control_msgs.msg import (ActuatorIntent, ActuatorGuardStatus,
                              ActiveTargetTrajectory, CarState, OpponentPrediction, GuardedActuatorCommand)
from f1tenth_racing_sim.physics.vehicle import State
from f1tenth_racing_sim.safety.stopping_envelope import StoppingEnvelope
from f1tenth_racing_sim.world.map import OccupancyMap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .actuator_guard_contract import (ActuatorGuardContract,
                                      ActuatorIntent as IntentContract)
from .state_timeline import StateTimeline, StateReceipt, ReceiptStatus, BindingStatus
from .red_dynamic_guard import RedDynamicWitness
from .guard_state_integrity import finite_physics_state


def ns(t):
    return int(t.sec)*1000000000+int(t.nanosec)


class IndependentActuatorGuard(Node):
    def __init__(self):
        super().__init__('actuator_guard0')
        share=Path(get_package_share_directory('f1tenth_racing_sim'))
        self.declare_parameter('environment_file',str(share/'config/competition_environment.yaml'))
        self.declare_parameter('map_file','')
        self.declare_parameter('track_id','')
        self.declare_parameter('map_sha256','')
        self.declare_parameter('run_epoch','')
        self.declare_parameter('producer','R_B')
        self.declare_parameter('solo',True)
        self.declare_parameter('dynamic_red_guard_enabled',False)
        self.declare_parameter('max_speed_cap_mps',0.0)
        self.declare_parameter('sim_online_adaptive',False)
        # EXPERIMENT ONLY. Default Guard remains full independent Python
        # StoppingEnvelope; native rollout requires explicit parity receipt.
        self.declare_parameter('experimental_native_guard',False)
        self.declare_parameter('native_guard_library','')
        self.declare_parameter('native_guard_receipt','')
        # Independent C++ physics/map VETO on every Python Guard proof.
        # Disabled by default until per-map native qualification. No new authority.
        self.declare_parameter('native_dual_veto_required',False)
        self.declare_parameter('state_age_ns',100000000)
        self.sim_online_adaptive=bool(self.get_parameter('sim_online_adaptive').value)
        proposed_cap=float(self.get_parameter('max_speed_cap_mps').value)
        if (not math.isfinite(proposed_cap) or proposed_cap<=0.0 or
                (proposed_cap>5.8 and not self.sim_online_adaptive)):
            raise ValueError('GUARD_HIGH_SPEED_INDEPENDENT_QUALIFICATION_MISSING')
        self.solo=bool(self.get_parameter('solo').value)
        self.red_guard_enabled=bool(self.get_parameter('dynamic_red_guard_enabled').value)
        if not self.solo and not self.red_guard_enabled:
            raise ValueError('H2H_REQUIRES_INDEPENDENT_DYNAMIC_RED_GUARD')
        if self.solo and self.red_guard_enabled:
            raise ValueError('DYNAMIC_RED_GUARD_REQUIRES_H2H')
        if not self.solo and not self.sim_online_adaptive:
            raise ValueError('H2H_ONLINE_SIM_ONLY')
        self.environment=yaml.safe_load(Path(self.get_parameter('environment_file').value).read_text())
        physics_bound=float(self.environment['dynamics']['limits']['speed_abs_max_m_s'])
        if not (0.0<proposed_cap<=physics_bound):
            raise ValueError('GUARD_CAP_EXCEEDS_SIM_PHYSICS_LIMIT')
        if self.sim_online_adaptive and not self.environment.get('simulation'):
            raise ValueError('GUARD_SIM_ADAPTIVE_REQUIRES_SIM_ENVIRONMENT')
        ident=self.environment['environment_identity']['active_track']
        map_hash=self.environment['map']['image_sha256']
        if not (self.get_parameter('run_epoch').value and
                self.get_parameter('track_id').value==ident and
                self.get_parameter('map_sha256').value==map_hash):
            raise ValueError('GUARD_LAUNCH_IDENTITY_MISMATCH')
        candidate_map=(self.get_parameter('map_file').value or
                       str(share/self.environment['map']['image_file']))
        import hashlib
        if hashlib.sha256(Path(candidate_map).read_bytes()).hexdigest()!=map_hash:
            raise ValueError('GUARD_MAP_SHA_MISMATCH')
        self.occupancy=OccupancyMap(candidate_map,self.environment['map'])
        footprint=self.environment['collision']['footprint_relative_to_base_link']
        self.envelope=StoppingEnvelope(self.environment['dynamics'],self.occupancy,
            footprint,reaction_s=0.25,uncertainty_padding_m=0.03)
        self.ego_collision_radius_m=max(math.hypot(x,y) for x in
            (footprint['min_x_m'],footprint['max_x_m']) for y in
            (footprint['min_y_m'],footprint['max_y_m']))+0.03
        self.red_prediction=None
        self.red_prediction_received_steady_ns=None
        if bool(self.get_parameter('experimental_native_guard').value):
            if not self.sim_online_adaptive or not self.get_parameter('solo').value:
                raise ValueError('EXPERIMENTAL_NATIVE_GUARD_REQUIRES_SIM_SOLO')
            from f1tenth_racing_sim.safety.native_stop_proof import (
                ExperimentalNativeProofProxy, check_experimental_parity_receipt)
            lib_path=str(self.get_parameter('native_guard_library').value)
            receipt_path=str(self.get_parameter('native_guard_receipt').value)
            if not lib_path or not receipt_path:
                raise ValueError('NATIVE_GUARD_PROVENANCE_MISSING')
            check_experimental_parity_receipt(lib_path,receipt_path,
                expected_map_sha256=map_hash,active_map_file=candidate_map)
            # Every Intent still gets a new, independent complete stopping
            # rollout against the current Guard physics state and map.
            # Never reuse R-B certificates or admit H2H without Red proof.
            self.envelope=ExperimentalNativeProofProxy(self.envelope,lib_path)
        self.native_dual_proof = None
        if bool(self.get_parameter('native_dual_veto_required').value):
            # Native proof must use the installed ROS package, not an arbitrary
            # caller-provided shared object. Pin its implementation source to
            # this live simulator's independent physics implementation.
            import hashlib
            from ament_index_python.packages import get_package_prefix
            from f1tenth_racing_sim.safety.native_stop_proof import ExperimentalNativeStopProof
            import f1tenth_racing_sim.safety.stopping_envelope as stop_module
            native_share=Path(get_package_share_directory('f1tenth_racing_native_proof'))
            installed_source=native_share/'source/native_stop_proof.cpp'
            live_source=Path(stop_module.__file__).with_name('native_stop_proof.cpp')
            lib_path=Path(get_package_prefix('f1tenth_racing_native_proof'))/'lib/libf1tenth_native_stop.so'
            if (not installed_source.is_file() or not live_source.is_file() or
                    not lib_path.is_file() or
                    hashlib.sha256(installed_source.read_bytes()).digest()!=
                    hashlib.sha256(live_source.read_bytes()).digest()):
                raise ValueError('NATIVE_DUAL_VETO_SOURCE_OR_BINARY_UNAVAILABLE')
            if bool(self.get_parameter('experimental_native_guard').value):
                raise ValueError('DUAL_VETO_INCOMPATIBLE_WITH_NATIVE_REPLACEMENT')
            self.native_dual_proof=ExperimentalNativeStopProof(self.envelope,lib_path)
        self.identity=(self.get_parameter('run_epoch').value,
            self.get_parameter('producer').value,ident,map_hash)
        if self.identity[1] not in ('R_B','FOUNDATION_BASELINE'):
            raise ValueError('UNKNOWN_GUARD_PRODUCER')
        self.contract=ActuatorGuardContract()
        self.last_state=None
        self.previous_state=None  # Previous physics tick for bounded latency check
        self.state_received_steady_ns=None
        self.last_state_sequence=-1
        # Exact sequence ledger; never converts a missing state into proof.
        self.state_timeline=StateTimeline(depth=16, max_age_ns=100_000_000)
        self.state_receipt_gap_count=0
        self.stop_proof_slow_count=0
        self.stop_proof_max_duration_us=0.0
        self.reference_id=('FOUNDATION_BASELINE' if self.identity[1]=='FOUNDATION_BASELINE' else '')
        self.reference_revision=0
        self.start_ns=None
        self.last_ros_ns=None
        self.last_intent_steady_ns=None
        # Monotonic deadline starts only when scheduled ROS start has elapsed.
        self.first_intent_wait_started_steady_ns=None
        self.last_reason='PRESTART_NO_STATE'
        self.last_accepted_sequence=0
        self.last_certificate=False
        # Diagnostic-only receipt; NEVER participates in Guard approval.
        self._stop_probe_last_detail=None
        self._stop_probe_emitted=False
        self.collision=False
        self.emergency_unrecoverable=False
        # The startup-only recoverable latch cannot clear a collision,
        # moving-vehicle incident, or an independently rejected certificate.
        self.startup_waiting_for_fresh_state=False
        self.h2h_abort_park_recovery=False
        self.startup_recovery_fresh_samples=0
        self.last_recovery_physics_sequence=-1
        self.last_recovery_ros_stamp_ns=-1
        self.last_recovery_steady_ns=-1
        self.recovery_first_sample_steady_ns=None
        self.recovery_samples_required=2
        self.last_accepted_acceleration_mps2=0.0
        # An Intent referencing exactly the NEXT physics tick may arrive
        # before the RELIABLE state subscriber callback. It has NO authority
        # while pending; state arrival triggers the FULL existing certificate.
        self.pending_future_intent=None
        self.pending_future_deadline_steady_ns=None
        self.pending_future_received_steady_ns=None
        self.pending_future_diagnostics_emitted=0
        # Diagnostic only: never affects acceptance, stop or reARM.
        self.previous_state_received_steady_ns=None
        self.state_gap_diagnostics_emitted=0
        self.watchdog_gap_diagnostic_emitted=False
        command_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
        # Match the Simulator's protected state0 publisher history.
        # This preserves intermediate physics ticks during bounded callback
        # stalls; it is not a promise of lossless execution under overload.
        state_sensor=QoSProfile(depth=16,reliability=ReliabilityPolicy.RELIABLE)
        self.final_pub=self.create_publisher(AckermannDriveStamped,'ackermann_cmd0',command_qos)
        self.atomic_pub=self.create_publisher(GuardedActuatorCommand,
                                              'guarded_actuator_cmd0',command_qos)
        self.status_pub=self.create_publisher(ActuatorGuardStatus,'actuator_guard_status0',latched)
        self.intent_receipt_pub=self.create_publisher(String,
            'actuator_guard_intent_receipt0',sensor)
        self.create_subscription(ActuatorIntent,'actuator_intent0',self.on_intent,command_qos)
        self.create_subscription(CarState,'state0',self.on_state,state_sensor)
        self.create_subscription(ActiveTargetTrajectory,'active_target_trajectory',
                                 self.on_reference,latched)
        self.create_subscription(Time,'race_start_time',self.on_start,latched)
        self.create_subscription(Bool,'collision0',self.on_collision,sensor)
        self.create_subscription(Bool,'vehicle_collision',self.on_collision,sensor)
        if self.red_guard_enabled:
            self.create_subscription(OpponentPrediction,'opponent_prediction',
                                     self.on_red_prediction,sensor)
        self.timer=self.create_timer(0.025,self.watchdog)

    def on_state(self,msg):
        if msg.physics_sequence<=self.last_state_sequence:
            # State regression is a hard integrity failure, whether or not an
            # Intent happens to be pending. Never leave a previously accepted
            # actuator lease alive while retaining an ARMED/RUNNING identity.
            self._brake('STATE_SEQUENCE_NOT_INCREASING')
            return
        prev_state=self.last_state
        prev_receive_ns=self.state_received_steady_ns
        receive_ns=time.monotonic_ns()
        if (prev_state is not None and prev_receive_ns is not None and
                receive_ns-prev_receive_ns>75000000 and
                self.state_gap_diagnostics_emitted<16):
            self.state_gap_diagnostics_emitted+=1
            try:
                self.get_logger().warning('F1TENTH_GUARD_STATE_RX_GAP='+json.dumps({
                    'previous_sequence':int(prev_state.physics_sequence),
                    'current_sequence':int(msg.physics_sequence),
                    'receive_gap_steady_ns':int(receive_ns-prev_receive_ns),
                    'physics_stamp_gap_ros_ns':int(ns(msg.physics_stamp)-ns(prev_state.physics_stamp)),
                    'physics_stamp_age_ros_ns':int(self.get_clock().now().nanoseconds-ns(msg.physics_stamp)),
                    'accepted_sequence':int(self.last_accepted_sequence),
                    'guard_mode':str(self.contract.state),
                },sort_keys=True))
            except (AttributeError,ValueError,TypeError):
                pass  # diagnostics must NEVER change Guard authority
        status=self.state_timeline.append(StateReceipt(
            sequence=int(msg.physics_sequence),
            stamp_ros_ns=ns(msg.physics_stamp),
            received_steady_ns=receive_ns))
        if status == ReceiptStatus.STAMP_REGRESSION:
            self._brake('PHYSICS_STAMP_NOT_INCREASING')
            return
        if status == ReceiptStatus.SEQUENCE_REGRESSION:
            self._brake('STATE_TIMELINE_SEQUENCE_REGRESSION')
            return
        if status == ReceiptStatus.GAP_OBSERVED:
            self.state_receipt_gap_count += 1
            if self.state_receipt_gap_count <= 24:
                self.get_logger().warning('F1TENTH_GUARD_MISSING_PHYSICS_TICKS='+json.dumps({
                    'missing_range':self.state_timeline.missing_ranges[-1],
                    'received_sequence':int(msg.physics_sequence),
                    'total_gaps':self.state_receipt_gap_count,
                    'max_receive_gap_ns':self.state_timeline.maximum_receive_gap_ns,
                },sort_keys=True))
        self.previous_state=prev_state
        self.last_state=msg
        self.last_state_sequence=msg.physics_sequence
        self.state_received_steady_ns=receive_ns
        if self.startup_waiting_for_fresh_state:
            current_steady=self.state_received_steady_ns
            stamp_ns=ns(msg.physics_stamp)
            age=self.get_clock().now().nanoseconds-stamp_ns
            fresh_ros=(0 <= age <= 50000000)
            stationary=(abs(msg.v)<=0.025 and
                        math.hypot(msg.vx,msg.vy)<=0.025 and
                        abs(msg.omega)<=0.05 and abs(msg.a)<=0.25 and
                        float(msg.accel)<=0.05)
            increasing=(msg.physics_sequence>self.last_recovery_physics_sequence and
                        (self.last_recovery_ros_stamp_ns<0 or
                         0<stamp_ns-self.last_recovery_ros_stamp_ns<=35000000) and
                        (self.last_recovery_steady_ns<0 or
                         0<current_steady-self.last_recovery_steady_ns<=70000000))
            if (fresh_ros and stationary and increasing and not self.collision
                    and not self.emergency_unrecoverable):
                if self.recovery_first_sample_steady_ns is None:
                    self.recovery_first_sample_steady_ns=current_steady
                self.startup_recovery_fresh_samples+=1
                self.last_recovery_physics_sequence=msg.physics_sequence
                self.last_recovery_ros_stamp_ns=stamp_ns
                self.last_recovery_steady_ns=current_steady
            else:
                self.startup_recovery_fresh_samples=0
                self.recovery_first_sample_steady_ns=None
                self.last_recovery_physics_sequence=msg.physics_sequence
                self.last_recovery_ros_stamp_ns=stamp_ns
                self.last_recovery_steady_ns=current_steady
            minimum_elapsed=(75000000 if self.last_accepted_sequence>0 else 25000000)
            if (self.startup_recovery_fresh_samples>=self.recovery_samples_required
                    and self.recovery_first_sample_steady_ns is not None and
                    current_steady-self.recovery_first_sample_steady_ns>=minimum_elapsed):
                self.startup_waiting_for_fresh_state=False
                self.first_intent_wait_started_steady_ns=None
                # Previous healthy accepted sequence is audit history; the
                # *contract* will independently reset its replay counter on
                # arm(). A fresh ActuatorIntent and new stop proof are needed.
        self._settle_pending_future_intent()

    def _settle_pending_future_intent(self):
        """Resolve only against the exact newly received physics tick.

        Never model-predict a missing state or grant approval based on a
        sender's timestamp. Fail closed on a missing/overtaken/late tick.
        """
        msg=self.pending_future_intent
        if msg is None:
            return
        now_steady=time.monotonic_ns()
        now_ros=self.get_clock().now().nanoseconds
        expired=(now_steady>self.pending_future_deadline_steady_ns or
                 now_ros>ns(msg.expires_ros_stamp))
        ahead=(self.last_state_sequence<int(msg.source_state_sequence))
        if ahead and not expired:
            return
        self.pending_future_intent=None
        self.pending_future_deadline_steady_ns=None
        self.pending_future_received_steady_ns=None
        if expired:
            self._brake('FUTURE_STATE_WAIT_DEADLINE_EXCEEDED')
            return
        binding=self.state_timeline.binding(
            int(msg.source_state_sequence), ns(msg.source_state_stamp),
            now_ros, now_steady)
        if (binding is not BindingStatus.EXACT_LATEST or
                self.last_state_sequence!=int(msg.source_state_sequence) or
                ns(self.last_state.physics_stamp)!=ns(msg.source_state_stamp)):
            self._brake('FUTURE_STATE_BINDING_MISMATCH:'+binding.value)
            return
        if (self.start_ns is None or now_ros<self.start_ns or
                self.contract.state not in ('ARMED','RUNNING') or
                self.collision or self.emergency_unrecoverable):
            self._brake('FUTURE_STATE_UNARMED_OR_COLLISION')
            return
        # Exactly matching CarState has now arrived. This still runs the
        # ORIGINAL full local stop envelope and Guard replay/age contract.
        self._validate_and_publish_intent(msg)

    def on_reference(self,msg):
        # Reference revision changes invalidate even an unissued proposal.
        if (self.pending_future_intent is not None and
                (not msg.active_reference_valid or
                 msg.active_reference_id!=self.reference_id or
                 msg.active_revision!=self.reference_revision)):
            self.pending_future_intent=None
            self.pending_future_deadline_steady_ns=None
            self.pending_future_received_steady_ns=None
        if self.identity[1]!='R_B':return
        if (msg.active_reference_valid and msg.track_id==self.identity[2] and
                msg.map_image_sha256==self.identity[3]):
            # A new reference must be re-armed while not moving; preserving
            # an old reference certificate across revisions is prohibited.
            if (msg.active_reference_id != self.reference_id or
                    msg.active_revision != self.reference_revision):
                if self.contract.state=='RUNNING' and not self.emergency_unrecoverable:
                    now_ros=self.get_clock().now().nanoseconds
                    transfer=self.contract.transfer_reference(
                        next_reference_id=msg.active_reference_id,
                        ros_time_ns=now_ros,steady_time_ns=time.monotonic_ns())
                    if not transfer.accepted:
                        self._brake('REFERENCE_TRANSFER_REJECTED:'+transfer.reason)
                        return
                    # No certificate carries over across an active revision.
                    # Next Intent MUST independently pass new stop proof.
                    self.last_certificate=False
                else:
                    self.contract.state='SAFE_HOLD'
                self.reference_id=msg.active_reference_id
                self.reference_revision=msg.active_revision
        else:
            # A rejected reference must revoke the previous reference's
            # ability to re-arm; never retain an old valid identity.
            self.reference_id=''
            self.reference_revision=0
            self.last_certificate=False
            if self.start_ns is not None:
                if (not self.solo and
                        str(msg.status_reason).startswith(
                            'H2H_ABORT_CONTINUATION_EXPIRED:')):
                    self.h2h_abort_park_recovery=True
                    self._brake('H2H_ABORT_REFERENCE_WITHDRAWN')
                else:
                    self._brake('REFERENCE_IDENTITY_REJECTED')
            else:
                self.contract.state='SAFE_HOLD'
                if not self.emergency_unrecoverable:
                    self.last_reason='PRESTART_REFERENCE_INVALID'

    def on_start(self,msg):
        self.start_ns=ns(msg)

    def on_red_prediction(self,msg):
        # Invalid samples REVOKE the prior witness, not just stop an update.
        self.red_prediction=msg if msg.valid else None
        self.red_prediction_received_steady_ns=time.monotonic_ns()

    def on_collision(self,msg):
        if msg.data:
            self.pending_future_intent=None
            self.pending_future_deadline_steady_ns=None
            self.pending_future_received_steady_ns=None
            self.collision=True
            self._brake('COLLISION_LATCHED')
            self.last_reason='COLLISION_LATCHED'

    def _arm_if_ready(self):
        if (self.contract.state not in ('BOOT','SAFE_HOLD') or
                not self.reference_id or self.last_state is None or self.collision):
            return
        if self.emergency_unrecoverable or abs(self.last_state.v)>0.025:
            return
        if self.startup_waiting_for_fresh_state:
            return
        if self.get_clock().now().nanoseconds-ns(self.last_state.physics_stamp)>100000000:
            return
        if self.state_received_steady_ns is None or time.monotonic_ns()-self.state_received_steady_ns>100000000:
            return
        now=self.get_clock().now().nanoseconds
        result=self.contract.arm(run_epoch=self.identity[0],source_id=self.identity[1],
            reference_id=self.reference_id,track_id=self.identity[2],
            map_sha256=self.identity[3],ros_time_ns=now,
            steady_time_ns=time.monotonic_ns())
        self.last_reason=result.reason

    def _state(self):
        m=self.last_state
        return State(x=m.px,y=m.py,yaw=m.yaw,speed=m.v,vx=m.vx,vy=m.vy,
                     yaw_rate=m.omega,acceleration=m.a,ax=m.ax,ay=m.ay,
                     slip=m.slip_angle,requested_acceleration=m.accel,
                     jerk=m.jerk,steering=m.steer,steering_rate=m.steer_vel)

    def _certificate(self,msg):
        # The certificate is computed locally from the latest physics tick.
        # A sender-supplied string or reference identifier never grants it.
        if self.last_state is None:return False
        if abs(self.last_state.v)>self.get_parameter('max_speed_cap_mps').value+0.05:
            return False
        if (not math.isfinite(msg.steering_rad) or
                not math.isfinite(msg.acceleration_mps2)):
            return False
        lim=self.environment['dynamics']['limits']
        if (abs(msg.steering_rad)>lim['steering_angle_abs_max_rad'] or
                msg.acceleration_mps2>lim['acceleration_max_m_s2'] or
                msg.acceleration_mps2< -lim['deceleration_max_m_s2']):
            return False
        if self.identity[1]=='R_B' and msg.active_revision!=self.reference_revision:return False
        # Source-state binding: exact match, or ONE preceding 25ms tick.
        # For the lagging tick case the proposal is NEVER certified against
        # the stale state: evaluate the command from latest Guard physics.
        if self.last_state_sequence==msg.source_state_sequence:
            if ns(self.last_state.physics_stamp)!=ns(msg.source_state_stamp):return False
        else:
            prev=self.previous_state
            if (prev is None or self.last_state_sequence!=msg.source_state_sequence+1 or
                    prev.physics_sequence!=msg.source_state_sequence or
                    ns(prev.physics_stamp)!=ns(msg.source_state_stamp)):
                return False
            delta_ns=ns(self.last_state.physics_stamp)-ns(prev.physics_stamp)
            now_ns=self.get_clock().now().nanoseconds
            if not (0<delta_ns<=self.contract.state_age_limit_ns and
                    0<=now_ns-ns(prev.physics_stamp)<=self.contract.state_age_limit_ns):
                return False
            # Fail closed if successive physical states are discontinuous.
            # A CarState sequence increment is ONE simulator physics step.
            # Simulator integrates physics_timestep_s (fixed 25 ms in the
            # current model), but physics_stamp is the ROS publication time;
            # timer jitter may make consecutive wall timestamps 15/35 ms.
            # Keep those timestamps for STRICT monotonicity/freshness above,
            # never use their delta as the physical integration duration.
            sim=self.environment['simulation']
            tick_s=float(sim['physics_timestep_s'])
            rate_hz=float(sim['physics_rate_hz'])
            if (not math.isfinite(tick_s) or tick_s<=0.0 or
                    not math.isfinite(rate_hz) or rate_hz<=0.0 or
                    not math.isclose(tick_s,1.0/rate_hz,
                                     rel_tol=1e-9,abs_tol=1e-12)):
                self._stop_probe_last_detail={'branch':'PHYSICS_TICK_CONTRACT_INVALID'}
                return False
            if not all(math.isfinite(v) for v in (
                    prev.px,prev.py,prev.v,prev.yaw,
                    self.last_state.px,self.last_state.py,
                    self.last_state.v,self.last_state.yaw)):
                self._stop_probe_last_detail={'branch':'NONFINITE_STATE_TRANSITION'}
                return False
            displacement=math.hypot(self.last_state.px-prev.px,
                                    self.last_state.py-prev.py)
            reach_m=max(abs(prev.v),abs(self.last_state.v))*tick_s+0.02
            if displacement>reach_m:
                self._stop_probe_last_detail={
                    'branch':'PHYSICS_TICK_DISPLACEMENT_UNREACHABLE',
                    'distance_m':float(displacement),
                    'physical_tick_s':float(tick_s),
                    'stamp_gap_s':float(delta_ns*1e-9),
                    'reach_m':float(reach_m),
                }
                return False
            if (abs(self.last_state.v-prev.v)>0.15 or
                    abs(math.remainder(self.last_state.yaw-prev.yaw,2*math.pi))>0.15):
                return False
        if self.collision:return False
        try:
            started=time.perf_counter_ns()
            cpu_started=time.thread_time_ns()
            # Linux per-thread rusage: diagnostic only; never affects proof.
            import resource
            _ctx_start=resource.getrusage(resource.RUSAGE_THREAD)
            z=self._state()
            z1=__import__('dataclasses').replace(z)
            self.envelope.dynamics.step(z1,msg.steering_rad,msg.acceleration_mps2,0.025)
            # Check immediately commanded motion followed by reaction/stop.
            # H2H uses the SAME Ego braking rollout, with a separate fresh
            # Red dynamic obstacle proof on every simulated physics step.
            # Missing, stale, unconfirmed or short-horizon predictions reject.
            dynamic_checker=None
            if not self.solo:
                if (self.red_prediction is None or
                        self.red_prediction_received_steady_ns is None or
                        time.monotonic_ns()-self.red_prediction_received_steady_ns>150_000_000):
                    self._stop_probe_last_detail={'branch':'H2H_RED_PREDICTION_MISSING_OR_STALE'}
                    return False
                try:
                    witness=RedDynamicWitness(self.red_prediction,
                        # Prediction is aligned at the current command decision;
                        # check the IMMEDIATE command tick before the stopping
                        # rollout, then offset the remaining rollout by 25ms.
                        now_s=self.get_clock().now().nanoseconds*1e-9,
                        track_id=self.identity[2], max_age_s=0.3,
                        # Necessary stop-time guess only: the actual full
                        # stopping rollout below continues until standstill.
                        # A prediction ending earlier than the REAL stop is
                        # rejected by collision_free at the first uncovered tick.
                        minimum_horizon_s=min(self.envelope.max_time,
                            self.envelope.reaction+abs(z1.speed)/self.envelope.brake+1.0),
                        ego_radius_m=self.ego_collision_radius_m,
                        red_acceleration_uncertainty_mps2=max(
                            float(self.environment['dynamics']['limits']['acceleration_max_m_s2']),
                            float(self.environment['dynamics']['limits']['deceleration_max_m_s2'])))
                    # Prevent tunneling during the commanded first tick. The
                    # ordinary envelope starts at z1 and does not cover z->z1.
                    if not witness.collision_free(z,z1,0.0,0.025):
                        self._stop_probe_last_detail={'branch':'H2H_RED_IMMEDIATE_SWEEP_REJECTED'}
                        return False
                    def dynamic_checker(before,after,begin_s,end_s):
                        return witness.collision_free(before,after,
                            begin_s+0.025,end_s+0.025)
                except (ValueError,OverflowError,TypeError,AttributeError) as exc:
                    self._stop_probe_last_detail={'branch':'H2H_RED_WITNESS_UNPROVEN',
                                                  'type':type(exc).__name__}
                    return False
            # Production StoppingEnvelope supports incremental fail-closed
            # budget checks; replay probes may only provide evaluate().
            if hasattr(self.envelope,'evaluate_bounded'):
                result=self.envelope.evaluate_bounded(
                    z1,msg.steering_rad,msg.acceleration_mps2,
                    started+15_000_000,**({'dynamic_checker':dynamic_checker}
                                          if dynamic_checker else {}))
            else:
                if dynamic_checker is not None:
                    return False  # no legacy unbounded/non-dynamic fallback in H2H
                result=self.envelope.evaluate(
                    z1,msg.steering_rad,msg.acceleration_mps2)
            if result.qualified and self.native_dual_proof is not None:
                # H2H Red occupancy remains checked by the original independent
                # Python dynamic checker. Native additionally vetoes against
                # its own C++ dynamics, full-stop rollout and occupancy map.
                # Both must approve the SAME current-state candidate within
                # the ORIGINAL 15ms deadline; native never bypasses Red.
                native=self.native_dual_proof.evaluate(
                    z1,msg.steering_rad,msg.acceleration_mps2,
                    deadline_ns=started+15_000_000)
                native_agrees=(
                    bool(native.qualified) and native.reason==result.reason and
                    native.steps==result.steps and
                    all(math.isclose(float(a),float(b),rel_tol=1e-6,abs_tol=1e-5)
                        for a,b in ((native.stopping_time_s,result.stopping_time_s),
                                    (native.stopping_distance_m,result.stopping_distance_m),
                                    (native.max_abs_slip_rad,result.max_abs_slip_rad),
                                    (native.max_lateral_accel_mps2,result.max_lateral_accel_mps2))))
                if not native_agrees:
                    self._stop_probe_last_detail={
                        'branch':'NATIVE_DUAL_STOP_VETO_MISMATCH',
                        'native_reason':native.reason,
                        'python_reason':result.reason,
                        'native_qualified':bool(native.qualified),
                    }
                    return False
            if not result.qualified:
                self._stop_probe_last_detail={
                    'branch':'STOPPING_ENVELOPE_REJECT',
                    'envelope_reason':str(result.reason),
                    'stop_time_s':float(result.stopping_time_s),
                    'stop_distance_m':float(result.stopping_distance_m),
                    'max_abs_slip_rad':float(result.max_abs_slip_rad),
                    'max_lateral_accel_mps2':float(result.max_lateral_accel_mps2),
                    'steps':int(result.steps),
                }
            elapsed_ns=time.perf_counter_ns()-started
            cpu_elapsed_ns=time.thread_time_ns()-cpu_started
            _ctx_end=resource.getrusage(resource.RUSAGE_THREAD)
            _voluntary=max(0,_ctx_end.ru_nvcsw-_ctx_start.ru_nvcsw)
            _involuntary=max(0,_ctx_end.ru_nivcsw-_ctx_start.ru_nivcsw)
            elapsed_us=elapsed_ns/1000.0
            self.stop_proof_max_duration_us=max(self.stop_proof_max_duration_us,elapsed_us)
            if elapsed_ns>15000000:
                self.stop_proof_slow_count+=1
                self.last_reason='STOP_CERTIFICATE_DEADLINE_EXCEEDED'
                self._stop_probe_last_detail={
                    'branch':'GUARD_15MS_DEADLINE',
                    'elapsed_us':elapsed_us,
                    'cpu_elapsed_us':cpu_elapsed_ns/1000.0,
                    'scheduler_gap_us':max(0.0,(elapsed_ns-cpu_elapsed_ns)/1000.0),
                    'slow_count':self.stop_proof_slow_count,
                    'voluntary_context_switches':int(_voluntary),
                    'involuntary_context_switches':int(_involuntary),
                    'scheduler_attribution':'OBSERVED_COUNTERS_NOT_CAUSAL_PROOF',
                }
                return False
            # A scheduler pause between accounting and the return may not
            # turn an expired proof into an authorized command.
            if time.perf_counter_ns()-started > 15_000_000:
                self.stop_proof_slow_count+=1
                self._stop_probe_last_detail={
                    'branch':'GUARD_15MS_RESPONSE_DEADLINE',
                    'elapsed_us':(time.perf_counter_ns()-started)/1000.0,
                }
                return False
            return bool(result.qualified)
        except (ArithmeticError,ValueError,OverflowError,TypeError,KeyError,RuntimeError,OSError) as exc:
            self._stop_probe_last_detail={'branch':'GUARD_CERT_EXCEPTION', 'type':type(exc).__name__}
            return False

    def _guard_stop_probe_snapshot(self,msg,now,reason):
        """Offline diagnostic input snapshot, constructed AFTER braking."""
        names=('physics_sequence','px','py','yaw','v','vx','vy',
               'omega','a','ax','ay','slip_angle','accel','jerk',
               'steer','steer_vel')
        def state_view(s):
            if s is None:return None
            value={k:getattr(s,k,None) for k in names}
            value['physics_stamp_ns']=ns(s.physics_stamp)
            return value
        lim=self.environment['dynamics']['limits']
        return {
            'schema':'F1TENTH_GUARD_CERT_FORENSIC_V1',
            'source_sha_original':'0e2525fc6aad02be61e94ac2cebeb2a08dae3542e5f59eb0d68480a5d4180fc2',
            'reason':reason,
            'now_ros_ns':now,
            'last_state_sequence':self.last_state_sequence,
            'previous_state':state_view(self.previous_state),
            'last_state':state_view(self.last_state),
            'collision_latched':bool(self.collision),
            'reference_revision':self.reference_revision,
            'max_speed_cap_mps':float(self.get_parameter('max_speed_cap_mps').value),
            'identity':{'epoch':self.identity[0], 'producer':self.identity[1],
                        'track':self.identity[2], 'map_sha256':self.identity[3]},
            'intent':{
                'sequence':int(msg.sequence),
                'source_state_sequence':int(msg.source_state_sequence),
                'source_state_stamp_ns':ns(msg.source_state_stamp),
                'created_ros_ns':ns(msg.created_ros_stamp),
                'expires_ros_ns':ns(msg.expires_ros_stamp),
                'source_id':msg.source_id,
                'run_epoch':msg.run_epoch,
                'active_reference_id':msg.active_reference_id,
                'active_revision':int(msg.active_revision),
                'track_id':msg.track_id,
                'map_sha256':msg.map_sha256,
                'candidate_mode':msg.candidate_mode,
                'steering_rad':float(msg.steering_rad),
                'acceleration_mps2':float(msg.acceleration_mps2),
            },
            'dynamics_limits':{k:float(v) for k,v in lim.items()},
            'stop_envelope_result':self._stop_probe_last_detail,
        }

    def _brake(self,reason):
        # This is minimum-risk braking, NOT proof of collision-free stop.
        self.pending_future_intent=None
        self.pending_future_deadline_steady_ns=None
        self.pending_future_received_steady_ns=None
        first_brake_report=None
        if not hasattr(self, 'first_brake_record'):
            self.first_brake_record={
                'reason':str(reason),
                'ros_ns':int(self.get_clock().now().nanoseconds),
                'steady_ns':int(time.monotonic_ns()),
                'state_sequence':int(self.last_state_sequence),
                'accepted_sequence':int(self.last_accepted_sequence),
            }
            first_brake_report=self.first_brake_record
        self.last_reason=reason
        self.last_certificate=False
        # Every failed safety action revokes the current actuator authority.
        # Recovery is possible only via the explicitly qualified parked-state
        # fresh-sample path; never from a subsequent successful Intent alone.
        self.contract.state='SAFE_HOLD'
        # The V2 first-six-intents failure was a STATE WATCHDOG gap after
        # certificates for ZERO acceleration. Its accepted-count check made
        # that sensor outage an irreversible latch. Preserve hard latches for
        # collision, dangerous/rejected intent, moving vehicle, ROS clock
        # faults and any positive actuator command. ONLY a fresh-state-loss
        # while demonstrably parked is eligible for an independent re-arm.
        # This exception does not restore actuation. It permits re-arming
        # only AFTER the simulator has physically parked, multiple new CarState
        # receipts and S0-R's separately gated nominal re-entry.
        planned_h2h_abort=(
            reason == 'H2H_ABORT_REFERENCE_WITHDRAWN'
            and self.h2h_abort_park_recovery and not self.solo
            and not self.collision and not self.emergency_unrecoverable)
        recoverable_gap=planned_h2h_abort or (reason in ('STATE_WATCHDOG_TIMEOUT','INITIAL_INTENT_TIMEOUT') and
                     not self.emergency_unrecoverable and not self.collision and
                     self.last_state is not None and abs(self.last_state.v)<=0.025 and
                     math.hypot(self.last_state.vx,self.last_state.vy)<=0.025 and
                     abs(self.last_state.omega)<=0.05 and
                     abs(self.last_state.a)<=0.25 and
                     float(self.last_state.accel)<=0.05 and
                     self.last_accepted_acceleration_mps2<=0.0)
        if recoverable_gap:
            if not self.startup_waiting_for_fresh_state:
                self.startup_recovery_fresh_samples=0
                self.last_recovery_physics_sequence=self.last_state_sequence
                # Outage duration is allowed to be long; the requalification
                # sequence starts from the FIRST newly fresh state, not from
                # the stale state that caused the watchdog failure.
                self.last_recovery_ros_stamp_ns=-1
                self.last_recovery_steady_ns=-1
                self.recovery_first_sample_steady_ns=None
            self.startup_waiting_for_fresh_state=True
            self.recovery_samples_required=(4 if self.last_accepted_sequence>0 else 2)
            # These are not eligible to count as healthy recently validated
            # heartbeat certificates: only NEW incoming intents can do so.
            self.last_intent_steady_ns=None
        else:
            self.emergency_unrecoverable=True
            self.startup_waiting_for_fresh_state=False
        # The fail-closed latch is already set. Command the physical brake
        # BEFORE any synchronous ROS logging. For absent/stale state preserve
        # the original behaviour: no command lease refresh; simulator deadman.
        if (self.last_state is not None and
                self.state_received_steady_ns is not None and
                time.monotonic_ns()-self.state_received_steady_ns<=100000000 and
                finite_physics_state(self.last_state)):
            # Poisoned state cannot even authorize a new braking *lease*.
            # The simulator's independent deadman owns stop on sensor loss.
            cmd=AckermannDriveStamped()
            cmd.header.stamp=self.get_clock().now().to_msg()
            cmd.drive.steering_angle=0.0
            speed=self.last_state.v
            cmd.drive.acceleration=(-2.0 if speed>0.02 else
                                     2.0 if speed< -0.02 else 0.0)
            if hasattr(self, "atomic_pub"):
                guarded=GuardedActuatorCommand()
                guarded.stamp=cmd.header.stamp
                until=ns(cmd.header.stamp)+100_000_000
                guarded.expires_ros_stamp=Time(sec=until//1_000_000_000,
                                               nanosec=until%1_000_000_000)
                guarded.run_epoch=self.identity[0]
                guarded.source_id=self.identity[1]
                guarded.intent_sequence=0
                guarded.source_state_sequence=max(0,int(self.last_state_sequence))
                guarded.active_reference_id=self.reference_id
                guarded.active_revision=int(self.reference_revision)
                guarded.track_id=self.identity[2]
                guarded.map_sha256=self.identity[3]
                guarded.authority_mode='MINIMUM_RISK_BRAKE'
                guarded.stop_certificate_verified=False
                guarded.steering_rad=0.0
                guarded.acceleration_mps2=cmd.drive.acceleration
                self.atomic_pub.publish(guarded)
            self.final_pub.publish(cmd)
        if first_brake_report is not None:
            # Evidence reporting cannot precede the safety action.
            self.get_logger().error('F1TENTH_GUARD_FIRST_BRAKE='+json.dumps(
                first_brake_report,sort_keys=True))

    def on_intent(self,msg):
        now=self.get_clock().now().nanoseconds
        if (self.emergency_unrecoverable or self.collision or
                self.start_ns is None or now<self.start_ns or
                self.contract.state not in ('ARMED','RUNNING')):
            self.last_reason='NOT_RUNNING_OR_ARMED'
            return
        if self.pending_future_intent is not None:
            # A second Intent before the awaited state cannot silently
            # replace/reorder the first pending command. Fail closed.
            self.pending_future_intent=None
            self.pending_future_deadline_steady_ns=None
            self.pending_future_received_steady_ns=None
            self._brake('FUTURE_STATE_PENDING_OVERTAKEN_BY_INTENT')
            return
        # Only the immediately NEXT tick is eligible for the bounded wait.
        # Two or more ticks ahead, contradictory timestamps, or stale
        # identity go through the ORIGINAL certificate and fail closed.
        source_seq=int(msg.source_state_sequence)
        if self.last_state is not None and source_seq==self.last_state_sequence+1:
            state_ns=ns(self.last_state.physics_stamp)
            future_ns=ns(msg.source_state_stamp)
            created_ns=ns(msg.created_ros_stamp)
            expiry_ns=ns(msg.expires_ros_stamp)
            now_steady=time.monotonic_ns()
            # Exactly-one sequence, ROS/staleness/lease and identity binding
            # constrain the pending Intent. Physical *stamp deltas* may vary
            # with the simulator scheduler and are not a permission oracle.
            # Never publish while pending; require exact state stamp on receipt.
            if (0<future_ns-state_ns and
                    0<=now-state_ns<=100_000_000 and
                    future_ns<=now and
                    0<=now-created_ns<=100_000_000 and
                    created_ns<=expiry_ns and now<=expiry_ns and
                    self.state_received_steady_ns is not None and
                    0<=now_steady-self.state_received_steady_ns<=100_000_000 and
                    msg.sequence>self.contract.previous_sequence and
                    msg.source_id==self.identity[1] and
                    msg.run_epoch==self.identity[0] and
                    msg.track_id==self.identity[2] and
                    msg.map_sha256==self.identity[3] and
                    msg.active_reference_id==self.reference_id and
                    msg.active_revision==self.reference_revision and
                    not self.collision and not self.emergency_unrecoverable):
                # NO actuation, NO acceptance, NO certificate at this point.
                self.pending_future_intent=msg
                self.pending_future_received_steady_ns=now_steady
                self.pending_future_deadline_steady_ns=now_steady+40_000_000
                self.last_reason='FUTURE_STATE_PENDING_NO_ACTUATION'
                if self.pending_future_diagnostics_emitted<12:
                    self.pending_future_diagnostics_emitted+=1
                    self.get_logger().warning(
                        'F1TENTH_GUARD_FUTURE_TICK_DEFERRED='+json.dumps({
                            'intent_sequence':int(msg.sequence),
                            'source_state_sequence':source_seq,
                            'guard_state_sequence':int(self.last_state_sequence),
                            'source_minus_guard_ros_ns':int(future_ns-state_ns),
                            'wait_budget_steady_ns':40000000,
                        },sort_keys=True))
                return
        self._validate_and_publish_intent(msg)

    def _validate_and_publish_intent(self,msg):
        now=self.get_clock().now().nanoseconds
        steady=time.monotonic_ns()
        if self.emergency_unrecoverable or self.collision:
            self._brake('HARD_LATCH_BLOCKS_INTENT')
            return
        self._arm_if_ready()
        if self.contract.state not in ('ARMED', 'RUNNING'):
            self._brake('INTENT_NOT_ARMED_AFTER_RECHECK')
            return
        contract_intent=IntentContract(msg.source_id,msg.run_epoch,msg.sequence,
            ns(msg.created_ros_stamp),ns(msg.expires_ros_stamp),
            msg.active_reference_id,msg.track_id,msg.map_sha256,
            ns(msg.source_state_stamp),msg.steering_rad,msg.acceleration_mps2)
        self._stop_probe_last_detail=None
        certificate=self._certificate(msg)
        # TOCTOU protection: the stop proof may consume most of the lease.
        # Validate against the clock AFTER the independent certificate, not
        # the snapshot captured before starting a potentially slow proof.
        # An intent that expires during the proof cannot gain authority.
        validation_ros_ns=self.get_clock().now().nanoseconds
        validation_steady_ns=time.monotonic_ns()
        result=self.contract.validate(contract_intent,ros_time_ns=validation_ros_ns,
            steady_time_ns=validation_steady_ns,
            state_received_steady_ns=(self.state_received_steady_ns or -1),
            validated_stop_certificate=certificate)
        self.last_reason=result.reason
        self.last_certificate=certificate
        if not result.accepted:
            self._brake('GUARD_REJECT:'+result.reason)
            # Safety action is already published. Emit only one bounded trace.
            # No extra certificate evaluation or retry is performed.
            if (result.reason=='STOP_CERTIFICATE_NOT_VERIFIED' and
                    not self._stop_probe_emitted):
                self._stop_probe_emitted=True
                try:
                    trace=self._guard_stop_probe_snapshot(msg,now,result.reason)
                    from .forensic_trace import trace_guard
                    trace_guard('GUARD_STOP_FAILURE_SNAPSHOT', snapshot=trace)
                    self.get_logger().error(
                        'F1TENTH_GUARD_CERT_FAILURE_SNAPSHOT_QUEUED seq='+
                        str(msg.sequence)+' reason='+str(result.reason))
                except Exception as exc:
                    self.get_logger().error('F1TENTH_GUARD_CERT_FORENSIC_ERROR='+
                                            type(exc).__name__)
            return
        cmd=AckermannDriveStamped()
        cmd.header.stamp=self.get_clock().now().to_msg()
        cmd.drive.steering_angle=result.steering_rad
        cmd.drive.acceleration=result.acceleration_mps2
        # Recheck immediately before the sole actuator publication: a
        # preemption while constructing a response must not convert a
        # now-expired certificate/physics lease into authority. This is a
        # fail-closed *time-of-use* gate, not a cached stop proof or a timeout
        # increase. The ROS transport/OS scheduling bound remains unproven.
        publish_ros_ns=self.get_clock().now().nanoseconds
        publish_steady_ns=time.monotonic_ns()
        if (publish_ros_ns < validation_ros_ns or
                publish_ros_ns < ns(msg.created_ros_stamp) or
                publish_ros_ns > ns(msg.expires_ros_stamp) or
                publish_ros_ns < ns(msg.source_state_stamp) or
                publish_ros_ns-ns(msg.source_state_stamp) > self.contract.state_age_limit_ns or
                self.state_received_steady_ns is None or
                not 0 <= publish_steady_ns-self.state_received_steady_ns <= self.contract.state_age_limit_ns):
            self._brake('POST_CERTIFICATE_LEASE_EXPIRED')
            return
        # Atomic authority and command identity: the simulator acts ONLY on
        # this one typed transport. ackermann_cmd0 remains telemetry-only.
        guarded=GuardedActuatorCommand()
        guarded.stamp=cmd.header.stamp
        guarded.expires_ros_stamp=msg.expires_ros_stamp
        guarded.run_epoch=self.identity[0]
        guarded.source_id=str(msg.source_id)
        guarded.intent_sequence=int(msg.sequence)
        guarded.source_state_sequence=int(msg.source_state_sequence)
        guarded.active_reference_id=str(msg.active_reference_id)
        guarded.active_revision=int(msg.active_revision)
        guarded.track_id=self.identity[2]
        guarded.map_sha256=self.identity[3]
        guarded.authority_mode='CERTIFIED'
        guarded.stop_certificate_verified=True
        guarded.steering_rad=float(result.steering_rad)
        guarded.acceleration_mps2=float(result.acceleration_mps2)
        self.atomic_pub.publish(guarded)
        self.final_pub.publish(cmd)
        # Diagnostic correlation only; never treats this separately delivered
        # metadata as actuator authorization. Simulator may report null when
        # receipt races the final command, rather than invent an identity.
        self.intent_receipt_pub.publish(String(data=json.dumps({
            'run_epoch':self.identity[0],
            'intent_sequence':int(msg.sequence),
            'source_state_sequence':int(msg.source_state_sequence),
            'source_id':str(msg.source_id),
            'active_reference_id':str(msg.active_reference_id),
            'active_revision':int(msg.active_revision),
            'command_stamp_ns':ns(cmd.header.stamp),
        },sort_keys=True)))
        self.last_accepted_sequence=msg.sequence
        self.last_accepted_acceleration_mps2=float(result.acceleration_mps2)
        self.last_intent_steady_ns=steady
        # A successful acceptance does not clear a hard fault. Hard faults
        # block validation above; only explicit restart can remove the latch.

    def watchdog(self):
        now=self.get_clock().now().nanoseconds
        steady=time.monotonic_ns()
        if self.pending_future_intent is not None:
            if (steady>self.pending_future_deadline_steady_ns or
                    now>ns(self.pending_future_intent.expires_ros_stamp)):
                self.pending_future_intent=None
                self.pending_future_deadline_steady_ns=None
                self.pending_future_received_steady_ns=None
                self._brake('FUTURE_STATE_WAIT_DEADLINE_EXCEEDED')
        self._arm_if_ready()
        live=(self.last_state is not None and
              self.state_received_steady_ns is not None and
              steady-self.state_received_steady_ns<=100000000 and
              now>=ns(self.last_state.physics_stamp) and
              now-ns(self.last_state.physics_stamp)<=100000000)
        if self.last_ros_ns is not None and now<=self.last_ros_ns:
            self.contract.state='SAFE_HOLD'
            self.emergency_unrecoverable=True
            self.last_certificate=False
            self.last_reason='ROS_CLOCK_STALL_OR_REGRESSION'
            self.pending_future_intent=None
            self.pending_future_deadline_steady_ns=None
            self.pending_future_received_steady_ns=None
            # Do not refresh simulator's command lease on frozen ROS time.
        elif not live:
            if self.start_ns is None or now<self.start_ns:
                # Report the missing prerequisite in causal order; do not
                # mislabel a rejected S0-R reference as a physics outage.
                if not self.reference_id and not self.emergency_unrecoverable:
                    self.last_reason='PRESTART_REFERENCE_INVALID'
                elif not self.emergency_unrecoverable:
                    self.last_reason='PRESTART_WAITING_FOR_FRESH_STATE'
            else:
                watchdog_report=None
                if not self.watchdog_gap_diagnostic_emitted:
                    self.watchdog_gap_diagnostic_emitted=True
                    watchdog_report={
                        'now_ros_ns':int(now),
                        'now_steady_ns':int(steady),
                        'last_state_sequence':int(self.last_state_sequence),
                        'last_state_stamp_ros_ns':(ns(self.last_state.physics_stamp) if self.last_state else None),
                        'last_state_receive_steady_ns':self.state_received_steady_ns,
                        'last_accepted_sequence':int(self.last_accepted_sequence),
                        'last_accepted_accel_mps2':self.last_accepted_acceleration_mps2,
                        'mode':str(self.contract.state),
                    }
                self.contract.state='SAFE_HOLD'
                self._brake('STATE_WATCHDOG_TIMEOUT')
                if watchdog_report is not None:
                    self.get_logger().error('F1TENTH_GUARD_FIRST_STATE_WATCHDOG='+json.dumps(
                        watchdog_report,sort_keys=True))
        elif (self.h2h_abort_park_recovery and not self.reference_id and
              self.contract.state == 'SAFE_HOLD' and
              not self.emergency_unrecoverable):
            # No reference = no actuation. Leave the intentional ABORT parked
            # recovery opportunity open; the S0-R + Red witness gate decides
            # when a new nominal reference may be published.
            self.last_reason='H2H_ABORT_WAITING_FOR_PARKED_NOMINAL_REENTRY'
        elif (self.start_ns is not None and now>=self.start_ns and
              (self.last_intent_steady_ns is None or steady-self.last_intent_steady_ns>100000000)):
            # The controller computes its first MPC only after scheduled start.
            # A missing FIRST intent cannot be treated as a missed 100ms
            # RUNNING heartbeat on the first 25ms watchdog tick.
            first_intent = self.last_intent_steady_ns is None
            stationary = (self.last_state is not None and
                          abs(self.last_state.v)<=0.025 and
                          math.hypot(self.last_state.vx,self.last_state.vy)<=0.025)
            first_wait_safe = (first_intent and self.contract.state=='ARMED' and
                               not self.collision and not self.emergency_unrecoverable and
                               stationary)
            if (first_intent and self.startup_waiting_for_fresh_state and
                    self.contract.state=='SAFE_HOLD' and stationary and
                    not self.collision and not self.emergency_unrecoverable):
                # First fresh state alone is not enough to re-arm. Wait for
                # the SECOND increasing physics state; do not turn this
                # bounded recovery wait into an irreversible intent latch.
                self.last_reason='STARTUP_RECOVERY_WAITING_FOR_FRESH_STATE'
            elif first_wait_safe:
                if self.first_intent_wait_started_steady_ns is None:
                    self.first_intent_wait_started_steady_ns=steady
                if steady-self.first_intent_wait_started_steady_ns<500000000:
                    # Absolutely no Ackermann output while awaiting the first
                    # independently certified command: simulator deadman rules.
                    self.last_reason='WAITING_FOR_FIRST_INTENT_STATIONARY'
                else:
                    self.contract.state='SAFE_HOLD'
                    self._brake('INITIAL_INTENT_TIMEOUT')
            else:
                # Existing post-first-intent 100ms watchdog, or a moving,
                # stale, collided, or previously rejected startup state.
                self.contract.state='SAFE_HOLD'
                self._brake('INTENT_WATCHDOG_TIMEOUT')
        self.last_ros_ns=now
        status=ActuatorGuardStatus()
        status.stamp=self.get_clock().now().to_msg()
        status.run_epoch=self.identity[0]
        status.mode=self.contract.state
        # Readiness and reported reason belong to the same watchdog snapshot.
        # Transient prestart sensor gaps must not leave an obsolete WAITING_FOR_STATE
        # reason attached to an ARMED + live=True status.
        if (self.contract.state == 'ARMED' and live and self.start_ns is None and
                self.last_reason == 'PRESTART_WAITING_FOR_FRESH_STATE'):
            self.last_reason='WAITING_FOR_VALID_INTENT_AND_CERTIFICATE'
        status.reason=self.last_reason
        status.source_id=self.identity[1]
        status.active_reference_id=self.reference_id
        status.last_accepted_sequence=self.last_accepted_sequence
        status.ready=(self.contract.state in ('ARMED','RUNNING') and live and
                      not self.collision and not self.emergency_unrecoverable)
        status.certificate_verified=self.last_certificate
        status.emergency_unrecoverable=self.emergency_unrecoverable
        self.status_pub.publish(status)


# === FORENSIC CALLBACK WRAPPERS: diagnostics only; no safety/control branch changes ===
from f1tenth_racing_control.forensic_trace import trace_guard as _trace

_guard_original_state=IndependentActuatorGuard.on_state

def _guard_observed_state(self,msg):
    import time
    t=time.monotonic_ns()
    _trace('GUARD_STATE_ENTER',physics_sequence=int(msg.physics_sequence),
           physics_stamp_ns=int(msg.physics_stamp.sec)*1000000000+int(msg.physics_stamp.nanosec),
           last_sequence=int(getattr(self,'last_state_sequence',-1)),
           contract_state=str(self.contract.state))
    try:return _guard_original_state(self,msg)
    finally:_trace('GUARD_STATE_EXIT',physics_sequence=int(msg.physics_sequence),
                   elapsed_ns=time.monotonic_ns()-t)
IndependentActuatorGuard.on_state=_guard_observed_state

_guard_original_intent=IndependentActuatorGuard.on_intent
def _guard_observed_intent(self,msg):
    _trace('GUARD_INTENT_ENTER', intent_sequence=int(msg.sequence),
           source_sequence=int(msg.source_state_sequence),
           source_stamp_ns=int(msg.source_state_stamp.sec)*1000000000+int(msg.source_state_stamp.nanosec),
           contract_state=str(self.contract.state), latch=bool(self.emergency_unrecoverable),
           local_state_sequence=int(getattr(self,'last_state_sequence',-1)))
    return _guard_original_intent(self,msg)
IndependentActuatorGuard.on_intent=_guard_observed_intent

_guard_original_brake=IndependentActuatorGuard._brake
def _guard_observed_brake(self,reason):
    _trace('GUARD_BRAKE', reason=str(reason), prior_reason=str(self.last_reason),
           contract_state=str(self.contract.state), latch=bool(self.emergency_unrecoverable),
           accepted_sequence=int(self.last_accepted_sequence))
    return _guard_original_brake(self,reason)
IndependentActuatorGuard._brake=_guard_observed_brake

_guard_original_validate=IndependentActuatorGuard._validate_and_publish_intent
def _guard_observed_validate(self,msg):
    _trace('GUARD_VALIDATE_ENTER',intent_sequence=int(msg.sequence),
           source_sequence=int(msg.source_state_sequence),
           local_state_sequence=int(getattr(self,'last_state_sequence',-1)),
           contract_state=str(self.contract.state))
    return _guard_original_validate(self,msg)
IndependentActuatorGuard._validate_and_publish_intent=_guard_observed_validate

_guard_original_watchdog=IndependentActuatorGuard.watchdog
def _guard_observed_watchdog(self):
    _trace('GUARD_WATCHDOG_ENTER', contract_state=str(self.contract.state),
           reason=str(self.last_reason), latch=bool(self.emergency_unrecoverable),
           accepted_sequence=int(self.last_accepted_sequence))
    return _guard_original_watchdog(self)
IndependentActuatorGuard.watchdog=_guard_observed_watchdog

def main():
    rclpy.init()
    node=IndependentActuatorGuard()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
