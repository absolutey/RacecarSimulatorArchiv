"""Offline, fail-closed Actuator Guard decision contract (NO ROS output).

The actual independent ROS final-publisher handoff is NOT implemented by this
module. The caller must enforce exclusive authority and supply a validated
stop certificate; a certificate *ID string* cannot be trusted by itself.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ActuatorIntent:
    source_id: str
    run_epoch: str
    sequence: int
    created_ros_ns: int
    expires_ros_ns: int
    reference_id: str
    track_id: str
    map_sha256: str
    state_ros_ns: int
    steering_rad: float
    acceleration_mps2: float


@dataclass(frozen=True)
class GuardDecision:
    accepted: bool
    mode: str
    reason: str
    steering_rad: float = 0.0
    acceleration_mps2: float = 0.0


class ActuatorGuardContract:
    """Monotonic replay/clock/identity guard; never claims stop safety itself."""
    def __init__(self, state_age_limit_ns=100_000_000, intent_age_limit_ns=100_000_000):
        self.state_age_limit_ns = int(state_age_limit_ns)
        self.intent_age_limit_ns = int(intent_age_limit_ns)
        if min(self.state_age_limit_ns,self.intent_age_limit_ns)<=0:
            raise ValueError('INVALID_LEASE_BUDGET')
        self.state='BOOT'
        self.run_epoch=None
        self.source_id=None
        self.reference_id=None
        self.track_id=None
        self.map_sha256=None
        self.previous_sequence=-1
        self.last_ros_time_ns=None
        self.last_steady_time_ns=None

    def arm(self, *, run_epoch, source_id, reference_id, track_id, map_sha256,
            ros_time_ns, steady_time_ns):
        fields=(run_epoch,source_id,reference_id,track_id,map_sha256)
        if any(not isinstance(v,str) or not v for v in fields):
            return GuardDecision(False,'SAFE_HOLD','INVALID_IDENTITY')
        if not isinstance(ros_time_ns,int) or not isinstance(steady_time_ns,int):
            return GuardDecision(False,'SAFE_HOLD','INVALID_CLOCK')
        if self.state not in ('BOOT','SAFE_HOLD'):
            return GuardDecision(False,'SAFE_HOLD','REARM_REQUIRES_RESET')
        self.state='ARMED'
        self.run_epoch, self.source_id, self.reference_id, self.track_id, self.map_sha256=fields
        self.previous_sequence=-1
        self.last_ros_time_ns=ros_time_ns
        self.last_steady_time_ns=steady_time_ns
        return GuardDecision(False,'ARMED','WAITING_FOR_VALID_INTENT_AND_CERTIFICATE')

    def transfer_reference(self, *, next_reference_id, ros_time_ns, steady_time_ns):
        """Atomic identity rotation; never resets replay sequence or clocks.

        A new reference is a change in intent identity, not an excuse to
        re-arm an unsafe stopped controller, accept replay, or preserve a
        previous dynamics certificate. Caller MUST independently validate
        the reference and future actuator commands against current physics.
        """
        if self.state!='RUNNING':
            return GuardDecision(False,self.state,'TRANSFER_REQUIRES_RUNNING')
        if not isinstance(next_reference_id,str) or not next_reference_id:
            return GuardDecision(False,self.state,'INVALID_NEXT_REFERENCE')
        if not isinstance(ros_time_ns,int) or not isinstance(steady_time_ns,int):
            return GuardDecision(False,self.state,'INVALID_TRANSFER_CLOCK')
        if ros_time_ns<self.last_ros_time_ns or steady_time_ns<self.last_steady_time_ns:
            return GuardDecision(False,self.state,'TRANSFER_CLOCK_REGRESSION')
        if next_reference_id==self.reference_id:
            return GuardDecision(True,self.state,'REFERENCE_UNCHANGED')
        self.reference_id=next_reference_id
        self.last_ros_time_ns=ros_time_ns
        self.last_steady_time_ns=steady_time_ns
        return GuardDecision(True,self.state,'TRANSFER_NEW_INTENT_AND_FRESH_STOP_PROOF_REQUIRED')

    def validate(self,intent,*,ros_time_ns,steady_time_ns,state_received_steady_ns,
                 validated_stop_certificate=False):
        def reject(reason):
            self.state='SAFE_HOLD'
            return GuardDecision(False,'SAFE_HOLD',reason)
        if self.state not in ('ARMED','RUNNING'):
            return reject('NOT_ARMED')
        if (ros_time_ns<self.last_ros_time_ns or
                steady_time_ns<self.last_steady_time_ns):
            return reject('CLOCK_REGRESSION')
        self.last_ros_time_ns=ros_time_ns
        self.last_steady_time_ns=steady_time_ns
        if (intent.source_id!=self.source_id or intent.run_epoch!=self.run_epoch or
                intent.reference_id!=self.reference_id or
                intent.track_id!=self.track_id or intent.map_sha256!=self.map_sha256):
            return reject('IDENTITY_MISMATCH')
        if intent.sequence<=self.previous_sequence:
            return reject('DUPLICATE_OR_OUT_OF_ORDER')
        if (intent.created_ros_ns>ros_time_ns or intent.expires_ros_ns<ros_time_ns or
                intent.created_ros_ns>intent.expires_ros_ns or
                ros_time_ns-intent.created_ros_ns>self.intent_age_limit_ns):
            return reject('STALE_OR_FUTURE_INTENT')
        if (intent.state_ros_ns>ros_time_ns or
                ros_time_ns-intent.state_ros_ns>self.state_age_limit_ns or
                state_received_steady_ns>steady_time_ns or
                steady_time_ns-state_received_steady_ns>self.state_age_limit_ns):
            return reject('STALE_OR_FUTURE_STATE')
        if (not math.isfinite(intent.steering_rad) or
                not math.isfinite(intent.acceleration_mps2)):
            return reject('NONFINITE_ACTUATION')
        # The certificate is validated by the caller using current physical
        # state / map / model / time, not by trusting an arbitrary message ID.
        if not validated_stop_certificate:
            return reject('STOP_CERTIFICATE_NOT_VERIFIED')
        self.previous_sequence=intent.sequence
        self.state='RUNNING'
        return GuardDecision(True,'RUNNING','VALID_INTENT_AND_STOP_CERTIFICATE',
                             intent.steering_rad,intent.acceleration_mps2)
