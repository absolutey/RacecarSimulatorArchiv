"""Explicit, ROS-free R-E CarState admission (never actuation authority)."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StateAdmission:
    fresh_identity: bool
    admissible: bool
    reason: str
    source_stamp_ns: int
    source_sequence: int


def admit_ego_state(message, *, now_ns, last_stamp_ns=-1, last_sequence=-1,
                    maximum_age_ns=75_000_000):
    """Monotonic source + bounded ROS age + finite kinematics.

    'fresh_identity' means a newer source must replace cached input even if
    poisoned: an invalid newest source cannot leave an older safe-looking
    state available for a new plan. Reordered input cannot overwrite newer.
    """
    seq=int(message.physics_sequence)
    stamp=int(message.physics_stamp.sec)*1_000_000_000+int(message.physics_stamp.nanosec)
    result=lambda fresh,allowed,why: StateAdmission(fresh,allowed,why,stamp,seq)
    if seq<=last_sequence or stamp<=last_stamp_ns:
        return result(False,False,'H2H_EGO_STATE_REPLAY_OR_REORDER')
    age=int(now_ns)-stamp
    if stamp<=0 or age<0 or age>maximum_age_ns:
        return result(True,False,'H2H_EGO_STATE_STALE_OR_FUTURE')
    values=(message.px,message.py,message.yaw,message.v,message.vx,
            message.vy,message.omega,message.a,message.accel,
            message.slip_angle,message.steer)
    if not all(math.isfinite(float(value)) for value in values):
        return result(True,False,'H2H_EGO_STATE_NONFINITE')
    return result(True,True,'H2H_EGO_STATE_ADMITTED')
