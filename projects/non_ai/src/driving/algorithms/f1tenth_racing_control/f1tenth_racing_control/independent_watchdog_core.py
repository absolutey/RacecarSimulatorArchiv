"""Independent Guard observer. No authority to issue actuator commands.

A separate ROS process checks latest Guard status AND physics state age,
collision latch, epoch/provenance and local monotonic time. Any missing input
is a REVOKE witness; ALLOW only removes this *additional veto*.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StateWitness:
    sequence: int
    ros_stamp_ns: int
    received_steady_ns: int


class IndependentWatchdogCore:
    def __init__(self, *, guard_lease, max_state_age_ns=100_000_000):
        if not 0<int(max_state_age_ns)<=100_000_000:
            raise ValueError('INVALID_WATCHDOG_STATE_MAX_AGE')
        self.guard_lease=guard_lease
        self.max_state_age_ns=int(max_state_age_ns)
        self.state=None
        self.collision_latched=False
        self.last_ros_tick_ns=0

    def receive_state(self, *, sequence, stamp_ns, now_ros_ns, now_steady_ns):
        try:
            seq=int(sequence); stamp=int(stamp_ns)
            if (seq<=0 or stamp<=0 or stamp>int(now_ros_ns) or
                    int(now_ros_ns)-stamp>self.max_state_age_ns or
                    int(now_steady_ns)<0 or
                    (self.state is not None and
                     (seq<=self.state.sequence or stamp<=self.state.ros_stamp_ns))):
                self.state=None
                return False
            self.state=StateWitness(seq,stamp,int(now_steady_ns))
            return True
        except (TypeError,ValueError,OverflowError):
            self.state=None
            return False

    def on_collision(self, collision):
        if bool(collision):
            self.collision_latched=True
            self.guard_lease.latest=None

    def decide(self, *, now_ros_ns, now_steady_ns):
        now_ros_ns=int(now_ros_ns); now_steady_ns=int(now_steady_ns)
        if self.collision_latched:
            return False,'COLLISION_LATCHED'
        if now_ros_ns<=self.last_ros_tick_ns:
            return False,'ROS_CLOCK_STALLED_OR_REGRESSED'
        self.last_ros_tick_ns=now_ros_ns
        state=self.state
        if state is None:
            return False,'STATE_MISSING'
        ros_age=now_ros_ns-state.ros_stamp_ns
        steady_age=now_steady_ns-state.received_steady_ns
        if (ros_age<0 or steady_age<0 or ros_age>self.max_state_age_ns or
                steady_age>self.max_state_age_ns):
            return False,'STATE_STALE'
        return self.guard_lease.permits_certified(
            now_ros_ns=now_ros_ns,now_steady_ns=now_steady_ns)
