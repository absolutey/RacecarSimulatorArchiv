"""Independent simulator-side Guard status veto (no ROS imports).

This does NOT replace a stopping certificate or guarantee a scheduling bound.
It ensures a queued CERTIFIED command cannot remain active when Guard status
expires or indicates SAFE_HOLD, with both ROS time and monotonic-clock checks.
The simulator remains a separate process from the Python Guard ROS node.
"""
from dataclasses import dataclass
import math


def _stamp_ns(stamp):
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError('INVALID_ROS_STAMP')
    return sec * 1_000_000_000 + nanosec


@dataclass(frozen=True)
class GuardHeartbeat:
    ros_stamp_ns: int
    steady_received_ns: int
    accepted_sequence: int
    mode: str
    ready: bool
    certificate_verified: bool
    emergency_unrecoverable: bool


class GuardStatusLease:
    """Non-authorizing revocation gate; status never certifies actuation alone."""
    def __init__(self, *, run_epoch, producer, max_age_ns=100_000_000):
        if not run_epoch or not producer or not 0 < int(max_age_ns) <= 100_000_000:
            raise ValueError('INVALID_GUARD_STATUS_LEASE_CONFIG')
        self.run_epoch = str(run_epoch)
        self.producer = str(producer)
        self.max_age_ns = int(max_age_ns)
        self.latest = None
        self.invalid_updates = 0

    def observe(self, message, *, now_ros_ns, now_steady_ns):
        """Store fresh or revoked status; reject regression and provenance errors."""
        try:
            stamp = _stamp_ns(message.stamp)
            now_ros_ns = int(now_ros_ns)
            now_steady_ns = int(now_steady_ns)
            seq = int(message.last_accepted_sequence)
            old = self.latest
            if (message.run_epoch != self.run_epoch or
                    message.source_id != self.producer or
                    not 0 < stamp <= now_ros_ns or
                    now_ros_ns - stamp > self.max_age_ns or
                    now_steady_ns < 0 or seq < 0 or
                    (old is not None and (stamp <= old.ros_stamp_ns or
                     seq < old.accepted_sequence))):
                self.latest = None
                self.invalid_updates += 1
                return False
            self.latest = GuardHeartbeat(
                ros_stamp_ns=stamp, steady_received_ns=now_steady_ns,
                accepted_sequence=seq, mode=str(message.mode),
                ready=bool(message.ready),
                certificate_verified=bool(message.certificate_verified),
                emergency_unrecoverable=bool(message.emergency_unrecoverable))
            return True
        except (ValueError, TypeError, AttributeError, OverflowError):
            self.latest = None
            self.invalid_updates += 1
            return False

    def revoke(self):
        self.latest = None

    def permits_certified(self, *, now_ros_ns, now_steady_ns):
        """A separate, current RUNNING heartbeat is required for CERTIFIED.

        Do not require an exact last_accepted_sequence match: heartbeat and
        actuation are asynchronous ROS transports and will interleave.
        The atomic GuardedActuatorCommand still carries command lineage.
        """
        h = self.latest
        if h is None:
            return False, 'GUARD_HEARTBEAT_MISSING'
        try:
            ros_age=int(now_ros_ns)-h.ros_stamp_ns
            steady_age=int(now_steady_ns)-h.steady_received_ns
            if (ros_age<0 or steady_age<0 or ros_age>self.max_age_ns or
                    steady_age>self.max_age_ns):
                return False, 'GUARD_HEARTBEAT_STALE'
            if (h.mode!='RUNNING' or not h.ready or
                    not h.certificate_verified or h.emergency_unrecoverable or
                    h.accepted_sequence<=0):
                return False, 'GUARD_HEARTBEAT_NOT_AUTHORIZED'
            return True, 'GUARD_HEARTBEAT_LIVE'
        except (TypeError, ValueError, OverflowError):
            return False, 'GUARD_HEARTBEAT_CLOCK_INVALID'
