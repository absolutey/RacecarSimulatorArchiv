"""Cross-process independent-watchdog VETO lease (never actuation authority).

ROS-free, run-epoch scoped, monotonic local freshness. Both this witness and
GuardStatusLease must permit a CERTIFIED command; neither issues a certificate.
"""
from dataclasses import dataclass
import json


@dataclass(frozen=True)
class ExternalWatchdogVerdict:
    epoch: str
    producer: str
    sequence: int
    emitted_ros_ns: int
    received_steady_ns: int
    physics_sequence: int
    guard_accepted_sequence: int
    verdict: str
    reason: str


class ExternalWatchdogLease:
    SCHEMA = 1

    def __init__(self, *, run_epoch, producer, max_age_ns=100_000_000):
        if not run_epoch or not producer or not 0 < int(max_age_ns) <= 100_000_000:
            raise ValueError('INVALID_EXTERNAL_WATCHDOG_LEASE')
        self.run_epoch = str(run_epoch)
        self.producer = str(producer)
        self.max_age_ns = int(max_age_ns)
        self.latest = None
        self.last_sequence = 0
        self.last_emitted_ros_ns = 0
        self.invalid_updates = 0

    def revoke(self):
        self.latest = None

    def observe(self, payload, *, now_ros_ns, now_steady_ns):
        """Fail closed on malformed or non-monotonic witnesses, even if older was ALLOW."""
        try:
            m = json.loads(payload) if isinstance(payload, str) else payload
            if not isinstance(m, dict) or type(m.get('schema')) is not int or m['schema'] != self.SCHEMA:
                raise ValueError('schema')
            # Reject bool masquerading as an integer, or floating-point casts.
            for key in ('sequence','emitted_ros_ns','physics_sequence','guard_accepted_sequence'):
                if type(m.get(key)) is not int:
                    raise ValueError(key)
            seq=m['sequence']; stamp=m['emitted_ros_ns']
            now_ros_ns=int(now_ros_ns); now_steady_ns=int(now_steady_ns)
            if (m.get('run_epoch') != self.run_epoch or m.get('producer') != self.producer or
                    seq<=self.last_sequence or stamp<=self.last_emitted_ros_ns or
                    stamp<=0 or stamp>now_ros_ns or now_ros_ns-stamp>self.max_age_ns or
                    now_steady_ns<0 or m['physics_sequence']<0 or
                    m['guard_accepted_sequence']<0 or
                    m.get('verdict') not in ('ALLOW','REVOKE') or
                    not isinstance(m.get('reason'), str) or len(m['reason'])>128):
                raise ValueError('invalid-witness')
            self.last_sequence=seq
            self.last_emitted_ros_ns=stamp
            self.latest=ExternalWatchdogVerdict(
                epoch=self.run_epoch,producer=self.producer,sequence=seq,
                emitted_ros_ns=stamp,received_steady_ns=now_steady_ns,
                physics_sequence=m['physics_sequence'],
                guard_accepted_sequence=m['guard_accepted_sequence'],
                verdict=m['verdict'],reason=m['reason'])
            return True
        except (ValueError,TypeError,KeyError,OverflowError,AttributeError):
            self.invalid_updates+=1
            self.revoke()
            return False

    def permits_certified(self, *, now_ros_ns, now_steady_ns, physics_sequence):
        h=self.latest
        if h is None:
            return False,'INDEPENDENT_WATCHDOG_MISSING'
        try:
            ros_age=int(now_ros_ns)-h.emitted_ros_ns
            steady_age=int(now_steady_ns)-h.received_steady_ns
            if (ros_age<0 or steady_age<0 or ros_age>self.max_age_ns or
                    steady_age>self.max_age_ns):
                return False,'INDEPENDENT_WATCHDOG_STALE'
            if h.verdict != 'ALLOW':
                return False,'INDEPENDENT_WATCHDOG_REVOKED:' + h.reason
            # The external observer cannot claim to have seen state ahead of
            # simulator's own physics clock; permit next-tick concurrent publish.
            if h.physics_sequence > int(physics_sequence)+1:
                return False,'INDEPENDENT_WATCHDOG_FUTURE_STATE'
            if h.physics_sequence<=0 or h.guard_accepted_sequence<=0:
                return False,'INDEPENDENT_WATCHDOG_NO_PROVENANCE'
            return True,'INDEPENDENT_WATCHDOG_LIVE'
        except (TypeError,ValueError,OverflowError):
            return False,'INDEPENDENT_WATCHDOG_CLOCK_INVALID'
