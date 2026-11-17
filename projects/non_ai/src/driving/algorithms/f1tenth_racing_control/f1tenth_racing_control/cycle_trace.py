"""Pure-python monotonic timing counters. No actuator rights and no ROS I/O.

Use measurement windows, not wallclock timestamps, to test runtime budgets.
A recorder/observer may export these periodically outside control callbacks.
"""
from dataclasses import dataclass
from time import perf_counter_ns


@dataclass(frozen=True)
class TimingReceipt:
    label: str
    elapsed_ns: int
    limit_ns: int

    @property
    def within_budget(self):
        return 0 <= self.elapsed_ns <= self.limit_ns


class StageTimer:
    def __init__(self, label, limit_ns, now=perf_counter_ns):
        if not label or limit_ns <= 0:
            raise ValueError('INVALID_STAGE_TIMER')
        self.label, self.limit_ns, self._now = label, int(limit_ns), now
        self._started = None

    def start(self):
        self._started = self._now()
        return self

    def finish(self):
        if self._started is None:
            raise RuntimeError('STAGE_NOT_STARTED')
        duration = self._now() - self._started
        self._started = None
        return TimingReceipt(self.label, duration, self.limit_ns)
