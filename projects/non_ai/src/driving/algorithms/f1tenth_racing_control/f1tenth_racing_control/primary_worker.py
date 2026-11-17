"""Bounded, one-flight, independently scheduled MPC solver.

A slow primary cannot stall fallback proof in the controller thread. No result
may be consumed after the tick's hard monotonic request deadline, including
results completed concurrently with timeout. Python threads are not realtime
and do NOT confer actuator approval; the independent Guard still certifies.
"""
from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class PrimaryWorkerCounters:
    submitted: int
    completed: int
    busy_rejected: int
    late_rejected: int
    exceptions: int
    outstanding_sequence: int
    last_completed_sequence: int
    last_compute_ns: int
    max_compute_ns: int


class MpcPrimaryWorker:
    """Exactly one in-flight job, no queueing or replay of stale work."""
    def __init__(self, name='r_b_mpc_primary'):
        self._condition = threading.Condition()
        self._job = None
        self._running = False
        self._closing = False
        self._result = None
        self._submitted = self._completed = self._busy_rejected = 0
        self._late_rejected = self._exceptions = 0
        self._last_sequence = -1
        self._last_completed = -1
        self._active_sequence = -1
        self._last_compute_ns = self._max_compute_ns = 0
        self._thread = threading.Thread(target=self._loop, name=name, daemon=True)
        self._thread.start()

    def submit(self, sequence, calculation, deadline_ns):
        seq = int(sequence)
        deadline = int(deadline_ns)
        with self._condition:
            if (self._closing or not callable(calculation) or
                    seq <= self._last_sequence or self._job is not None or
                    self._running or time.perf_counter_ns() >= deadline):
                self._busy_rejected += 1
                return False
            self._result = None
            self._last_sequence = seq
            self._job = (seq, calculation, deadline)
            self._submitted += 1
            self._condition.notify_all()
            return True

    def _loop(self):
        while True:
            with self._condition:
                while self._job is None and not self._closing:
                    self._condition.wait()
                if self._closing:
                    self._job = None
                    return
                sequence, calculation, deadline_ns = self._job
                self._job = None
                self._running = True
                self._active_sequence = sequence
            started = time.perf_counter_ns()
            try:
                if started >= deadline_ns:
                    raise RuntimeError('MPC_WORKER_DEADLINE_EXPIRED')
                result = (calculation(), None)
            except Exception as error:
                result = (None, error)
            finished = time.perf_counter_ns()
            with self._condition:
                self._running = False
                self._active_sequence = -1
                self._completed += 1
                self._last_completed = sequence
                self._last_compute_ns = max(0, finished-started)
                self._max_compute_ns = max(self._max_compute_ns, self._last_compute_ns)
                if result[1] is not None:
                    self._exceptions += 1
                # Do not even retain tardy outcomes as a possible next-tick input.
                self._result = (sequence, result, finished, deadline_ns) if finished <= deadline_ns else None
                self._condition.notify_all()

    def take(self, sequence, *, deadline_ns):
        seq = int(sequence)
        deadline = int(deadline_ns)
        with self._condition:
            while True:
                now = time.perf_counter_ns()
                if self._closing:
                    raise RuntimeError('MPC_WORKER_CLOSED')
                if now >= deadline:
                    self._late_rejected += 1
                    raise RuntimeError('MPC_WORKER_RESULT_LATE')
                if self._result is not None and self._result[0] == seq:
                    _, (value, error), finished, original_deadline = self._result
                    self._result = None
                    if finished > deadline or now >= original_deadline:
                        self._late_rejected += 1
                        raise RuntimeError('MPC_WORKER_RESULT_LATE')
                    if error is not None:
                        raise error
                    return value
                if self._last_completed >= seq:
                    self._late_rejected += 1
                    raise RuntimeError('MPC_WORKER_RESULT_MISSING_OR_EXPIRED')
                if (self._last_sequence < seq or
                        self._last_sequence > seq):
                    raise RuntimeError('MPC_WORKER_TICK_IDENTITY_UNPROVEN')
                self._condition.wait(timeout=min((deadline-now)*1e-9, 0.002))

    def counters(self):
        with self._condition:
            return PrimaryWorkerCounters(
                self._submitted,self._completed,self._busy_rejected,
                self._late_rejected,self._exceptions,self._active_sequence,
                self._last_completed,self._last_compute_ns,self._max_compute_ns)

    def close(self, timeout_s=2.0):
        with self._condition:
            self._closing = True
            self._job = None
            self._result = None
            self._condition.notify_all()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=max(0.,float(timeout_s)))
        return not self._thread.is_alive()
