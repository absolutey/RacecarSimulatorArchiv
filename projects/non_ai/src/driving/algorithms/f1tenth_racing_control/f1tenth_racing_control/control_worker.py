"""Latest-only controller worker, independent from the rclpy callback executor.

This separates heavy MPC/Fallback/GuardAhead computation from ROS message
receipt. Python threading does NOT provide hard realtime scheduling or a hard
7 ms / 15 ms safety proof. Guard remains the only actuator authority.
"""
from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class WorkerCounters:
    submitted: int
    processed: int
    overwritten: int
    rejected: int
    errors: int
    pending_sequence: int
    active_sequence: int
    last_completed_sequence: int
    last_error: str
    last_queue_wait_ns: int
    max_queue_wait_ns: int
    last_cycle_ns: int
    max_cycle_ns: int


class LatestOnlyControlWorker:
    """Consume the newest single physics state on a dedicated daemon thread.

    Monotonicity is enforced both at submit and completion boundaries. Never
    replay a superseded item, and do not invoke callbacks after close starts.
    Callback failures are caught and reported, never treated as safe commands.
    """

    def __init__(self, callback, on_error=None, *, name='r_b_control_worker'):
        self._callback = callback
        self._on_error = on_error
        self._condition = threading.Condition()
        self._pending = None
        self._pending_sequence = -1
        self._pending_at_ns = 0
        self._last_queue_wait_ns = self._max_queue_wait_ns = 0
        self._last_cycle_ns = self._max_cycle_ns = 0
        self._last_submitted = -1
        self._active_sequence = -1
        self._last_completed = -1
        self._closing = False
        self._submitted = self._processed = self._overwritten = 0
        self._rejected = self._errors = 0
        self._last_error = ''
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._thread.start()

    def submit(self, sequence, item):
        sequence = int(sequence)
        with self._condition:
            if self._closing or item is None or sequence <= self._last_submitted:
                self._rejected += 1
                return False
            if self._pending is not None:
                self._overwritten += 1
            self._last_submitted = sequence
            self._pending = item
            self._pending_sequence = sequence
            self._pending_at_ns = time.monotonic_ns()
            self._submitted += 1
            self._condition.notify()
            return True

    def _run(self):
        while True:
            with self._condition:
                while self._pending is None and not self._closing:
                    self._condition.wait()
                if self._closing:
                    self._pending = None
                    self._pending_sequence = -1
                    return
                item = self._pending
                sequence = self._pending_sequence
                self._pending = None
                self._pending_sequence = -1
                self._active_sequence = sequence
                callback_started_ns = time.monotonic_ns()
                self._last_queue_wait_ns = max(0, callback_started_ns - self._pending_at_ns)
                self._max_queue_wait_ns = max(self._max_queue_wait_ns,
                                              self._last_queue_wait_ns)
            try:
                self._callback(item)
            except Exception as exc:
                with self._condition:
                    self._errors += 1
                    self._last_error = type(exc).__name__ + ':' + str(exc)[:160]
                if self._on_error is not None:
                    try:
                        self._on_error(exc)
                    except Exception:
                        pass  # independent Guard still expires old intent
            finally:
                with self._condition:
                    self._processed += 1
                    self._last_cycle_ns = max(0, time.monotonic_ns() - callback_started_ns)
                    self._max_cycle_ns = max(self._max_cycle_ns, self._last_cycle_ns)
                    self._last_completed = sequence
                    self._active_sequence = -1
                    self._condition.notify_all()

    def counters(self):
        with self._condition:
            return WorkerCounters(
                self._submitted, self._processed, self._overwritten,
                self._rejected, self._errors, self._pending_sequence,
                self._active_sequence, self._last_completed, self._last_error,
                self._last_queue_wait_ns, self._max_queue_wait_ns,
                self._last_cycle_ns, self._max_cycle_ns,
            )

    def close(self, timeout_s=2.0):
        with self._condition:
            self._closing = True
            self._pending = None
            self._pending_sequence = -1
            self._condition.notify_all()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=max(0.0, float(timeout_s)))
        return not self._thread.is_alive()
