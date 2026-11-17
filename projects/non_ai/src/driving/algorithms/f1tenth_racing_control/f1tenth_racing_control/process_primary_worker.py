"""Isolated, single-flight MPC process with fail-closed result deadline.

This is a selectable *solver* process, never a Guard or actuator publisher.
Spawn is used deliberately to avoid forking the live ROS2 DDS participant.
No primary result is consumed after its original monotonic deadline.
"""
from dataclasses import dataclass
import multiprocessing as mp
from queue import Empty, Full
import time
import threading

from .primary_worker import PrimaryWorkerCounters


def _process_loop(jobs, replies, ready, dynamics, limits, config, formulation):
    # Construct the numerical solver AFTER spawning: its mutable warm-start is
    # never shared with the rclpy executor or the fallback computation thread.
    from .mpc import MpcConfig, LtvDynamicBicycleMpc, NonlinearDynamicBicycleMpc
    cls = NonlinearDynamicBicycleMpc if formulation == 'nmpc' else LtvDynamicBicycleMpc
    solver = cls(dynamics, limits, MpcConfig(**config))
    ready.set()  # A spawned ROS-independent process is fully initialized.
    while True:
        request = jobs.get()
        if request is None:
            return
        seq, reference, msg, station, previous, cap, deadline_ns = request
        started = time.perf_counter_ns()
        try:
            if started >= deadline_ns:
                raise TimeoutError('MPC_PROCESS_QUEUE_DEADLINE_EXPIRED')
            value = solver.solve(reference, msg, station, previous, cap,
                                 deadline_ns=deadline_ns)
            error = None
        except Exception as exc:
            value = None
            error = type(exc).__name__ + ':' + str(exc)[:180]
        finished = time.perf_counter_ns()
        # No stale result is ever transmitted, much less used as a warm start.
        if finished > deadline_ns:
            value = None
            error = 'MPC_PROCESS_SOLVE_DEADLINE_EXPIRED'
        try:
            replies.put_nowait((seq, value, error, started, finished, deadline_ns))
        except Full:
            # A congested channel cannot grant authorization.
            pass


class IsolatedMpcProcessWorker:
    """One-flight solver; expired generation can never publish a later result.

    Process recycling is supervised OUTSIDE the bounded R-B callback. Until a
    fresh spawned solver signals ready, submit_request is rejected and the
    original Guard-certified fallback retains all actuator authority.
    """
    def __init__(self, *, dynamics, limits, mpc_config, formulation='ltv'):
        if formulation not in ('ltv', 'nmpc'):
            raise ValueError('UNKNOWN_MPC_PROCESS_FORMULATION')
        self._context = mp.get_context('spawn')
        self._solver_args = (dynamics, limits, mpc_config, formulation)
        self._lock = threading.Lock()
        self._recycle_event = threading.Event()
        self._closing = False
        self._recycling = False
        self._permanent_failure = False
        self._restart_count = 0
        self._restart_failures = 0
        self._last_recycle_reason = ''
        self._active_sequence = -1
        self._last_sequence = -1
        self._last_completed = -1
        self._submitted = self._completed = self._busy_rejected = 0
        self._late_rejected = self._exceptions = 0
        self._last_compute_ns = self._max_compute_ns = 0
        self._child, self._jobs, self._replies = self._spawn_ready()
        self._supervisor = threading.Thread(
            target=self._supervisor_loop, name='r_b_mpc_process_supervisor', daemon=True)
        self._supervisor.start()

    def _spawn_ready(self):
        jobs = self._context.Queue(maxsize=1)
        replies = self._context.Queue(maxsize=1)
        ready = self._context.Event()
        child = self._context.Process(
            target=_process_loop, name='r_b_mpc_isolated',
            args=(jobs, replies, ready, *self._solver_args), daemon=True)
        try:
            child.start()
            # Never wait for solver construction in the 7ms/15ms safety path.
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline and not self._closing:
                if ready.wait(timeout=0.05):
                    if child.is_alive() and not self._closing:
                        return child, jobs, replies
                    break
                if not child.is_alive():
                    break
            raise RuntimeError('MPC_ISOLATED_WORKER_PRESTART_NOT_READY')
        except BaseException:
            self._dispose(child, jobs, replies)
            raise

    @staticmethod
    def _dispose(child, jobs, replies):
        # Never hold the controller-facing lock during joins/termination.
        if child.pid is not None:
            if child.is_alive():
                child.terminate()
            child.join(timeout=0.5)
            if child.is_alive():
                child.kill()
                child.join(timeout=0.5)
        for queue in (jobs, replies):
            queue.cancel_join_thread()
            queue.close()
        return child.pid is None or not child.is_alive()

    def _schedule_recycle_locked(self, reason):
        if self._closing or self._permanent_failure or self._recycling:
            return
        self._recycling = True
        self._last_recycle_reason = reason
        self._recycle_event.set()

    def _supervisor_loop(self):
        while True:
            self._recycle_event.wait()
            self._recycle_event.clear()
            with self._lock:
                if self._closing:
                    return
                if not self._recycling:
                    continue
                old = self._child, self._jobs, self._replies
            disposed = self._dispose(*old)
            if not disposed:
                # Never permit two concurrently running MPC solvers.
                with self._lock:
                    self._permanent_failure = True
                    self._restart_failures += 1
                continue
            fresh = None
            try:
                fresh = self._spawn_ready()
            except Exception:
                with self._lock:
                    self._permanent_failure = True
                    self._restart_failures += 1
            if fresh is None:
                continue
            with self._lock:
                if self._closing:
                    discard = True
                else:
                    self._child, self._jobs, self._replies = fresh
                    # The old generation, including every queued late reply, is
                    # destroyed. Its sequence ID is never reused.
                    self._active_sequence = -1
                    self._recycling = False
                    self._restart_count += 1
                    discard = False
            if discard:
                self._dispose(*fresh)
                return

    def submit_request(self, sequence, reference, message, station, previous,
                       speed_cap, deadline_ns):
        seq, deadline = int(sequence), int(deadline_ns)
        with self._lock:
            if (self._closing or self._recycling or self._permanent_failure or
                    self._active_sequence >= 0 or seq <= self._last_sequence or
                    time.perf_counter_ns() >= deadline):
                self._busy_rejected += 1
                return False
            if not self._child.is_alive():
                self._schedule_recycle_locked('MPC_PROCESS_DIED_BEFORE_SUBMIT')
                self._busy_rejected += 1
                return False
            try:
                self._jobs.put_nowait((seq, reference, message, station,
                            previous, speed_cap, deadline))
            except Full:
                self._busy_rejected += 1
                return False
            except (OSError, EOFError, ValueError):
                # The feeder/pipe can close AFTER is_alive() succeeds.
                # Never let IPC exceptions escape the R-B callback and bypass
                # the safe fallback, or leave a stale worker generation active.
                self._schedule_recycle_locked('MPC_PROCESS_CHANNEL_WRITE_FAILED')
                self._busy_rejected += 1
                return False
            self._last_sequence = self._active_sequence = seq
            self._submitted += 1
            return True

    def take(self, sequence, *, deadline_ns):
        seq, deadline = int(sequence), int(deadline_ns)
        with self._lock:
            if (self._closing or self._recycling or
                    seq != self._active_sequence):
                raise RuntimeError('MPC_PROCESS_TICK_IDENTITY_UNPROVEN')
            child, replies = self._child, self._replies
        received_exact = False
        recycle_reason = None
        try:
            while True:
                now = time.perf_counter_ns()
                if now >= deadline or not child.is_alive():
                    recycle_reason = 'MPC_PROCESS_TIMEOUT_OR_DEAD'
                    self._late_rejected += 1
                    raise RuntimeError(recycle_reason)
                try:
                    response = replies.get(timeout=min((deadline-now)*1e-9, .001))
                except Empty:
                    continue
                except (OSError, EOFError, ValueError) as exc:
                    recycle_reason = 'MPC_PROCESS_CHANNEL_READ_FAILED'
                    self._exceptions += 1
                    raise RuntimeError(recycle_reason) from exc
                completed_seq, value, error, started, finished, original = response
                if completed_seq != seq:
                    recycle_reason = 'MPC_PROCESS_REPLY_REPLAY_OR_REORDER'
                    self._late_rejected += 1
                    raise RuntimeError(recycle_reason)
                received_exact = True
                self._last_completed = seq
                self._completed += 1
                self._last_compute_ns = max(0, finished-started)
                self._max_compute_ns = max(self._max_compute_ns, self._last_compute_ns)
                if (finished > original or time.perf_counter_ns() >= deadline or
                        time.perf_counter_ns() >= original):
                    self._late_rejected += 1
                    raise RuntimeError('MPC_PROCESS_RESULT_EXPIRED')
                if error is not None:
                    self._exceptions += 1
                    raise RuntimeError('MPC_PROCESS_FAIL_CLOSED:' + error)
                return value
        finally:
            with self._lock:
                if recycle_reason is not None:
                    self._schedule_recycle_locked(recycle_reason)
                if received_exact and not self._recycling:
                    self._active_sequence = -1

    def counters(self):
        with self._lock:
            return PrimaryWorkerCounters(self._submitted, self._completed,
                self._busy_rejected, self._late_rejected, self._exceptions,
                self._active_sequence, self._last_completed,
                self._last_compute_ns, self._max_compute_ns)

    def recovery_status(self):
        """Diagnostics only; no field grants actuator or Guard authority."""
        with self._lock:
            return {
                'recycling': self._recycling,
                'permanent_failure': self._permanent_failure,
                'restart_count': self._restart_count,
                'restart_failures': self._restart_failures,
                'last_recycle_reason': self._last_recycle_reason,
            }

    def close(self, timeout_s=2.0):
        with self._lock:
            self._closing = True
            self._recycle_event.set()
        self._supervisor.join(timeout=max(0.0, float(timeout_s)))
        if self._supervisor.is_alive():
            return False
        with self._lock:
            child, jobs, replies = self._child, self._jobs, self._replies
        return self._dispose(child, jobs, replies)
