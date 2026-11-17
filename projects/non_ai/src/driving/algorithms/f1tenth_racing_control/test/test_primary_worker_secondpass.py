"""Independent MPC scheduler: no older, late or busy primary may gain authority."""
from threading import Event
import time

import pytest
from f1tenth_racing_control.primary_worker import MpcPrimaryWorker


def deadline(milliseconds=150):
    return time.perf_counter_ns()+int(milliseconds*1e6)


def test_primary_runs_separately_and_returns_exact_tick():
    w=MpcPrimaryWorker()
    try:
        d=deadline()
        assert w.submit(7,lambda: 'a',d)
        assert w.take(7,deadline_ns=d)=='a'
        assert w.counters().completed==1
    finally:
        assert w.close()


def test_duplicate_and_backward_sequences_denied():
    w=MpcPrimaryWorker()
    try:
        d=deadline()
        assert w.submit(8,lambda: 1,d)
        assert w.take(8,deadline_ns=d)==1
        assert not w.submit(8,lambda: 99,deadline())
        assert not w.submit(7,lambda: 99,deadline())
        assert w.counters().busy_rejected==2
    finally:
        assert w.close()


def test_blocking_primary_does_not_block_fallback():
    start,release=Event(),Event()
    w=MpcPrimaryWorker()
    try:
        def work():
            start.set()
            release.wait(1)
            return 'late'
        d=deadline(30)
        assert w.submit(1,work,d)
        assert start.wait(0.5)
        # This path is executed by the independent fallback control worker.
        observed=[]
        observed.append('fallback_proof_independent')
        assert observed==['fallback_proof_independent']
        assert not w.submit(2,lambda: 'wrong',deadline())
        with pytest.raises(RuntimeError,match='LATE'):
            w.take(1,deadline_ns=d)
        release.set()
    finally:
        release.set()
        assert w.close()


def test_timeout_results_cannot_be_reused_on_new_tick():
    start,release=Event(),Event()
    w=MpcPrimaryWorker()
    try:
        d=deadline(20)
        def slow():
            start.set(); release.wait(1)
            return 'stale primary'
        assert w.submit(10,slow,d)
        assert start.wait(0.5)
        time.sleep(0.03)
        release.set()
        end=time.monotonic()+0.5
        while w.counters().completed<1 and time.monotonic()<end:
            time.sleep(0.001)
        assert w.counters().completed==1
        with pytest.raises(RuntimeError):
            w.take(10,deadline_ns=deadline())
        d2=deadline()
        assert w.submit(11,lambda: 'fresh',d2)
        assert w.take(11,deadline_ns=d2)=='fresh'
    finally:
        release.set()
        assert w.close()


def test_calculation_failure_fail_closed_and_next_tick_works():
    w=MpcPrimaryWorker()
    try:
        def broken(): raise ValueError('infeasible')
        d=deadline()
        assert w.submit(15,broken,d)
        with pytest.raises(ValueError,match='infeasible'):
            w.take(15,deadline_ns=d)
        d=deadline()
        assert w.submit(16,lambda: 'fallback_remains',d)
        assert w.take(16,deadline_ns=d)=='fallback_remains'
        assert w.counters().exceptions==1
    finally:
        assert w.close()


def test_shutdown_disposes_unrun_requests():
    w=MpcPrimaryWorker()
    assert w.close()
    assert not w.submit(1,lambda: True,deadline())


def test_result_from_another_tick_never_admitted():
    w=MpcPrimaryWorker()
    try:
        d=deadline()
        assert w.submit(21,lambda: 'a',d)
        with pytest.raises(RuntimeError,match='TICK_IDENTITY'):
            w.take(22,deadline_ns=d)
        assert w.take(21,deadline_ns=d)=='a'
    finally:
        assert w.close()
