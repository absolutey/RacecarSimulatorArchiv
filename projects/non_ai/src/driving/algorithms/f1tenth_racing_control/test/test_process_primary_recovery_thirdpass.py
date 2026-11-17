"""Fail-closed recovery of a timed-out or crashed isolated MPC solver."""
import time
from pathlib import Path

import pytest
import yaml

from f1tenth_racing_control.process_primary_worker import IsolatedMpcProcessWorker
from test_process_primary_worker_firstpass import config, inputs


def worker():
    root = Path(__file__).resolve().parents[4]
    environment = yaml.safe_load((root/'environment/f1tenth_racing_sim/config/competition_environment.yaml').read_text())
    dyn = environment['dynamics']
    return IsolatedMpcProcessWorker(
        dynamics=dyn, limits=dyn['limits'], mpc_config=config(), formulation='ltv')


def wait_recovered(w, duration_s=8):
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        status = w.recovery_status()
        if status['restart_count'] == 1 and not status['recycling']:
            return
        if status['permanent_failure']:
            pytest.fail('solver could not be restarted')
        time.sleep(.02)
    pytest.fail('bounded asynchronous solver restart not completed')


def test_timeout_discards_entire_generation_before_next_primary():
    w = worker()
    try:
        reference, state = inputs()
        deadline = time.perf_counter_ns() + 2_000_000_000
        assert w.submit_request(51, reference, state, 1., (0., 0.), 2., deadline)
        with pytest.raises(RuntimeError, match='MPC_PROCESS_TIMEOUT_OR_DEAD'):
            w.take(51, deadline_ns=time.perf_counter_ns() - 1)
        assert w.recovery_status()['recycling']
        assert not w.submit_request(52, reference, state, 1., (0., 0.), 2.,
                                    time.perf_counter_ns() + 2_000_000_000)
        wait_recovered(w)
        assert w.counters().outstanding_sequence == -1
        # Stale sequences stay denied even after process generation changes.
        assert not w.submit_request(51, reference, state, 1., (0., 0.), 2.,
                                    time.perf_counter_ns() + 2_000_000_000)
        deadline = time.perf_counter_ns() + 2_000_000_000
        assert w.submit_request(53, reference, state, 1., (0., 0.), 2., deadline)
        result = w.take(53, deadline_ns=deadline)
        assert result.status == 'PRIMARY_VALID'
        assert w.recovery_status()['restart_count'] == 1
        assert w.counters().completed == 1
    finally:
        assert w.close()


def test_process_crash_uses_fallback_until_new_generation_ready():
    w = worker()
    try:
        reference, state = inputs()
        w._child.terminate()
        w._child.join(timeout=1.)
        assert not w.submit_request(71, reference, state, 1., (0., 0.), 2.,
                                    time.perf_counter_ns() + 2_000_000_000)
        assert w.recovery_status()['recycling']
        wait_recovered(w)
        deadline = time.perf_counter_ns() + 2_000_000_000
        assert w.submit_request(72, reference, state, 1., (0., 0.), 2., deadline)
        assert w.take(72, deadline_ns=deadline).status == 'PRIMARY_VALID'
    finally:
        assert w.close()


def test_shutdown_denies_new_generation_and_no_primary_reactivation():
    w = worker()
    reference, state = inputs()
    assert w.close()
    assert not w.submit_request(81, reference, state, 1., (0., 0.), 2.,
                                time.perf_counter_ns() + 2_000_000_000)
