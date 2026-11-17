"""V6 off-executor latest-only R-B worker: deterministic fault injection."""
from threading import Event

from f1tenth_racing_control.control_worker import LatestOnlyControlWorker


def test_one_receipt_runs_exactly_once():
    completed = Event()
    done = []
    worker = LatestOnlyControlWorker(lambda x: (done.append(x), completed.set()))
    try:
        assert worker.submit(1, 's1')
        assert completed.wait(2)
        assert done == ['s1']
        assert worker.counters().processed == 1
    finally:
        assert worker.close()


def test_mailbox_superseded_work_never_replayed():
    active = Event()
    release = Event()
    done = Event()
    seen = []
    def process(x):
        seen.append(x)
        if x == 1:
            active.set()
            assert release.wait(2)
        if x == 3:
            done.set()
    worker = LatestOnlyControlWorker(process)
    try:
        assert worker.submit(1, 1)
        assert active.wait(2)
        assert worker.submit(2, 2)
        assert worker.submit(3, 3)
        assert worker.counters().overwritten == 1
        release.set()
        assert done.wait(2)
        assert seen == [1, 3]
        assert worker.counters().last_completed_sequence == 3
    finally:
        release.set()
        assert worker.close()


def test_duplicate_and_regressive_sequences_rejected_even_when_busy():
    active = Event()
    release = Event()
    def process(_):
        active.set()
        assert release.wait(2)
    worker = LatestOnlyControlWorker(process)
    try:
        assert worker.submit(42, 42)
        assert active.wait(2)
        assert not worker.submit(42, 43)
        assert not worker.submit(41, 43)
        assert worker.counters().rejected == 2
    finally:
        release.set()
        assert worker.close()


def test_caught_worker_exception_notifies_but_never_promotes():
    failed = Event()
    completed = Event()
    errors = []
    seen = []
    def process(x):
        seen.append(x)
        if x == 1:
            raise ValueError('poison')
        completed.set()
    def on_error(exc):
        errors.append(type(exc).__name__)
        failed.set()
    worker = LatestOnlyControlWorker(process, on_error=on_error)
    try:
        assert worker.submit(1, 1)
        assert failed.wait(2)
        assert worker.submit(2, 2)
        assert completed.wait(2)
        counters = worker.counters()
        assert counters.errors == 1
        assert counters.last_error == 'ValueError:poison'
        assert seen == [1, 2]
        assert errors == ['ValueError']
    finally:
        assert worker.close()


def test_error_reporting_exception_does_not_kill_consumer():
    failed = Event()
    first_exception = Event()
    def process(x):
        if x == 1:
            raise RuntimeError('bad')
        failed.set()
    def broken_logger(_):
        first_exception.set()
        raise RuntimeError('logger failure')
    worker = LatestOnlyControlWorker(process, on_error=broken_logger)
    try:
        assert worker.submit(1, 1)
        assert first_exception.wait(2)
        # The callback continues despite secondary logging failure.
        assert worker.submit(2, 'ok')
        assert failed.wait(2)
    finally:
        assert worker.close()


def test_close_discards_waiting_command_not_drain_to_actuator():
    active = Event()
    release = Event()
    seen = []
    def process(x):
        seen.append(x)
        active.set()
        assert release.wait(2)
    worker = LatestOnlyControlWorker(process)
    try:
        assert worker.submit(1, 1)
        assert active.wait(2)
        assert worker.submit(2, 2)
        assert not worker.close(timeout_s=0.005)
        assert not worker.submit(3, 3)
        release.set()
        assert worker.close()
        assert seen == [1]
    finally:
        release.set()
        worker.close()


def test_empty_close_clean_and_idempotent():
    worker = LatestOnlyControlWorker(lambda _: None)
    assert worker.close()
    assert worker.close()
    assert not worker.submit(1, 1)


def test_slow_consumer_does_not_block_producer_receipt():
    active = Event()
    release = Event()
    def process(_):
        active.set()
        release.wait(2)
    worker = LatestOnlyControlWorker(process)
    try:
        assert worker.submit(1, 1)
        assert active.wait(2)
        assert worker.submit(2, 2)
        assert worker.counters().active_sequence == 1
        assert worker.counters().pending_sequence == 2
    finally:
        release.set()
        assert worker.close()


def test_concurrent_submitters_share_monotonic_sequence_boundary():
    # Strictly increasing acceptance is enforced under a Condition lock.
    barrier = Event()
    worker = LatestOnlyControlWorker(lambda _: barrier.wait(1))
    from concurrent.futures import ThreadPoolExecutor
    try:
        with ThreadPoolExecutor(max_workers=8) as executor:
            results = list(executor.map(lambda x: worker.submit(x, x), range(1, 41)))
        assert any(results)
        assert worker.counters().submitted + worker.counters().rejected == 40
        assert not worker.submit(1, 1)
    finally:
        barrier.set()
        assert worker.close()


def test_queue_wait_and_cycle_duration_measurements_are_monotonic():
    done = Event()
    worker = LatestOnlyControlWorker(lambda _: done.set())
    try:
        assert worker.submit(12, 'm')
        assert done.wait(2)
        # Completion counters update just after the callback signals done.
        worker.close()
        c = worker.counters()
        assert c.last_queue_wait_ns >= 0
        assert c.max_queue_wait_ns >= c.last_queue_wait_ns
        assert c.last_cycle_ns >= 0
        assert c.max_cycle_ns >= c.last_cycle_ns
        assert c.processed == 1
    finally:
        worker.close()


def test_null_payload_is_rejected_before_it_can_stall_the_consumer():
    worker = LatestOnlyControlWorker(lambda _: None)
    try:
        assert not worker.submit(1, None)
        assert worker.submit(1, 1)
        assert worker.counters().rejected == 1
    finally:
        assert worker.close()
