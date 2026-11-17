"""IPC transport failures must not escape the R-B submit path or strand it."""
from queue import Empty
import threading
import time
import pytest
from f1tenth_racing_control.process_primary_worker import IsolatedMpcProcessWorker

class Alive:
    def is_alive(self): return True

class ExplodingQueue:
    def __init__(self, failure):self.failure=failure
    def put_nowait(self, value):raise self.failure
    def get(self, timeout):raise self.failure


def bare_worker():
    w=object.__new__(IsolatedMpcProcessWorker)
    w._lock=threading.Lock()
    w._recycle_event=threading.Event()
    w._closing=False
    w._permanent_failure=False
    w._recycling=False
    w._last_recycle_reason=''
    w._active_sequence=-1
    w._last_sequence=-1
    w._child=Alive()
    w._busy_rejected=0
    w._late_rejected=0
    w._exceptions=0
    w._submitted=0
    w._completed=0
    return w

@pytest.mark.parametrize('error_type',[OSError,EOFError,ValueError])
def test_broken_job_pipe_denies_primary_and_requests_fresh_process(error_type):
    w=bare_worker()
    w._jobs=ExplodingQueue(error_type('broken job transport'))
    accepted=w.submit_request(1,object(),object(),0.,(0.,0.),8.,
                              time.perf_counter_ns()+1_000_000_000)
    assert not accepted
    assert w._recycling
    assert w._last_recycle_reason=='MPC_PROCESS_CHANNEL_WRITE_FAILED'
    assert w._busy_rejected==1
    assert w._active_sequence==-1

@pytest.mark.parametrize('error_type',[OSError,EOFError,ValueError])
def test_broken_reply_pipe_recycles_worker_and_fails_closed(error_type):
    w=bare_worker()
    w._active_sequence=w._last_sequence=4
    w._replies=ExplodingQueue(error_type('broken reply transport'))
    with pytest.raises(RuntimeError,match='MPC_PROCESS_CHANNEL_READ_FAILED'):
        w.take(4,deadline_ns=time.perf_counter_ns()+1_000_000_000)
    assert w._recycling
    assert w._last_recycle_reason=='MPC_PROCESS_CHANNEL_READ_FAILED'
    assert w._exceptions==1
