"""Non-blocking diagnostics do not invoke JSON encoder from control thread."""
import json
from types import SimpleNamespace
from f1tenth_racing_control import forensic_trace as t


class Mailbox:
    def __init__(self):
        self.items=[]
    def put_nowait(self, value):
        self.items.append(value)


def test_guard_and_rb_diagnostic_queue_is_unserialized(monkeypatch):
    q=Mailbox()
    monkeypatch.setattr(t,'_QUEUE',q)
    def no_sync_json(*args, **kwargs):
        raise AssertionError('JSON serialization attempted on guard/control thread')
    monkeypatch.setattr(t.json,'dumps',no_sync_json)
    t.trace_guard('CERTIFICATE',sequence=123,time_us=700.)
    t.trace_rb('BUDGET',cpu_us=200.,wall_us=1100.)
    assert len(q.items)==2
    assert q.items[0][0]=='GUARD'
    assert q.items[0][1]['sequence']==123
    assert q.items[1][1]['cpu_us']==200.


def test_diagnostics_can_be_disabled_without_affecting_actuation(monkeypatch):
    monkeypatch.setattr(t,'_QUEUE',None)
    t.trace_guard('NOT_AUTHORIZATION',any_data='unimportant')
