"""No G8 promotion without an independently witnessed abort/retry."""
import importlib.util
from pathlib import Path
P=Path(__file__).resolve().parents[4]/'verification/qualification_gates/gate_model.py'
spec=importlib.util.spec_from_file_location('test_g8_pure',P)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def values():
    return dict(overtakes_completed=1, abort_or_rejoin_verified=True,
        abort_events=1,successful_retries=1,
        fresh_red_after_abort_verified=True,retry_used_new_plan_identity=True,
        wall_collisions=0,vehicle_collisions=0)

def test_complete_abort_retry_witness_can_be_provisionally_reviewed():
    assert m._metric_ok('G8',values())

def test_no_replay_fence_no_promote():
    for field in ('fresh_red_after_abort_verified','retry_used_new_plan_identity'):
        v=values();v[field]=False
        assert not m._metric_ok('G8',v)

def test_no_abort_retry_no_promote():
    for field in ('abort_events','successful_retries'):
        v=values();v[field]=0
        assert not m._metric_ok('G8',v)

def test_any_collision_denied():
    v=values();v['wall_collisions']=1
    assert not m._metric_ok('G8',v)
