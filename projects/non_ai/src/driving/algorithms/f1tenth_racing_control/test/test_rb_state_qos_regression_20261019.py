"""Regression for real 2026-10-19 Solo DDS 357ms R-B state starvation."""
from pathlib import Path
import ast
import copy

import pytest
from f1tenth_racing_control.rb_state_qos_contract import evaluate_rb_state_path

CONTROL = Path(__file__).resolve().parents[1] / 'f1tenth_racing_control' / 'node.py'
ROOT = Path(__file__).resolve().parents[5]
PRESTART = ROOT / 'PREFLIGHT_LIVE.py'
SIMULATOR = ROOT / 'src/environment/f1tenth_racing_sim/f1tenth_racing_sim/simulator.py'
GUARD = Path(__file__).resolve().parents[1] / 'f1tenth_racing_control' / 'actuator_guard_node.py'


def healthy():
    subscribers = [{'name':'r_b0_racing_pursuit','reliability':'BEST_EFFORT',
                    'depth':1, 'durability':'VOLATILE'}]
    publishers = [{'name':'competition_simulator0','reliability':'RELIABLE',
                   'depth':16, 'durability':'VOLATILE'}]
    metrics = {'type':'R_B_CONTROL_WORKER_SCHEDULING', 'schema_version':2,
               'state_rx_qos_reliability':'BEST_EFFORT', 'state_rx_qos_depth':1,
               'state_rx_count':100, 'state_rx_last_sequence':99,
               'state_rx_recent_ages_ns':[2_000_000]*8,
               'state_rx_last_receipt_steady_age_ns':5_000_000,
               'state_rx_recent_8_fresh':True}
    return subscribers, publishers, metrics


def test_live_fresh_qos_contract_passes():
    ok,details=evaluate_rb_state_path(*healthy())
    assert ok and details['rb_qos_endpoint_verified']
    assert details['rb_state_diagnostics_fresh']


@pytest.mark.parametrize('field,value',[
    ('reliability','RELIABLE'),
    ('depth',16),
    ('durability','TRANSIENT_LOCAL'),
    ('name','fake_r_b'),
])
def test_rb_endpoint_provenance_is_enforced(field,value):
    sub,pub,metric=healthy();sub[0][field]=value
    assert not evaluate_rb_state_path(sub,pub,metric)[0]


@pytest.mark.parametrize('field,value',[
    ('reliability','BEST_EFFORT'),
    ('depth',1),
    ('name','other_node'),
])
def test_simulator_reliable_state_publisher_is_enforced(field,value):
    sub,pub,metric=healthy();pub[0][field]=value
    assert not evaluate_rb_state_path(sub,pub,metric)[0]


def test_rmw_unknown_depth_is_not_mistaken_for_known_bad_depth():
    sub,pub,metric=healthy();sub[0]['depth']=0;pub[0]['depth']=0
    ok,details=evaluate_rb_state_path(sub,pub,metric)
    assert ok and details['dds_depth_reporting']=='UNKNOWN_BY_MIDDLEWARE'


def test_two_rb_endpoint_owners_fail_closed():
    sub,pub,metric=healthy();sub.append(copy.deepcopy(sub[0]))
    assert not evaluate_rb_state_path(sub,pub,metric)[0]


@pytest.mark.parametrize('field,value',[
    ('state_rx_recent_ages_ns',[357_000_000]*8),
    ('state_rx_recent_ages_ns',[1_000_000]*7),
    ('state_rx_recent_ages_ns',[1_000_000]*7+[-1]),
    ('state_rx_recent_ages_ns',[1_000_000]*7+[75_000_001]),
    ('state_rx_last_receipt_steady_age_ns',250_000_001),
    ('state_rx_last_receipt_steady_age_ns',-1),
    ('state_rx_count',7),
    ('state_rx_last_sequence',0),
    ('state_rx_recent_8_fresh',False),
    ('state_rx_qos_reliability','RELIABLE'),
    ('state_rx_qos_depth',16),
    ('schema_version',1),
])
def test_bad_or_stale_rb_receipts_reject_before_race_start(field,value):
    sub,pub,metric=healthy();metric[field]=value
    assert not evaluate_rb_state_path(sub,pub,metric)[0]


def test_missing_metrics_cannot_be_replaced_by_fresh_guard_state():
    sub,pub,_=healthy()
    for bad in (None, {}, {'state_rx_recent_8_fresh':True}):
        assert not evaluate_rb_state_path(sub,pub,bad)[0]


def test_code_wires_explicit_best_effort_qos_not_implicit_reliable_depth1():
    source=CONTROL.read_text()
    assert 'self.state_subscription_qos = QoSProfile(' in source
    assert 'depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)' in source
    assert 'self.receive_state_snapshot,\n                                 self.state_subscription_qos)' in source
    assert 'self.create_subscription(CarState, "state0", self.receive_state_snapshot, 1)' not in source
    # The 75ms execution-admission contract is NEVER lengthened to 357ms.
    assert 'min(self.state_freshness_ns, 75_000_000)' in source
    assert 'self._state_rx_recent_ages_ns.append(received_ros_ns - stamp_ns)' in source
    assert "'state_rx_recent_8_fresh': (" in source


def test_ros_publisher_and_independent_guard_contract_not_modified():
    sim=SIMULATOR.read_text()
    guard=GUARD.read_text()
    assert 'state_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=16,' in sim
    assert 'reliability=ReliabilityPolicy.RELIABLE)' in sim
    assert "state_sensor=QoSProfile(depth=16,reliability=ReliabilityPolicy.RELIABLE)" in guard
    assert "self.create_subscription(CarState,'state0',self.on_state,state_sensor)" in guard
    assert 'if elapsed_ns>15000000:' in guard


def test_live_prestart_gate_wires_r_b_receipt_freshness_and_endpoint_proof():
    source=PRESTART.read_text()
    ast.parse(source)
    assert "self.get_subscriptions_info_by_topic('/state0')" in source
    assert "self.get_publishers_info_by_topic('/state0')" in source
    assert "'/control_worker_metrics',self.on_rb_worker" in source
    assert 'rb_ok, rb_facts = self.rb_state_path()' in source
    assert 'monotonic_ok and fresh and recent and rb_ok)' in source


def test_rb_integrity_never_accepts_nonfinite_or_invented_good_samples():
    sub,pub,metric=healthy()
    for poisoned in ([float('nan')]*8, [float('inf')]*8, ['1']*8):
        m=dict(metric, state_rx_recent_ages_ns=poisoned)
        assert not evaluate_rb_state_path(sub,pub,m)[0]
