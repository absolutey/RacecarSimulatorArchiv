"""ROS-free external process authorization veto; exercised in colcon tests."""
import json
from pathlib import Path
from types import SimpleNamespace as N
import pytest
from f1tenth_racing_sim.external_watchdog_lease import ExternalWatchdogLease
from f1tenth_racing_sim.guard_status_lease import GuardStatusLease


def verdict(*, epoch='epoch1', producer='R_B', seq=1, emitted=1_000_000_000,
            physics=5, guard=4, mode='ALLOW', reason='GUARD_HEARTBEAT_LIVE'):
    return dict(schema=1,run_epoch=epoch,producer=producer,sequence=seq,
                emitted_ros_ns=emitted,physics_sequence=physics,
                guard_accepted_sequence=guard,verdict=mode,reason=reason)


def lease():return ExternalWatchdogLease(run_epoch='epoch1',producer='R_B')


def observe(l,m, now=1_000_000_000, steady=500):
    return l.observe(json.dumps(m),now_ros_ns=now,now_steady_ns=steady)


def permit(l,now=1_000_000_000,steady=500,physics=5):
    return l.permits_certified(now_ros_ns=now,now_steady_ns=steady,
                               physics_sequence=physics)


def test_no_watchdog_means_no_certified():
    assert not permit(lease())[0]


def test_fresh_matching_watchdog_removes_only_its_own_veto():
    l=lease();assert observe(l,verdict());assert permit(l)[0]


def test_revoke_immediately_overrides_prior_allow():
    l=lease();assert observe(l,verdict())
    assert observe(l,verdict(seq=2,emitted=1_025_000_000,mode='REVOKE',
                             reason='STATE_STALE'),now=1_025_000_000,steady=25_000_500)
    assert not permit(l,now=1_025_000_001,steady=25_000_501)[0]


def test_revoked_verdict_never_grants_command():
    l=lease();assert observe(l,verdict(mode='REVOKE'))
    assert not permit(l)[0]


def test_watchdog_process_death_revokes_via_steady_clock():
    l=lease();assert observe(l,verdict())
    assert not permit(l,steady=100_000_501)[0]


def test_watchdog_ros_clock_age_revokes():
    l=lease();assert observe(l,verdict())
    assert not permit(l,now=1_100_000_001)[0]


def test_clock_regression_revokes():
    l=lease();assert observe(l,verdict())
    assert not permit(l,now=999_999_999)[0]


@pytest.mark.parametrize('item',[
    {'epoch':'wrong'}, {'producer':'other'}, {'seq':0}, {'emitted':0},
    {'physics':-1}, {'guard':-1}, {'mode':'ESCALATE'}])
def test_bad_provenance_fails_closed(item):
    l=lease();assert not observe(l,verdict(**item));assert not permit(l)[0]


def test_timestamp_or_sequence_rollback_revokes_cached_allow():
    l=lease();assert observe(l,verdict())
    assert not observe(l,verdict(seq=1,emitted=1_025_000_000),
                       now=1_025_000_000,steady=25_000_500)
    assert not permit(l,now=1_025_000_000,steady=25_000_500)[0]


def test_future_external_state_provenance_rejected():
    l=lease();assert observe(l,verdict(physics=30))
    assert not permit(l,physics=5)[0]


@pytest.mark.parametrize('key', ['sequence','emitted_ros_ns','physics_sequence','guard_accepted_sequence'])
def test_json_boolean_does_not_masquerade_as_int(key):
    l=lease();v=verdict();v[key]=True
    assert not observe(l,v)


def test_missing_guard_or_physics_provenance_rejected():
    for k in ('physics','guard'):
        l=lease();assert observe(l,verdict(**{k:0}))
        assert not permit(l)[0]


def test_revoke_cannot_unilaterally_authorize():
    l=lease();assert observe(l,verdict());l.revoke()
    assert not permit(l)[0]


def test_no_egovehicle_watchdog_disable_escape_hatch():
    text=(Path(__file__).parents[1]/'f1tenth_racing_sim/simulator.py').read_text()
    assert "raise ValueError('EGO_EXTERNAL_WATCHDOG_MANDATORY')" in text
    assert "'independent_guard_watchdog_lease0'" in text
    assert 'def receive_external_watchdog' in text
    assert 'not watchdog_live or not heartbeat_live' in text
    assert 'self.external_watchdog_lease.permits_certified' in text


def test_guard_and_external_veto_are_distinct():
    l=GuardStatusLease(run_epoch='epoch1',producer='R_B')
    e=lease();assert observe(e,verdict())
    assert not l.permits_certified(now_ros_ns=1_000_000_000,now_steady_ns=500)[0]


def test_separate_process_is_registered_and_launched():
    root=Path(__file__).resolve().parents[3]
    setup=(root/'driving/algorithms/f1tenth_racing_control/setup.py').read_text()
    launch=(root/'platform/f1tenth_racing_bringup/launch/race_foundation.launch.py').read_text()
    assert 'independent_guard_watchdog = ' in setup
    assert 'executable="independent_guard_watchdog"' in launch
    assert '"independent_watchdog_required": True' in launch
