"""Separately scheduled watchdog's deterministic, authority-free core."""
from types import SimpleNamespace as N
import pytest
from f1tenth_racing_sim.guard_status_lease import GuardStatusLease
from f1tenth_racing_control.independent_watchdog_core import IndependentWatchdogCore


def make_core():
    return IndependentWatchdogCore(guard_lease=GuardStatusLease(run_epoch='e1',producer='R_B'))


def good_guard(core, stamp=1_000_000_000, seq=4, steady=1):
    msg=N(stamp=N(sec=stamp//1_000_000_000,nanosec=stamp%1_000_000_000),
          run_epoch='e1',source_id='R_B',last_accepted_sequence=seq,
          mode='RUNNING',ready=True,certificate_verified=True,
          emergency_unrecoverable=False)
    return core.guard_lease.observe(msg,now_ros_ns=stamp,now_steady_ns=steady)


def good_state(core):
    return core.receive_state(sequence=5,stamp_ns=1_000_000_000,
                              now_ros_ns=1_000_000_000,now_steady_ns=1)


def test_no_guard_no_state_denied():
    assert make_core().decide(now_ros_ns=1_000_000_000,now_steady_ns=2)[0] is False


def test_missing_guard_denied_even_with_state():
    c=make_core();good_state(c)
    assert not c.decide(now_ros_ns=1_000_000_000,now_steady_ns=2)[0]


def test_missing_state_denied_even_with_guard():
    c=make_core();good_guard(c)
    assert not c.decide(now_ros_ns=1_000_000_000,now_steady_ns=2)[0]


def test_healthy_fresh_guard_and_state_remove_watchdog_veto():
    c=make_core();good_guard(c);good_state(c)
    assert c.decide(now_ros_ns=1_025_000_000,now_steady_ns=25_000_000)[0]


def test_guard_staleness_rejected_after_valid():
    c=make_core();good_guard(c);good_state(c)
    assert not c.decide(now_ros_ns=1_101_000_000,now_steady_ns=101_000_000)[0]


def test_collision_latches_even_if_bool_subsequently_false():
    c=make_core();good_guard(c);good_state(c)
    c.on_collision(True);c.on_collision(False)
    assert not c.decide(now_ros_ns=1_025_000_000,now_steady_ns=25_000_000)[0]
    assert c.collision_latched


def test_state_regression_clears_state():
    c=make_core();good_guard(c);good_state(c)
    assert not c.receive_state(sequence=4,stamp_ns=1_000_000_000,
                              now_ros_ns=1_025_000_000,now_steady_ns=100)
    assert not c.decide(now_ros_ns=1_025_000_000,now_steady_ns=200)[0]


def test_future_state_cannot_authorize():
    c=make_core();good_guard(c)
    assert not c.receive_state(sequence=1,stamp_ns=2_000_000_000,
                              now_ros_ns=1_000_000_000,now_steady_ns=1)


def test_clock_stall_revokes():
    c=make_core();good_guard(c);good_state(c)
    assert c.decide(now_ros_ns=1_010_000_000,now_steady_ns=2)[0]
    assert not c.decide(now_ros_ns=1_010_000_000,now_steady_ns=3)[0]


def test_invalid_state_timeout_does_not_turn_into_allow():
    c=make_core();good_guard(c);good_state(c)
    allowed,reason=c.decide(now_ros_ns=1_101_000_000,now_steady_ns=2)
    assert not allowed and reason=='STATE_STALE'


def test_bad_lease_configuration_rejected():
    with pytest.raises(ValueError):
        IndependentWatchdogCore(guard_lease=None,max_state_age_ns=500_000_000)
