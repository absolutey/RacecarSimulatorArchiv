"""Simulator-side independent heartbeat veto; ROS-free behavior tests."""
from types import SimpleNamespace as N
from pathlib import Path
from f1tenth_racing_sim.guard_status_lease import GuardStatusLease


def msg(*, stamp=1_000_000_000, seq=3, mode='RUNNING',
        ready=True, proof=True, epoch='run-A', producer='R_B', fault=False):
    return N(stamp=N(sec=stamp//1_000_000_000,nanosec=stamp%1_000_000_000),
             run_epoch=epoch,source_id=producer,last_accepted_sequence=seq,
             mode=mode,ready=ready,certificate_verified=proof,
             emergency_unrecoverable=fault)


def lease(): return GuardStatusLease(run_epoch='run-A',producer='R_B')


def test_no_heartbeat_is_never_authority():
    assert not lease().permits_certified(now_ros_ns=1,now_steady_ns=1)[0]


def test_running_heartbeat_is_a_veto_not_an_exact_command_seq_join():
    l=lease()
    assert l.observe(msg(),now_ros_ns=1_002_000_000,now_steady_ns=1_000_000)
    assert l.permits_certified(now_ros_ns=1_060_000_000,now_steady_ns=71_000_000)[0]


def test_guard_process_death_revokes_existing_lease_on_monotonic_timeout():
    l=lease(); l.observe(msg(),now_ros_ns=1_000_000_000,now_steady_ns=500)
    assert not l.permits_certified(now_ros_ns=1_001_000_000,now_steady_ns=100_000_501)[0]


def test_ros_clock_regression_or_stall_revokes():
    l=lease(); l.observe(msg(),now_ros_ns=1_000_000_000,now_steady_ns=5)
    assert not l.permits_certified(now_ros_ns=999_999_999,now_steady_ns=6)[0]
    assert not l.permits_certified(now_ros_ns=1_100_000_001,now_steady_ns=6)[0]


def test_safe_hold_revokes_authority_even_if_cached_certificate_was_true():
    l=lease();l.observe(msg(),now_ros_ns=1_000_000_000,now_steady_ns=10)
    l.observe(msg(stamp=1_025_000_000,mode='SAFE_HOLD'),
              now_ros_ns=1_025_000_000,now_steady_ns=20)
    assert not l.permits_certified(now_ros_ns=1_030_000_000,now_steady_ns=30)[0]


def test_rejects_stale_or_wrong_run_epoch_and_source():
    for m in (msg(epoch='wrong'),msg(producer='OTHER'),
              msg(stamp=1_000_000_000)):
        l=lease()
        assert not l.observe(m,now_ros_ns=1_200_000_000,now_steady_ns=1)
        assert not l.permits_certified(now_ros_ns=1_200_000_000,now_steady_ns=2)[0]


def test_invalid_update_revokes_even_previous_valid_heartbeat():
    l=lease();l.observe(msg(),now_ros_ns=1_000_000_000,now_steady_ns=1)
    assert not l.observe(msg(stamp=1_001_000_000,epoch='wrong'),
                         now_ros_ns=1_001_000_000,now_steady_ns=2)
    assert not l.permits_certified(now_ros_ns=1_001_000_000,now_steady_ns=3)[0]


def test_sequence_or_timestamp_regression_revokes():
    for bad in (msg(stamp=1_001_000_000,seq=2),msg(stamp=999_999_999,seq=4)):
        l=lease();l.observe(msg(),now_ros_ns=1_000_000_000,now_steady_ns=1)
        assert not l.observe(bad,now_ros_ns=1_002_000_000,now_steady_ns=2)


def test_unarmed_or_unverified_or_hard_fault_status_is_veto():
    for m in (msg(mode='ARMED'),msg(ready=False),msg(proof=False),
              msg(fault=True),msg(seq=0)):
        l=lease();l.observe(m,now_ros_ns=1_000_000_000,now_steady_ns=1)
        assert not l.permits_certified(now_ros_ns=1_000_000_100,now_steady_ns=2)[0]


def test_simulator_commands_and_physics_are_guarded_by_atomic_lock():
    text=(Path(__file__).parents[1]/'f1tenth_racing_sim/simulator.py').read_text()
    for marker in ('with self.actuation_lock:',
                   'def receive_guard_status(self, message):',
                   'self.guard_status_lease.permits_certified(',
                   'heartbeat_live=True',
                   'self._revoke_guarded_lease('):
        assert marker in text
