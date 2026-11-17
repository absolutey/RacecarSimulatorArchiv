from types import SimpleNamespace as NS
import pytest
from f1tenth_racing_overtaking.state_input_contract import admit_ego_state


def state(seq=1,stamp_ns=1_000_000_000,**kwargs):
    s=NS(physics_sequence=seq,physics_stamp=NS(sec=stamp_ns//1_000_000_000,
          nanosec=stamp_ns%1_000_000_000),px=0.,py=0.,yaw=0.,v=0.,vx=0.,vy=0.,
          omega=0.,a=0.,accel=0.,slip_angle=0.,steer=0.)
    for k,v in kwargs.items():setattr(s,k,v)
    return s


def check(s,now=1_050_000_000,**kw):
    return admit_ego_state(s,now_ns=now,**kw)


def test_fresh_finite_accepted():
    p=check(state()); assert p.admissible and p.fresh_identity


@pytest.mark.parametrize('bad',['px','py','yaw','v','vx','vy','omega','a','accel','slip_angle','steer'])
def test_each_nonfinite_poison_invalidate_newest_source(bad):
    p=check(state(seq=9,**{bad:float('nan')}),last_sequence=8)
    assert p.fresh_identity and not p.admissible


def test_reordered_sequence_is_rejected_without_cache_replacement():
    p=check(state(seq=4),last_sequence=5)
    assert not p.fresh_identity and not p.admissible


def test_duplicate_stamp_is_rejected():
    p=check(state(seq=4),last_stamp_ns=1_000_000_000)
    assert not p.fresh_identity


def test_future_is_denied():
    p=check(state(stamp_ns=1_100_000_000))
    assert p.fresh_identity and not p.admissible


def test_over_age_denied():
    p=check(state(stamp_ns=1_000_000_000),now=1_076_000_000)
    assert p.fresh_identity and not p.admissible


def test_exact_age_limit_allowed():
    assert check(state(),now=1_075_000_000).admissible


def test_zero_stamp_denied():
    assert not check(state(stamp_ns=0)).admissible


def test_poisoned_latest_invalidates_prior_cache():
    x=check(state(seq=2,px=float('inf')),last_sequence=1)
    assert x.fresh_identity and not x.admissible and x.source_sequence==2
    assert not check(state(seq=1),last_sequence=x.source_sequence).fresh_identity
