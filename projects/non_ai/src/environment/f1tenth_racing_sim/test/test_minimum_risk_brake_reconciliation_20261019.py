"""Emergency authority may brake but never accelerate, reverse, or certify.

This protects against the time-of-check/time-of-use speed-sign crossing seen
in the 2026-10-19 Solo capture (three invalid-emergency-brake rejections).
"""
from types import SimpleNamespace
import math
import pytest
from f1tenth_racing_sim.guarded_actuation_contract import (
    verify_guarded_actuation, minimum_risk_brake_at_tick,
)


def message(*, accel=-2., steer=0., proof=False, mode='MINIMUM_RISK_BRAKE'):
    t=SimpleNamespace(sec=20,nanosec=0)
    u=SimpleNamespace(sec=20,nanosec=80_000_000)
    return SimpleNamespace(stamp=t,expires_ros_stamp=u,run_epoch='epoch',
        source_id='R_B',track_id='iccas2025',map_sha256='a'*64,
        intent_sequence=0,source_state_sequence=100,steering_rad=steer,
        acceleration_mps2=accel,stop_certificate_verified=proof,
        authority_mode=mode,active_reference_id='ref')


def verify(m, speed, now=20_025_000_000, phys=101):
    return verify_guarded_actuation(
        m,now_ros_ns=now,physics_sequence=phys,prior_intent_sequence=99,
        run_epoch='epoch',track_id='iccas2025',map_sha256='a'*64,
        producer='R_B',actual_speed_mps=speed,
        steer_limit_rad=.5,accel_limit_mps2=8.,brake_limit_mps2=9.)


@pytest.mark.parametrize('guard_a,actual_speed',[
    (-2.,1.),(-2.,-0.003),(-2.,0.),(2.,-1.),(2.,.002),(0.,-.5)
])
def test_previous_guard_velocity_direction_does_not_override_local_physics(guard_a,actual_speed):
    okay,reason=verify(message(accel=guard_a),actual_speed)
    assert okay and reason=='MINIMUM_RISK_BRAKE'
    a=minimum_risk_brake_at_tick(actual_speed,9.,.025)
    assert abs(a)<=2.+1e-10
    assert a*actual_speed<=0.
    assert abs(actual_speed+a*.025)<=abs(actual_speed)+1e-10
    assert actual_speed*(actual_speed+a*.025)>=-1e-10


@pytest.mark.parametrize('speed',[-20.,-2.,-0.001,0.,0.001,2.,20.])
def test_local_recomputed_brake_is_nonpropulsive_and_no_zero_crossing(speed):
    a=minimum_risk_brake_at_tick(speed,3.,.025)
    assert a*speed<=0.
    assert abs(a)<=2.
    assert speed*(speed+a*.025)>=-1e-10


@pytest.mark.parametrize('accel,steer,proof',[
    (3.,0.,False),(-9.,0.,False),(-2.,.0001,False),
    (-2.,0.,True),(math.nan,0.,False),(math.inf,0.,False)
])
def test_emergency_malformed_or_certification_spoof_rejected(accel,steer,proof):
    okay,_=verify(message(accel=accel,steer=steer,proof=proof),.2)
    assert not okay


@pytest.mark.parametrize('now,phys',[(20_090_000_000,101),(20_025_000_000,106)])
def test_expired_or_too_old_emergency_never_refreshes_lease(now,phys):
    okay,_=verify(message(),1.,now,phys)
    assert not okay


def test_uncertified_brake_cannot_promote_itself_to_certified():
    ok,reason=verify(message(mode='CERTIFIED',accel=-2.),1.)
    assert not ok and reason=='GUARD_PROOF_OR_REPLAY'


@pytest.mark.parametrize('speed,dt,lim',[(math.nan,.025,2.),(1.,0.,2.),(1.,.025,0.),
                                    (math.inf,.025,2.)])
def test_poisoned_local_physics_never_produces_brake(speed,dt,lim):
    with pytest.raises(ValueError):
        minimum_risk_brake_at_tick(speed,lim,dt)
