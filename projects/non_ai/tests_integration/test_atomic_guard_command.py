"""Atomic Guard -> Ego simulator command safety contract: pure non-ROS tests."""
from dataclasses import replace
from types import SimpleNamespace as N
import pytest
from f1tenth_racing_sim.guarded_actuation_contract import verify_guarded_actuation


def command(**kw):
    d=dict(stamp=N(sec=12,nanosec=0),expires_ros_stamp=N(sec=12,nanosec=90_000_000),
        run_epoch='runA',source_id='R_B',intent_sequence=7,source_state_sequence=50,
        active_reference_id='r_a-1',active_revision=2,track_id='iccas2025',map_sha256='d'*64,
        authority_mode='CERTIFIED',stop_certificate_verified=True,
        steering_rad=0.02,acceleration_mps2=2.0)
    d.update(kw)
    return N(**d)


def verify(c, **kw):
    d=dict(now_ros_ns=12_020_000_000,physics_sequence=50,prior_intent_sequence=6,
        run_epoch='runA',track_id='iccas2025',map_sha256='d'*64,
        producer='R_B',actual_speed_mps=3.0,
        steer_limit_rad=0.4,accel_limit_mps2=8.0,brake_limit_mps2=5.0)
    d.update(kw)
    return verify_guarded_actuation(c, **d)


def test_valid_certified_command_binds_identity():
    assert verify(command())==(True,'CERTIFIED')

@pytest.mark.parametrize('field,value', [('run_epoch','runB'),('source_id','RED'),
    ('track_id','spielberg'),('map_sha256','0'*64)])
def test_identity_mismatch_rejected(field,value):
    assert not verify(command(**{field:value}))[0]

@pytest.mark.parametrize('kw', [dict(stop_certificate_verified=False),
    dict(intent_sequence=6),dict(intent_sequence=0),dict(active_reference_id=''),
    dict(source_state_sequence=51),dict(source_state_sequence=45),
    dict(acceleration_mps2=float('nan')),dict(acceleration_mps2=9.),
    dict(steering_rad=.6),dict(authority_mode='UNTRUSTED')])
def test_invalid_proof_replay_state_bounds_or_mode_rejected(kw):
    assert not verify(command(**kw))[0]

def test_time_lease_fails_if_expired():
    assert not verify(command(), now_ros_ns=12_200_000_000)[0]

def test_future_stamp_fails():
    assert not verify(command(stamp=N(sec=12,nanosec=50_000_000)))[0]

def test_emergency_brake_is_separate_from_certificate():
    c=command(authority_mode='MINIMUM_RISK_BRAKE',stop_certificate_verified=False,
              steering_rad=0.,acceleration_mps2=-2.)
    assert verify(c)==(True,'MINIMUM_RISK_BRAKE')
    assert not verify(command(authority_mode='MINIMUM_RISK_BRAKE',stop_certificate_verified=True,
                              steering_rad=0.,acceleration_mps2=-2.))[0]
    assert not verify(command(authority_mode='MINIMUM_RISK_BRAKE',stop_certificate_verified=False,
                              steering_rad=.01,acceleration_mps2=-2.))[0]
    assert not verify(c,actual_speed_mps=-1.)[0]
    assert not verify(command(authority_mode='MINIMUM_RISK_BRAKE',
        stop_certificate_verified=False,steering_rad=0.,acceleration_mps2=0.))[0]


def test_raw_ackermann_does_not_directly_actuate_ego_source_contract():
    from pathlib import Path
    p=Path(__file__).resolve().parents[1]/'src/environment/f1tenth_racing_sim/f1tenth_racing_sim/simulator.py'
    source=p.read_text()
    assert "'guarded_actuator_cmd0'" in source
    assert 'if self.guard_required:' in source
    assert "'EGO_RAW_COMMAND_FORBIDDEN'" in source
