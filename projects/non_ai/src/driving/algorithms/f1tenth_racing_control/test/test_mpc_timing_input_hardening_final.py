"""MPC speed and warm-start fail-closed regression gates."""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from f1tenth_racing_control.mpc import ExecutionSpeedEnvelope, LtvDynamicBicycleMpc
from pathlib import Path
import math
import yaml
from f1tenth_racing_control import PursuitReference
from f1tenth_racing_control.mpc import MpcConfig


def reference():
    pts=((0.,0.),(4.,0.),(4.,2.),(0.,2.))
    lens=[math.dist(pts[i],pts[(i+1)%4]) for i in range(4)]
    ss=[0.,lens[0],lens[0]+lens[1],lens[0]+lens[1]+lens[2]]
    samples=tuple(SimpleNamespace(sample_index=i,s_m=ss[i],x_m=p[0],y_m=p[1],
        heading_rad=math.atan2(pts[(i+1)%4][1]-p[1],pts[(i+1)%4][0]-p[0]),
        curvature_per_m=0.,lateral_offset_m=0.,target_speed_mps=2.,
        target_accel_mps2=0.,ds_to_next_m=lens[i]) for i,p in enumerate(pts))
    return PursuitReference(samples,sum(lens))


def configuration(**kw):
    args=dict(horizon_steps=4,timestep_s=.025,maximum_solver_iterations=200,
        solver_penalty=1.,solver_tolerance=.001,primary_deadline_s=1.,
        lateral_error_weight=30.,longitudinal_error_weight=1.,
        heading_error_weight=12.,speed_error_weight=4.,slip_weight=1.,
        yaw_rate_weight=.5,steering_state_weight=.2,acceleration_state_weight=.05,
        steering_input_weight=.05,acceleration_input_weight=.01,steering_rate_weight=.5,
        acceleration_rate_weight=.02,preview_distance_m=3.,preview_step_m=.25,
        preview_braking_mps2=8.,lateral_acceleration_limit_mps2=12.,
        lateral_error_limit_m=.28,slip_angle_limit_rad=.35,
        fallback_recoverable_lateral_error_m=.55,
        fallback_recoverable_heading_error_rad=.85)
    args.update(kw)
    return MpcConfig(**args)


@pytest.fixture
def foundation():
    config=Path(__file__).resolve().parents[4]/'environment/f1tenth_racing_sim/config/competition_environment.yaml'
    dynamics=yaml.safe_load(config.read_text())['dynamics']
    return dynamics,dynamics['limits']


def state():
    return SimpleNamespace(px=1.,py=.1,yaw=0.,v=2.,slip_angle=0.,omega=0.,steer=0.,accel=0.)

@pytest.mark.parametrize('value',[float('nan'),float('inf'),-1.0,0.0])
def test_speed_authority_invalid_does_not_enter_envelope(value):
    with pytest.raises(ValueError,match='MPC_ENVELOPE_PARAMETERS_INVALID'):
        ExecutionSpeedEnvelope(reference(),value,12.,8.,4.,3.)

@pytest.mark.parametrize('field,value',[
    ('target_speed_mps',float('nan')),('target_speed_mps',float('inf')),
    ('target_speed_mps',-0.01),('curvature_per_m',float('nan')),
    ('ds_to_next_m',0.),('ds_to_next_m',float('inf')),
    ('s_m',float('nan'))])
def test_poisoned_reference_rejected_before_sqrt(field,value):
    ref=reference()
    bad=SimpleNamespace(**{**ref.samples[0].__dict__,field:value})
    ref.samples=(bad,)+ref.samples[1:]
    with pytest.raises(ValueError,match='MPC_REFERENCE_NONFINITE_OR_INVALID'):
        ExecutionSpeedEnvelope(ref,5.,12.,8.,4.,3.)

def test_late_primary_cannot_reseed_warm_start(foundation):
    dynamics,limits=foundation
    mpc=LtvDynamicBicycleMpc(dynamics,limits,configuration(primary_deadline_s=1e-12))
    original=mpc.warm_start.copy()
    result=mpc.solve(reference(),state(),1.0,(0.,0.),2.)
    assert result.status=='PRIMARY_LATE'
    assert np.array_equal(mpc.warm_start,original)
