"""New MPC process has no direct ROS writer and rejects stale input/results."""
import math
import time
from types import SimpleNamespace as O
from pathlib import Path
import yaml
from f1tenth_racing_control.core import PursuitReference
from f1tenth_racing_control.process_primary_worker import IsolatedMpcProcessWorker


def config():
    return dict(horizon_steps=4,timestep_s=.025,maximum_solver_iterations=200,
        solver_penalty=1.,solver_tolerance=.001,primary_deadline_s=1.,
        lateral_error_weight=30.,longitudinal_error_weight=1.,
        heading_error_weight=12.,speed_error_weight=4.,slip_weight=1.,
        yaw_rate_weight=.5,steering_state_weight=.2,
        acceleration_state_weight=.05,steering_input_weight=.05,
        acceleration_input_weight=.01,steering_rate_weight=.5,
        acceleration_rate_weight=.02,preview_distance_m=3.,preview_step_m=.25,
        preview_braking_mps2=8.,lateral_acceleration_limit_mps2=12.,
        lateral_error_limit_m=.28,slip_angle_limit_rad=.35,
        fallback_recoverable_lateral_error_m=.55,
        fallback_recoverable_heading_error_rad=.85)


def inputs():
    pts=((0.,0.),(4.,0.),(4.,2.),(0.,2.))
    lengths=[math.dist(pts[i],pts[(i+1)%4]) for i in range(4)]
    st=0.;samples=[]
    for i,(x,y) in enumerate(pts):
        after=pts[(i+1)%4]
        samples.append(O(sample_index=i,s_m=st,x_m=x,y_m=y,
            heading_rad=math.atan2(after[1]-y,after[0]-x),
            curvature_per_m=0.,lateral_offset_m=0.,target_speed_mps=2.,
            target_accel_mps2=0.,ds_to_next_m=lengths[i]))
        st+=lengths[i]
    return PursuitReference(tuple(samples),st),O(px=1.,py=.1,yaw=0.,v=2.,
        slip_angle=0.,omega=0.,steer=0.,accel=0.)


def test_real_solver_runs_in_separate_spawned_process():
    root=Path(__file__).resolve().parents[4]
    env=yaml.safe_load((root/'environment/f1tenth_racing_sim/config/competition_environment.yaml').read_text())
    dyn=env['dynamics']
    w=IsolatedMpcProcessWorker(dynamics=dyn,limits=dyn['limits'],
                              mpc_config=config(),formulation='ltv')
    try:
        ref,state=inputs();deadline=time.perf_counter_ns()+3_000_000_000
        assert w.submit_request(11,ref,state,1.,(0.,0.),2.,deadline)
        assert not w.submit_request(11,ref,state,1.,(0.,0.),2.,deadline)
        result=w.take(11,deadline_ns=deadline)
        assert result.status=='PRIMARY_VALID'
        assert w.counters().completed==1
    finally:
        assert w.close()


def test_invalid_formulation_denied_before_process_spawn():
    try:
        IsolatedMpcProcessWorker(dynamics={},limits={},mpc_config={},formulation='mystery')
    except ValueError:
        pass
    else:
        raise AssertionError('invalid formulation not rejected')
