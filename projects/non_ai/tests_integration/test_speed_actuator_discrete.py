from types import SimpleNamespace as N
import math
import pytest
from f1tenth_racing_control.execution_speed_authority import derive_closed_loop_execution_speed

def run(steer=0.03,jerk=1.0,desired=18.):
    # closed track with a sharp steering transition at two stations
    k=[0,0,0.8,0.8,0,0]
    source=tuple(N(s_m=i*1.,curvature_per_m=k[i],target_speed_mps=desired,
                   target_accel_mps2=0,ds_to_next_m=1.) for i in range(6))
    return derive_closed_loop_execution_speed(source,6.,vehicle_speed_bound_mps=20.,
        braking_mps2=3.,acceleration_mps2=4.,proof_horizon_s=8.,
        reaction_s=.25,lateral_accel_bound_mps2=6.,physics_step_s=.025,
        wheelbase_m=0.32,steering_rate_limit_rad_s=steer,jerk_limit_mps3=jerk)

def test_discrete_actuator_constraint_lowers_without_fixed_five():
    output,audit=run()
    assert audit.planned_peak_mps<18.
    assert audit.qualified is False
    assert 'STEERING_JERK_DISCRETE' in audit.limit_reason
    assert min(x.target_speed_mps for x in output)>0

def test_tighter_steer_rate_monotonic_lowers():
    fast,_=run(steer=.5,jerk=1e5)
    slow,_=run(steer=.02,jerk=1e5)
    assert max(x.target_speed_mps for x in slow) < max(x.target_speed_mps for x in fast)

def test_tighter_jerk_limit_monotonic_lowers():
    fast,_=run(steer=100.,jerk=1e5)
    slow,_=run(steer=100.,jerk=.05)
    assert max(x.target_speed_mps for x in slow) <= max(x.target_speed_mps for x in fast)

def test_partial_actuator_contract_rejected():
    with pytest.raises(ValueError,match='INCOMPLETE_ACTUATOR'):
        run(steer=None)
