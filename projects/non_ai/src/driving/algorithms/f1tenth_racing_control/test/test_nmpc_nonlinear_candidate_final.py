"""Nonlinear B2 candidate numerical and safety gates; no realtime claim."""
import math
import time
import pytest
from f1tenth_racing_control.mpc import NonlinearDynamicBicycleMpc, MpcCycleBudgetExceeded
from test_mpc_timing_input_hardening_final import reference, configuration, state, foundation

def test_b2_nonlinear_rollout_solution_obeys_hard_constraints(foundation):
    dyn,limits=foundation
    solver=NonlinearDynamicBicycleMpc(dyn,limits,configuration(horizon_steps=3,maximum_solver_iterations=40,primary_deadline_s=2.))
    r=solver.solve(reference(),state(),1.,(0.,0.),2.,deadline_ns=time.perf_counter_ns()+1_500_000_000)
    assert r.status=='PRIMARY_VALID'
    assert r.max_constraint_residual<=solver.config.solver_tolerance
    assert abs(r.steering_rad)<=limits['steering_rate_abs_max_rad_s']*.025+1e-5
    assert abs(r.acceleration_mps2)<=limits['jerk_abs_max_m_s3']*.025+1e-5
    assert all(math.isfinite(v) for v in solver.warm_start)

@pytest.mark.parametrize('bad',[float('nan'),float('inf'),0.,-1.])
def test_invalid_nmpc_speed_never_enters_optimization(foundation,bad):
    dyn,limits=foundation
    solver=NonlinearDynamicBicycleMpc(dyn,limits,configuration(horizon_steps=3))
    with pytest.raises(ValueError,match='NMPC_SPEED_AUTHORITY_INVALID'):
        solver.solve(reference(),state(),1.,(0.,0.),bad)

def test_expired_nmpc_request_cannot_change_warm_start(foundation):
    dyn,limits=foundation
    solver=NonlinearDynamicBicycleMpc(dyn,limits,configuration(horizon_steps=3))
    original=solver.warm_start.copy()
    with pytest.raises(MpcCycleBudgetExceeded):
        solver.solve(reference(),state(),1.,(0.,0.),2.,deadline_ns=time.perf_counter_ns()-1)
    assert all(solver.warm_start==original)
