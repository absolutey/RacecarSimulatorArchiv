"""MPC wall-clock cutoff: never accepts non-converged output under time pressure."""
import time
import numpy as np
import pytest
from types import SimpleNamespace
from f1tenth_racing_control.mpc import _qp,MpcCycleBudgetExceeded

def qp_args():
    h=np.eye(2)*2
    g=np.array([-1.0,-0.5])
    a=np.eye(2)
    b=np.array([1.0,1.0])
    low=np.array([-1.0,-1.0]);high=np.array([1.0,1.0])
    initial=np.zeros(2)
    cfg=SimpleNamespace(solver_penalty=1.0, maximum_solver_iterations=500,
                        solver_tolerance=1e-3)
    return h,g,a,b,low,high,initial,cfg

def test_unbounded_legacy_qp_math_unchanged():
    x,it=_qp(*qp_args())
    y,j=_qp(*qp_args(),deadline_ns=None)
    assert it==j and np.array_equal(x,y)

def test_expired_deadline_fails_without_candidate():
    with pytest.raises(MpcCycleBudgetExceeded) as ex:
        _qp(*qp_args(), deadline_ns=time.perf_counter_ns()-1)
    assert ex.value.iterations==0
    assert str(ex.value)=="MPC_CYCLE_BUDGET_EXCEEDED"

def test_cutoff_is_not_mislabeled_infeasibility():
    with pytest.raises(MpcCycleBudgetExceeded) as ex:
        _qp(*qp_args(), deadline_ns=time.perf_counter_ns()-1, started_ns=time.perf_counter_ns()-50000)
    assert ex.value.stage=='QP_SETUP'
    assert ex.value.elapsed_ns >= 0

def test_failed_deadline_does_not_return_unqualified_primary():
    with pytest.raises(MpcCycleBudgetExceeded):
        _qp(*qp_args(),deadline_ns=0)
