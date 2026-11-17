import numpy as np
import pytest

from f1tenth_racing_raceline.scp import ScpConfig, solve_convex_qp


def config(**changes):
    values = {
        "maximum_scp_iterations": 2,
        "maximum_admm_iterations": 2000,
        "admm_penalty": 1.0,
        "admm_tolerance": 1e-6,
        "finite_difference_step_m": 1e-4,
        "initial_offset_trust_region_m": 0.1,
        "minimum_offset_trust_region_m": 0.01,
        "offset_proximal_weight": 1.0,
        "speed_squared_proximal_weight": 1.0,
        "offset_smoothness_weight": 0.1,
        "minimum_time_improvement_s": 1e-8,
    }
    values.update(changes)
    return ScpConfig(**values)


def test_specialized_convex_solver_enforces_inequality_and_bounds():
    # min (x-2)^2 / 2 subject to x <= 1 and -3 <= x <= 3.
    solution, iterations = solve_convex_qp(
        np.array([[1.0]]), np.array([-2.0]),
        (np.array([[1.0]]), np.array([1.0])),
        np.array([-3.0]), np.array([3.0]), config(), np.array([0.0]),
    )
    assert iterations < 2000
    assert solution[0] == pytest.approx(1.0, abs=1e-5)


def test_scp_configuration_rejects_nonpositive_values():
    with pytest.raises(ValueError):
        config(admm_penalty=0.0)
