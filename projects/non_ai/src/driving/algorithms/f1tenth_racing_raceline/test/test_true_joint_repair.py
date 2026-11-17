"""R-A3 feasibility-first and true joint QP regression (offline only)."""
import math
from pathlib import Path

import numpy as np
import pytest
from scipy import sparse
from ament_index_python.packages import get_package_share_directory

from f1tenth_racing_raceline.scp_adapter import (
    load_scp_problem, solve_scp_problem, write_scp_candidate)
from f1tenth_racing_raceline.sparse_joint import (
    candidate_speed_profile, jerk_feasible_initial_profile,
    sparse_linear_constraints)
from f1tenth_racing_raceline.speed_adapter import (
    solve_speed_problem, validate_dense)
from f1tenth_racing_raceline.trajectory import (
    load_racing_trajectory, validate_racing_trajectory)


def _problem():
    share = Path(get_package_share_directory("f1tenth_racing_raceline"))
    return load_scp_problem(config_path=share / "config/r_a3_scp_true_joint_experimental.json")


def test_qp_contains_three_neighbor_lateral_rows_and_periodic_jerk_rows():
    n = 6
    offsets = np.zeros(n)
    speeds2 = np.full(n, 4.0)
    curvature = np.array([0.1, 0.4, 0.1, 0.1, 0.1, 0.1])
    lengths = np.ones(n)
    zeros = sparse.csr_matrix((n, n))
    a, b = sparse_linear_constraints(offsets, speeds2, curvature, lengths,
                                     zeros, zeros, 1.0, 3.0, 4.0, jerk_limit=25.0)
    assert a.shape == (10 * n, 2 * n)
    # Own-point speed*curvature passes at point 0, yet neighbor 1 violates.
    residual = a @ np.r_[offsets, speeds2] - b
    assert np.any(residual[:6] > 0.5)
    assert np.all(np.isfinite(a.data)) and np.all(np.isfinite(b))


def test_phase_i_creates_jerk_qualified_initial_speed_without_modifying_ra1():
    problem = _problem()
    raw = solve_speed_problem(problem["speed_problem"])
    limit = problem["speed_problem"]["limits"]["jerk_m_s3"]
    with pytest.raises(ValueError, match="jerk"):
        candidate_speed_profile(raw, np.square(raw.target_speed), jerk_limit=limit)
    safe, info = jerk_feasible_initial_profile(raw, jerk_limit=limit,
                dense_samples=problem["speed_problem"]["optimizer_data"]["dense_samples_per_segment"])
    assert 0.0 <= info["phase_i_alpha"] < 1.0
    assert safe.predicted_lap_time >= raw.predicted_lap_time
    assert validate_dense(safe, safe.station_period, 20)["maximum_lateral_acceleration_excess_m_s2"] <= 1e-9
    # Source R-A1 output object is untouched.
    assert raw.target_speed != safe.target_speed


@pytest.fixture(scope="module")
def solved():
    candidate, outcome = solve_scp_problem(_problem())
    return candidate, outcome


def test_qp_joint_steps_and_feasible_lap_time_improvement(solved):
    trajectory, result = solved
    assert result.accepted_steps >= 1
    assert result.diagnostics["joint_qp_accepted_steps"] == result.accepted_steps
    assert result.final_predicted_time_s < result.initial_predicted_time_s
    assert result.diagnostics["phase_i"]["raw_envelope_predicted_time_s"] < result.initial_predicted_time_s
    assert all(sample.target_speed_mps > 0.0 for sample in trajectory.samples)
    assert trajectory.metadata["qualification_state"] == "OFFLINE_EXPERIMENTAL_UNQUALIFIED"
    assert trajectory.metadata["runtime_tracking_qualified"] is False
    assert trajectory.metadata["scp"]["physical_station_dense_validation"]["maximum_lateral_acceleration_excess_m_s2"] <= 1e-9


def test_joint_output_has_physical_stations_and_roundtrips(solved, tmp_path):
    trajectory, result = solved
    stations = [v.s_m for v in trajectory.samples]
    lengths = [v.ds_to_next_m for v in trajectory.samples]
    assert stations[0] == 0.0
    assert all(b > a for a, b in zip(stations, stations[1:]))
    assert np.allclose(stations[1:], np.cumsum(lengths)[:-1], atol=1e-9, rtol=0)
    assert math.isclose(trajectory.station_period_m, sum(lengths), abs_tol=1e-9)
    paths = write_scp_candidate(tmp_path, trajectory)
    reread = load_racing_trajectory(paths[0], paths[1])
    assert reread.trajectory_id == paths[2].trajectory_id
    validate_racing_trajectory(reread, paths[0])
