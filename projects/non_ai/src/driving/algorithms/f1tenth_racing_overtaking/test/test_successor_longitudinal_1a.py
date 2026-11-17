"""Surgery 1-A regression: live ego clock vs legacy target-speed clock."""
from dataclasses import replace
from types import SimpleNamespace
import math

import pytest
from f1tenth_racing_overtaking.successor_core import (
    CommittedSpatiotemporalFrenetPlanner, SuccessorConfig, TrajectorySample,
)


def planner_fixture():
    n = 120
    r = 10.0
    points = tuple((r * math.cos(2*math.pi*i/n), r * math.sin(2*math.pi*i/n)) for i in range(n))
    ds = 2*r*math.sin(math.pi/n)
    samples = tuple(TrajectorySample(i, ds*i, x, y, 2*math.pi*i/n+math.pi/2,
        1/r, 0.0, 5.0, 0.0, ds) for i,(x,y) in enumerate(points))
    config = SuccessorConfig(
        prediction_max_age_s=.5, minimum_track_confidence=.6,
        minimum_confirmed_hits=3, maximum_source_misses=4,
        maximum_covariance_eigenvalue_m2=.2, eligibility_ahead_min_m=.8,
        eligibility_ahead_max_m=6., candidate_lateral_offset_m=.75,
        entry_length_early_m=5., entry_length_late_m=7.5,
        rejoin_lengths_m=(5., 7.5, 10.), corridor_clearance_m=.08,
        blue_tracking_uncertainty_m=.1, qualification_margin_m=.05,
        pass_completion_ahead_m=.8, pass_timeout_s=1., delta_v_pass_mps=.25,
        footprint_half_length_m=.2, footprint_half_width_m=.1,
        wheelbase_m=.34, steering_angle_max_rad=4., steering_rate_max_rad_s=4.,
        lateral_acceleration_max_mps2=30., execution_speed_cap_mps=5.,
        longitudinal_acceleration_max_mps2=1.,
        longitudinal_deceleration_max_mps2=2.)
    planner = CommittedSpatiotemporalFrenetPlanner(
        config, points, [2.]*n, [2.]*n, 'test','map','test-hash')
    return planner, samples


def test_live_clock_does_not_teleport_from_two_to_five():
    p,s=planner_fixture()
    st=p.state_at_time(s,0.,1.,initial_speed_mps=2.)
    assert p.geometry.forward_delta_s(0.,st['s_m'])==pytest.approx(2.5,abs=.04)
    assert st['speed_mps']==pytest.approx(3.,abs=.001)
    target_clock=p.state_at_time(s,0.,1.)
    assert p.geometry.forward_delta_s(0.,target_clock['s_m'])==pytest.approx(5.,abs=.04)


def test_actual_deceleration_is_bounded():
    p,s=planner_fixture()
    s=tuple(replace(item,target_speed_mps=2.) for item in s)
    st=p.state_at_time(s,0.,1.,initial_speed_mps=5.)
    assert p.geometry.forward_delta_s(0.,st['s_m'])==pytest.approx(4.,abs=.04)
    assert st['speed_mps']==pytest.approx(3.,abs=.001)


def test_impossible_global_upper_bound_rejected_even_if_nominal_speed_five():
    p,_=planner_fixture()
    prediction=SimpleNamespace(samples=tuple(SimpleNamespace(
        dt_s=t,s_m=8.+2*t,vx_mps=2.,vy_mps=0.) for t in (.1,.5,1.)))
    assert not p._upper_bound_can_complete(0.,1.96,prediction)


def test_possible_upper_bound_is_not_a_pass_approval():
    p,_=planner_fixture()
    p.config=replace(p.config,longitudinal_acceleration_max_mps2=10.)
    prediction=SimpleNamespace(samples=(SimpleNamespace(
        dt_s=1.,s_m=2.,vx_mps=.5,vy_mps=0.),))
    assert p._upper_bound_can_complete(0.,2.,prediction)
    assert p.plan is None  # feasibility and commitment have not occurred


def test_unqualified_longitudinal_dynamics_fail_closed():
    p,_=planner_fixture()
    with pytest.raises(ValueError):
        replace(p.config,longitudinal_deceleration_max_mps2=0.)
    assert p.state_at_time((),0.,1.,initial_speed_mps=float('nan')) is None
