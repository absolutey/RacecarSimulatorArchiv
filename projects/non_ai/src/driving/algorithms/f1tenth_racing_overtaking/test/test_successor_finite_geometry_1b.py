"""Surgery 1-B: finite active pass profile and conservative side bounds."""
from dataclasses import replace
from types import SimpleNamespace
import math
import pytest

from f1tenth_racing_overtaking.successor_core import (
    Candidate, CommittedSpatiotemporalFrenetPlanner, SuccessorConfig,
    Topology, Trackability, TrajectorySample,
)


def fixture():
    n = 120
    radius = 10.0
    points = tuple((radius * math.cos(2*math.pi*i/n),
                    radius * math.sin(2*math.pi*i/n)) for i in range(n))
    ds = 2*radius*math.sin(math.pi/n)
    samples = tuple(TrajectorySample(
        i, i*ds, x, y, (2*math.pi*i/n)+math.pi/2, 1/radius,
        0., 5., 0., ds) for i,(x,y) in enumerate(points))
    c = SuccessorConfig(
        prediction_max_age_s=.5, minimum_track_confidence=.6,
        minimum_confirmed_hits=3, maximum_source_misses=4,
        maximum_covariance_eigenvalue_m2=.2, eligibility_ahead_min_m=.8,
        eligibility_ahead_max_m=6., candidate_lateral_offset_m=.75,
        entry_length_early_m=5., entry_length_late_m=7.5,
        rejoin_lengths_m=(5.,7.5,10.), corridor_clearance_m=.08,
        blue_tracking_uncertainty_m=.1, qualification_margin_m=.05,
        pass_completion_ahead_m=.8, pass_timeout_s=1., delta_v_pass_mps=.25,
        footprint_half_length_m=.2, footprint_half_width_m=.1,
        wheelbase_m=.34, steering_angle_max_rad=4.,
        steering_rate_max_rad_s=4., lateral_acceleration_max_mps2=30.,
        execution_speed_cap_mps=5., longitudinal_acceleration_max_mps2=1.,
        longitudinal_deceleration_max_mps2=2.)
    planner = CommittedSpatiotemporalFrenetPlanner(
        c, points, [2.]*n, [2.]*n, 'test', 'map', 'test-hash')
    return planner, samples


def test_profile_returns_to_nominal_before_other_track_sectors():
    p, samples = fixture()
    pre, hold, merge = p._finite_pass_window(5.)
    assert 5. < hold < hold + merge < p.geometry.length-pre
    def d(at):
        return p._finite_pass_d(
            SimpleNamespace(s_m=p.geometry.wrap_s(at),lateral_offset_m=0.),
            0.,0.,0.,-.75,5.,hold,merge,0.,0.)
    assert d(0.) == pytest.approx(0.)
    assert d(5.) == pytest.approx(-.75)
    assert d(hold-.01) == pytest.approx(-.75)
    assert d(hold+merge) == pytest.approx(0.)
    assert d(hold+merge+1.) == pytest.approx(0.)
    assert d(-pre) == pytest.approx(0.)


def test_narrow_right_sector_outside_window_does_not_force_full_lap_offset():
    p, samples = fixture()
    widths=list(p.right_widths)
    widths[65]=.4  # approximately the opposite half of the circular loop
    p.right_widths=tuple(widths)
    pre, hold, merge=p._finite_pass_window(5.)
    assert p._side_target_d(samples, 0., hold, 'RIGHT') == pytest.approx(-.75)
    # At the narrow sector the generated path must have become nominal again.
    d=p._finite_pass_d(samples[65],0.,0.,0.,-.75,5.,hold,merge,0.,0.)
    assert d == pytest.approx(samples[65].lateral_offset_m)
    yaw=p.geometry.interpolate(0.).heading
    made, clearance, _, reason=p._candidate_geometry(samples,0.,0.,yaw,'RIGHT',5.)
    assert made is not None, (reason, clearance)
    assert made[65].lateral_offset_m == pytest.approx(0.,abs=.01)
    assert clearance >= p.config.corridor_clearance_m


def test_active_side_width_limits_target_and_retains_safety_gate():
    p, samples=fixture()
    widths=list(p.right_widths)
    widths[8]=.55
    p.right_widths=tuple(widths)
    _, hold, _=p._finite_pass_window(5.)
    assert p._side_target_d(samples,0.,hold,'RIGHT') == pytest.approx(-.27)
    widths[8]=.30
    p.right_widths=tuple(widths)
    assert p._side_target_d(samples,0.,hold,'RIGHT') is None
    yaw=p.geometry.interpolate(0.).heading
    made,_,_,reason=p._candidate_geometry(samples,0.,0.,yaw,'RIGHT',5.)
    assert made is None and reason=='PASS_LATERAL_SPACE_INSUFFICIENT'


def test_finite_window_rejects_track_that_cannot_hold_and_reconnect():
    p, samples=fixture()
    p.config=replace(p.config,pass_timeout_s=20.)
    yaw=p.geometry.interpolate(0.).heading
    made,_,_,reason=p._candidate_geometry(samples,0.,0.,yaw,'LEFT',5.)
    assert made is None and reason=='FINITE_PASS_WINDOW_TOO_LONG'


def test_commit_does_not_forge_verified_merge_station():
    p, samples=fixture()
    candidate=Candidate(Topology.LEFT_EARLY,'LEFT',5.,samples,.8,3.,1.,.3,3.,1.,
        Trackability(0.,0.,0.,0.,0.))
    prediction=SimpleNamespace(identity='p',source_revision=1,
        source_time_s=10.,source_time_ns=int(10e9))
    result=p.commit_candidate(candidate,prediction,0.,10.)
    assert result.plan.pass_end_s==pytest.approx(3.)
    _,hold,merge=p._finite_pass_window(candidate.entry_length_m)
    assert p.geometry.forward_delta_s(0.,result.plan.merge_end_s)==pytest.approx(hold+merge)
    assert result.plan.merge_end_s != result.plan.pass_end_s
    assert result.plan.state=='COMMITTED_PASS'  # MERGE requires fresh confirmation
