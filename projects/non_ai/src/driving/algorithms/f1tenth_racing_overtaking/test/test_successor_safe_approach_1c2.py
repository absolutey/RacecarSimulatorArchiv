"""1-C2: conservative approach is enabled only by qualified R-D occupancy."""
from dataclasses import replace
import pytest
from test_successor_q0 import fixture, prediction


def configured():
    planner, nominal = fixture()
    planner.config = replace(planner.config, safe_approach_max_closing_speed_mps=.75,
        longitudinal_acceleration_max_mps2=40., longitudinal_deceleration_max_mps2=40.)
    return planner, nominal


def test_disabled_preserves_legacy_follow_speed():
    p, nominal = fixture()
    pred = prediction(p, ahead=5., red_speed=2.)
    result = p._follow(nominal, 0., pred, 10., "UNIT", blue_speed=2.)
    assert result.plan.state == "BLOCKING_HOLD"
    assert max(x.target_speed_mps for x in result.plan.samples) == pytest.approx(2.)


def test_qualified_wide_gap_allows_bounded_approach_but_no_pass():
    p, nominal = configured()
    pred = prediction(p, ahead=5., red_speed=2.)
    result = p._follow(nominal, 0., pred, 10., "UNIT", blue_speed=2.)
    assert result.plan.state == "BLOCKING_HOLD"
    assert result.plan.side == "NONE"
    assert 2. < max(x.target_speed_mps for x in result.plan.samples) <= 2.75
    assert "APPROACH_OCCUPANCY_GATED" in result.plan.reason


def test_close_gap_rejects_unsafe_approach():
    p, nominal = configured()
    pred = prediction(p, ahead=.8, red_speed=2.)
    result = p._follow(nominal, 0., pred, 10., "UNIT", blue_speed=2.)
    assert result.plan.state == "BLOCKING_HOLD"
    assert max(x.target_speed_mps for x in result.plan.samples) == pytest.approx(2.)


def test_large_r_d_occupancy_never_discarded():
    p, nominal = configured()
    pred = prediction(p, ahead=5., red_speed=2.)
    wide=replace(pred, samples=tuple(replace(x, occupancy_half_length_m=10.) for x in pred.samples))
    result=p._follow(nominal, 0., wide, 10., "UNIT", blue_speed=2.)
    assert max(x.target_speed_mps for x in result.plan.samples) == pytest.approx(2.)


def test_speed_cap_and_config_validation():
    p, nominal = configured()
    pred=prediction(p,ahead=5.,red_speed=4.8)
    target,_=p._safe_approach_target(nominal,0.,4.8,pred,4.8)
    assert target <= p.config.execution_speed_cap_mps
    with pytest.raises(ValueError):
        replace(p.config, safe_approach_max_closing_speed_mps=-.1)
