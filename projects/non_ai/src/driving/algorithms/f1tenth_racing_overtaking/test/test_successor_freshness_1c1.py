"""Surgery 1C-1: validate predictive provenance and bounded frozen loss."""
from dataclasses import replace
import pytest
from f1tenth_racing_overtaking.successor_core import InternalState, WireState
from test_successor_q0 import fixture, prediction, manual_candidate, state


def test_declared_horizon_must_cover_pass_timeout():
    p, _ = fixture()
    pred = prediction(p)
    assert p._prediction_status(replace(pred, horizon_s=.8), 10.) == (
        'PREDICTION_HORIZON_SHORTER_THAN_PASS_TIMEOUT')


def test_prediction_samples_cannot_exceed_advertised_horizon():
    p, _ = fixture()
    p.config = replace(p.config, pass_timeout_s=.5)
    pred = prediction(p)
    assert p._prediction_status(replace(pred, horizon_s=.8), 10.) == (
        'PREDICTION_SAMPLE_EXCEEDS_DECLARED_HORIZON')


def test_prediction_occupancy_and_covariance_fail_closed():
    p, _ = fixture()
    pred = prediction(p)
    for mutated in (
        replace(pred.samples[0], occupancy_half_width_m=float('nan')),
        replace(pred.samples[0], occupancy_half_length_m=-.01),
        replace(pred.samples[0], covariance_xy=(.01,.05,.05,.01)),
    ):
        new = replace(pred, samples=(mutated,)+pred.samples[1:])
        assert p._prediction_status(new, 10.) == (
            'PREDICTION_OCCUPANCY_OR_COVARIANCE_INVALID')


def test_frozen_pass_tolerates_brief_loss_but_aborts_long_loss_and_preserves_identity():
    p, nominal = fixture()
    pred = prediction(p)
    committed = p.commit_candidate(manual_candidate(p, nominal), pred, 0., 10.)
    initial_digest = committed.plan.identity_digest
    pose = state(p,.2,.02)
    brief = p.update(nominal,pose.x,pose.y,5.,None,10.1,pose.heading)
    assert brief.action == 'CONTINUE_FROZEN'
    assert brief.plan.identity_digest == initial_digest
    long = p.update(nominal,pose.x,pose.y,5.,None,10.501,pose.heading)
    assert long.action == 'PUBLISH_ABORT_DEGRADE'
    assert long.plan.state == WireState.ABORT.value
    assert 'FROZEN_PASS_PREDICTION_LOST:' in long.plan.reason
    assert long.plan.identity_digest == initial_digest


def test_frozen_merge_tolerates_brief_loss_but_aborts_long_loss():
    p, nominal = fixture()
    pred = prediction(p)
    committed = p.commit_candidate(manual_candidate(p, nominal), pred, 0., 10.)
    p.plan = replace(committed.plan, state=WireState.MERGE.value)
    p.internal_state = InternalState.FROZEN_REJOIN
    pose = state(p,.2,.02)
    brief = p.update(nominal,pose.x,pose.y,5.,None,10.2,pose.heading)
    assert brief.action == 'CONTINUE_FROZEN'
    lost = p.update(nominal,pose.x,pose.y,5.,None,10.501,pose.heading)
    assert lost.action == 'PUBLISH_ABORT_DEGRADE'
    assert lost.plan.state == WireState.ABORT.value
    assert 'FROZEN_REJOIN_PREDICTION_LOST:' in lost.plan.reason
    assert lost.plan.identity_digest == committed.plan.identity_digest


def test_fresh_prediction_renews_frozen_safety_heartbeat(monkeypatch):
    p, nominal = fixture()
    pred = prediction(p)
    committed = p.commit_candidate(manual_candidate(p, nominal), pred, 0., 10.)
    pose = state(p,.2,.02)
    monkeypatch.setattr(p, '_fresh_frozen_safety', lambda *args,**kwargs: (True,1.))
    # Qualified updated R-D observation; no false longitudinal pass confirmation.
    newer = prediction(p, blue_s=.2, ahead=2., now_s=10.3)
    event = p.update(nominal,pose.x,pose.y,5.,newer,10.3,pose.heading)
    assert event.action == 'CONTINUE_FROZEN'
    assert p.last_qualified_prediction_at_s == pytest.approx(10.3)
    still = p.update(nominal,pose.x,pose.y,5.,None,10.7,pose.heading)
    assert still.action == 'CONTINUE_FROZEN'
    timeout = p.update(nominal,pose.x,pose.y,5.,None,10.81,pose.heading)
    assert timeout.action == 'PUBLISH_ABORT_DEGRADE'
