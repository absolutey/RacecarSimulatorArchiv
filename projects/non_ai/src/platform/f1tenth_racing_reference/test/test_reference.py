from copy import deepcopy
from dataclasses import fields, replace
from types import SimpleNamespace

import pytest

from f1tenth_racing_raceline import RacingTrajectory, load_racing_trajectory
from f1tenth_racing_reference import (
    invalid_active_reference,
    select_nominal,
    select_override,
    validate_active_target,
)
from f1tenth_racing_reference.core import (
    matching_complete, overtaking_plan_identity, override_expiry_action, prediction_identity,
)
from f1tenth_racing_reference.transport import to_message
from f1tenth_racing_reference.node import NominalActiveReferenceNode


MAP_HASH = "471a7c09112458358a4e1b8b540622ea4341abf931c5cfc009ed4d9a9af2350d"


def prediction_fixture(stamp, nominal):
    sample = SimpleNamespace(
        dt_s=0.1, x_m=1.0, y_m=2.0, vx_mps=0.5, vy_mps=0.0,
        heading_rad=0.0, s_m=1.0, d_m=0.0,
        covariance_xy=[0.01, 0.0, 0.0, 0.01],
        occupancy_half_length_m=0.3, occupancy_half_width_m=0.3,
    )
    return SimpleNamespace(
        schema_version=1, valid=True, source_observation_time=stamp,
        source_opponent_id="red", source_track_revision=7, predictor="TRACK_CV",
        source_lifecycle="TRACKED", source_confidence=0.9,
        source_hit_count=8, source_miss_count=0, horizon_s=1.0, timestep_s=0.1,
        track_id=nominal.track_id, frame_id=nominal.frame_id, samples=[sample],
    )


def changed(trajectory, **updates):
    metadata = dict(trajectory.metadata)
    metadata.update(updates)
    return RacingTrajectory(trajectory.samples, metadata)


def test_canonical_nominal_is_exact_stable_and_not_runtime_qualified():
    source = load_racing_trajectory()
    first = select_nominal(source)
    second = select_nominal(source)
    assert validate_active_target(first) is True
    assert first.trajectory is source
    assert first.trajectory.samples == source.samples
    assert first.source_kind == "NOMINAL"
    assert first.source_trajectory_id == source.trajectory_id
    assert first.active_reference_id == second.active_reference_id
    assert first.active_revision == second.active_revision == 1
    assert first.source_product_valid is True
    assert first.source_runtime_tracking_qualified is False
    assert first.active_reference_valid is True


@pytest.mark.parametrize(
    "updates,reason",
    [
        ({"product_valid": False}, "valid closed-loop product"),
        ({"track_id": "wrong"}, "track identity"),
        ({"frame_id": "odom"}, "identity/frame"),
        ({"schema_version": 999}, "unsupported"),
        ({"qualification_state": "NOT_QUALIFIED"}, "qualification"),
        ({"source": {}}, "map identity"),
        ({"source": {"map_image_sha256": "0" * 64}}, "map identity mismatch"),
    ],
)
def test_invalid_nominal_is_rejected_without_fabricated_payload(updates, reason):
    active = select_nominal(
        changed(load_racing_trajectory(), **updates),
        expected_map_image_sha256=MAP_HASH,
    )
    assert active.active_reference_valid is False
    assert active.trajectory is None
    assert active.sample_count == 0
    assert reason in active.status_reason
    assert validate_active_target(active) is False
    message = to_message(active)
    assert message.active_reference_valid is False
    assert message.samples == []


def test_transport_preserves_metadata_and_every_sample():
    active = select_nominal(load_racing_trajectory())
    message = to_message(active)
    assert message.active_reference_id == active.active_reference_id
    assert message.active_revision == active.active_revision
    assert message.source_kind == active.source_kind
    assert message.source_trajectory_id == active.source_trajectory_id
    assert message.source_runtime_tracking_qualified is False
    assert message.active_reference_valid is True
    assert message.sample_count == active.sample_count == len(message.samples)
    assert message.station_period_m == active.station_period_m
    assert message.path_length_m == active.path_length_m
    for source, transmitted in zip(active.trajectory.samples, message.samples):
        for field in fields(source):
            assert getattr(transmitted, field.name) == getattr(source, field.name)


def test_offline_primary_candidate_is_eligible_without_runtime_relabeling():
    baseline = load_racing_trajectory()
    candidate = changed(
        baseline,
        qualification_state="OFFLINE_PRIMARY_CANDIDATE_QUALIFIED",
        trajectory_id="sha256:primary",
    )
    active = select_nominal(candidate)
    assert active.active_reference_valid is True
    assert active.source_qualification == "OFFLINE_PRIMARY_CANDIDATE_QUALIFIED"
    assert active.source_runtime_tracking_qualified is False


def test_active_metadata_tampering_is_rejected():
    active = select_nominal(load_racing_trajectory())
    with pytest.raises(ValueError, match="identity mismatch"):
        validate_active_target(replace(active, source_trajectory_id="sha256:wrong"))
    invalid = invalid_active_reference("test")
    with pytest.raises(ValueError, match="must not carry"):
        validate_active_target(replace(invalid, trajectory=active.trajectory))


def test_override_preserves_payload_and_rejects_wrong_map():
    nominal = select_nominal(load_racing_trajectory())
    samples = to_message(nominal).samples
    stamp = SimpleNamespace(sec=10, nanosec=0)
    prediction = prediction_fixture(stamp, nominal)
    plan = SimpleNamespace(
        schema_version=2, valid=True, maneuver_state="COMMITTED_PASS", passing_side="LEFT",
        safety_state="TEMPORAL_SAFE", plan_id="", plan_revision=4,
        source_prediction_id=prediction_identity(prediction), source_track_revision=7,
        source_observation_time=stamp, header=SimpleNamespace(stamp=stamp),
        valid_until=SimpleNamespace(sec=10, nanosec=500_000_000),
        continuation_until=SimpleNamespace(sec=20, nanosec=0),
        status_reason="SAFE_CANDIDATE_COMMITTED", track_id=nominal.track_id,
        frame_id=nominal.frame_id, map_image_sha256=nominal.map_image_sha256,
        sample_count=len(samples), samples=samples, station_period_m=nominal.station_period_m,
        path_length_m=nominal.path_length_m, start_progress_m=0.0,
        commit_end_progress_m=5.0, pass_end_progress_m=25.0, merge_end_progress_m=30.0,
    )
    plan.plan_id = overtaking_plan_identity(plan)
    active = select_override(nominal, plan, prediction, 10.1)
    assert validate_active_target(active) is True
    assert active.source_kind == "OVERTAKE_OVERRIDE"
    assert active.active_revision == 4
    assert to_message(active).samples == samples
    assert not select_override(nominal, plan, None, 10.2).active_reference_valid
    continuation = select_override(nominal, plan, None, 10.2, committed_plan=plan)
    assert continuation.active_reference_valid
    passing = SimpleNamespace(**{**vars(plan), "maneuver_state": "PASSING"})
    assert select_override(nominal, passing, None, 10.2,
                           committed_plan=plan).active_reference_valid
    changed_revision = SimpleNamespace(**{
        **vars(plan), "plan_revision": plan.plan_revision + 1,
    })
    assert not select_override(nominal, changed_revision, None, 10.2,
                               committed_plan=plan).active_reference_valid
    changed_payload = SimpleNamespace(**{**vars(plan), "samples": deepcopy(samples)})
    changed_payload.samples[0].target_speed_mps -= 0.1
    assert not select_override(nominal, changed_payload, None, 10.2,
                               committed_plan=plan).active_reference_valid
    changed_provenance = SimpleNamespace(**{
        **vars(plan), "source_prediction_id": "sha256:forged",
    })
    assert not select_override(nominal, changed_provenance, None, 10.2,
                               committed_plan=plan).active_reference_valid
    rejected = select_override(nominal, SimpleNamespace(**{**vars(plan), "map_image_sha256": "wrong"}), prediction, 10.1)
    assert not rejected.active_reference_valid and rejected.trajectory is None

    abort = SimpleNamespace(**{
        **vars(plan), "maneuver_state": "ABORT", "safety_state": "FAIL_CLOSED",
        "header": SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=600_000_000)),
        "valid_until": SimpleNamespace(sec=11, nanosec=100_000_000),
    })
    abort.plan_id = overtaking_plan_identity(abort)
    continuation = select_override(nominal, abort, None, 10.7, committed_plan=plan)
    assert continuation.active_reference_valid and continuation.source_kind == "OVERTAKE_OVERRIDE"
    assert to_message(continuation).samples == samples
    abort_with_safe_exit = SimpleNamespace(**{**vars(abort), "samples": deepcopy(samples)})
    abort_with_safe_exit.samples[0].target_speed_mps -= 0.1
    abort_with_safe_exit.plan_id = overtaking_plan_identity(abort_with_safe_exit)
    assert select_override(
        nominal, abort_with_safe_exit, None, 10.7, committed_plan=plan,
    ).active_reference_valid

    hold = SimpleNamespace(**{
        **vars(plan), "maneuver_state": "BLOCKING_HOLD", "passing_side": "NONE",
        "safety_state": "FAIL_CLOSED",
    })
    hold.plan_id = overtaking_plan_identity(hold)
    renewed_hold = SimpleNamespace(**{
        **vars(hold), "plan_revision": hold.plan_revision + 1,
    })
    renewed_hold.plan_id = overtaking_plan_identity(renewed_hold)
    assert select_override(
        nominal, renewed_hold, prediction, 10.2, committed_plan=hold,
    ).active_reference_valid

    overlong = SimpleNamespace(**{
        **vars(plan), "continuation_until": SimpleNamespace(sec=26, nanosec=0),
    })
    overlong.plan_id = overtaking_plan_identity(overlong)
    assert not select_override(nominal, overlong, prediction, 10.1).active_reference_valid

    complete = SimpleNamespace(**{
        **vars(plan), "valid": False, "maneuver_state": "COMPLETE",
    })
    assert matching_complete(complete, plan)
    assert not matching_complete(
        SimpleNamespace(**{**vars(complete), "plan_revision": plan.plan_revision + 1}), plan,
    )
    published = []
    expired_node = SimpleNamespace(
        _accept_overtaking_plans=True,
        _plan=None, _expired_plan=plan, _nominal=nominal,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_200_000_000)),
        publish=published.append,
    )
    NominalActiveReferenceNode.receive_plan(expired_node, complete)
    assert published == [nominal]
    assert expired_node._plan is None and expired_node._expired_plan is None
    assert not select_override(nominal, complete, None, 10.2,
                               committed_plan=plan).active_reference_valid


@pytest.mark.parametrize(
    "mutation,reason",
    [
        ({"source_prediction_id": ""}, "identity mismatch"),
        ({"source_prediction_id": "sha256:forged"}, "identity mismatch"),
        ({"source_track_revision": 6}, "revision mismatch"),
        ({"source_observation_time": SimpleNamespace(sec=9, nanosec=0)}, "time mismatch"),
        ({"track_id": "wrong"}, "track/frame mismatch"),
        ({"frame_id": "odom"}, "track/frame mismatch"),
    ],
)
def test_override_rejects_incoherent_prediction_provenance(mutation, reason):
    nominal = select_nominal(load_racing_trajectory())
    stamp = SimpleNamespace(sec=10, nanosec=0)
    prediction = prediction_fixture(stamp, nominal)
    plan = SimpleNamespace(
        schema_version=2, valid=True, maneuver_state="COMMITTED_PASS", passing_side="LEFT",
        safety_state="TEMPORAL_SAFE", plan_id="placeholder", plan_revision=4,
        source_prediction_id=prediction_identity(prediction), source_track_revision=7,
        source_observation_time=stamp, header=SimpleNamespace(stamp=stamp),
        valid_until=SimpleNamespace(sec=10, nanosec=500_000_000),
        continuation_until=SimpleNamespace(sec=20, nanosec=0),
        status_reason="SAFE_CANDIDATE_COMMITTED", track_id=nominal.track_id,
        frame_id=nominal.frame_id, map_image_sha256=nominal.map_image_sha256,
        sample_count=nominal.sample_count, samples=to_message(nominal).samples,
        station_period_m=nominal.station_period_m, path_length_m=nominal.path_length_m,
        start_progress_m=0.0, commit_end_progress_m=5.0,
        pass_end_progress_m=25.0, merge_end_progress_m=30.0,
    )
    plan.plan_id = overtaking_plan_identity(plan)
    forged = SimpleNamespace(**{**vars(plan), **mutation})
    active = select_override(nominal, forged, prediction, 10.1)
    assert not active.active_reference_valid
    assert reason in active.status_reason


def test_override_rejects_stale_future_and_tampered_content():
    nominal = select_nominal(load_racing_trajectory())
    stamp = SimpleNamespace(sec=10, nanosec=0)
    prediction = prediction_fixture(stamp, nominal)
    plan = SimpleNamespace(
        schema_version=2, valid=True, maneuver_state="COMMITTED_PASS", passing_side="LEFT",
        safety_state="TEMPORAL_SAFE", plan_id="placeholder", plan_revision=4,
        source_prediction_id=prediction_identity(prediction), source_track_revision=7,
        source_observation_time=stamp, header=SimpleNamespace(stamp=stamp),
        valid_until=SimpleNamespace(sec=10, nanosec=500_000_000),
        continuation_until=SimpleNamespace(sec=20, nanosec=0),
        status_reason="SAFE_CANDIDATE_COMMITTED", track_id=nominal.track_id,
        frame_id=nominal.frame_id, map_image_sha256=nominal.map_image_sha256,
        sample_count=nominal.sample_count, samples=to_message(nominal).samples,
        station_period_m=nominal.station_period_m, path_length_m=nominal.path_length_m,
        start_progress_m=0.0, commit_end_progress_m=5.0,
        pass_end_progress_m=25.0, merge_end_progress_m=30.0,
    )
    plan.plan_id = overtaking_plan_identity(plan)
    assert "stale" in select_override(nominal, plan, prediction, 10.6).status_reason
    future_stamp = SimpleNamespace(sec=11, nanosec=0)
    future_prediction = SimpleNamespace(**{**vars(prediction), "source_observation_time": future_stamp})
    future_plan = SimpleNamespace(**{
        **vars(plan), "source_observation_time": future_stamp,
        "header": SimpleNamespace(stamp=future_stamp),
        "valid_until": SimpleNamespace(sec=11, nanosec=500_000_000),
        "continuation_until": SimpleNamespace(sec=21, nanosec=0),
        "source_prediction_id": prediction_identity(future_prediction),
    })
    future_plan.plan_id = overtaking_plan_identity(future_plan)
    assert "future" in select_override(nominal, future_plan, future_prediction, 10.5).status_reason
    tampered = SimpleNamespace(**{**vars(plan), "pass_end_progress_m": 26.0})
    assert "content identity" in select_override(nominal, tampered, prediction, 10.1).status_reason


def test_prediction_identity_binds_decision_payload():
    nominal = select_nominal(load_racing_trajectory())
    prediction = prediction_fixture(SimpleNamespace(sec=10, nanosec=0), nominal)
    changed = SimpleNamespace(**vars(prediction))
    changed.samples = [SimpleNamespace(**{
        **vars(prediction.samples[0]), "occupancy_half_width_m": 9.0,
    })]
    assert prediction_identity(prediction) != prediction_identity(changed)


def test_committed_plan_expiry_continues_then_fails_degraded_not_nominal():
    plan = SimpleNamespace(
        valid_until=SimpleNamespace(sec=10, nanosec=500_000_000),
        continuation_until=SimpleNamespace(sec=20, nanosec=0),
    )
    assert override_expiry_action(plan, 10_400_000_000) == "FRESH"
    assert override_expiry_action(plan, 10_500_000_000) == "CONTINUE"
    assert override_expiry_action(plan, 20_000_000_000) == "DEGRADE"


def test_abort_complete_cannot_bypass_parked_recovery_gate(monkeypatch):
    """Even an identity-matched COMPLETE is not a new safety certificate."""
    import f1tenth_racing_reference.node as adapter
    monkeypatch.setattr(adapter, 'matching_complete', lambda *_: True)
    accepted = SimpleNamespace(maneuver_state='ABORT',plan_id='a',plan_revision=4)
    completed = SimpleNamespace(valid=False,maneuver_state='COMPLETE')
    published=[]
    node=SimpleNamespace(
        _accept_overtaking_plans=True,
        _plan=accepted,_expired_plan=None,_nominal='unrestricted_nominal',
        _recovery_good_ticks=2,_recovery_last_sequence=42,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)),
        publish=published.append)
    NominalActiveReferenceNode.receive_plan(node,completed)
    assert node._plan is None and node._expired_plan is accepted
    assert node._recovery_good_ticks==0 and node._recovery_last_sequence==-1
    assert len(published)==1 and not published[0].active_reference_valid
    assert published[0].status_reason=='H2H_ABORT_COMPLETE_NOT_AUTHORIZED_FOR_NOMINAL'


def test_blocking_hold_complete_cannot_grant_nominal(monkeypatch):
    import f1tenth_racing_reference.node as adapter
    monkeypatch.setattr(adapter, 'matching_complete', lambda *_: True)
    accepted=SimpleNamespace(maneuver_state='BLOCKING_HOLD')
    published=[]
    node=SimpleNamespace(
        _accept_overtaking_plans=True,
        _plan=accepted,_expired_plan=None,_nominal='unrestricted_nominal',
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)),
        publish=published.append)
    NominalActiveReferenceNode.receive_plan(node,
        SimpleNamespace(valid=False,maneuver_state='COMPLETE'))
    assert node._expired_plan is accepted
    assert len(published)==1 and not published[0].active_reference_valid


def test_future_prediction_never_inserted_in_nominal_override_cache():
    clock=SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    nominal=SimpleNamespace(track_id='iccas2025',frame_id='map')
    node=SimpleNamespace(
        _nominal=nominal,_predictions={},_recovery_prediction=None,
        get_clock=lambda: clock,
        get_parameter=lambda _: SimpleNamespace(value=0.5))
    p=SimpleNamespace(
        source_observation_time=SimpleNamespace(sec=11,nanosec=0),
        track_id='iccas2025',frame_id='map')
    NominalActiveReferenceNode.receive_prediction(node,p)
    assert node._predictions=={} and node._recovery_prediction is None


@pytest.mark.parametrize("plan", [
    SimpleNamespace(valid=True, maneuver_state="COMMITTED_PASS"),
    SimpleNamespace(valid=False, maneuver_state="COMPLETE"),
    SimpleNamespace(valid=True, maneuver_state="ABORT"),
])
def test_dev_observe_never_consumes_overtaking_plan(plan):
    """Observer-only R-E must not change S0-R, even for COMPLETE/ABORT."""
    def forbidden_clock():
        raise AssertionError("OBSERVER_CONSUMED_PLAN_OR_CLOCK")
    published = []
    node = SimpleNamespace(
        _accept_overtaking_plans=False,
        _plan=None, _expired_plan=None, _nominal="nominal", _active="nominal",
        get_clock=forbidden_clock, publish=published.append,
    )
    assert NominalActiveReferenceNode.receive_plan(node, plan) is None
    assert published == []
    assert node._plan is None and node._expired_plan is None
    assert node._active == "nominal"
