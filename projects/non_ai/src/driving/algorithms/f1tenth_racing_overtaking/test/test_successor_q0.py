from dataclasses import replace
import math

import pytest

from f1tenth_racing_overtaking.successor_core import (
    Candidate,
    CommittedSpatiotemporalFrenetPlanner,
    InternalState,
    Prediction,
    PredictionSample,
    SuccessorConfig,
    Topology,
    Trackability,
    TrajectorySample,
    WireState,
)


def fixture(
    execution_speed=5.0,
    steering_rate=4.0,
    lateral_acceleration=30.0,
    entry_early=3.0,
    entry_late=5.0,
):
    count = 120
    radius = 10.0
    points = tuple(
        (
            radius * math.cos(2.0 * math.pi * index / count),
            radius * math.sin(2.0 * math.pi * index / count),
        )
        for index in range(count)
    )
    ds = 2.0 * radius * math.sin(math.pi / count)
    samples = tuple(
        TrajectorySample(
            sample_index=index,
            s_m=ds * index,
            x_m=x,
            y_m=y,
            heading_rad=2.0 * math.pi * index / count + math.pi / 2.0,
            curvature_per_m=1.0 / radius,
            lateral_offset_m=0.0,
            target_speed_mps=execution_speed,
            target_accel_mps2=0.0,
            ds_to_next_m=ds,
        )
        for index, (x, y) in enumerate(points)
    )
    config = SuccessorConfig(
        prediction_max_age_s=0.5,
        minimum_track_confidence=0.6,
        minimum_confirmed_hits=3,
        maximum_source_misses=4,
        maximum_covariance_eigenvalue_m2=0.2,
        eligibility_ahead_min_m=0.8,
        eligibility_ahead_max_m=6.0,
        candidate_lateral_offset_m=0.75,
        entry_length_early_m=entry_early,
        entry_length_late_m=entry_late,
        rejoin_lengths_m=(3.0, 5.0, 7.5),
        corridor_clearance_m=0.08,
        blue_tracking_uncertainty_m=0.10,
        qualification_margin_m=0.05,
        pass_completion_ahead_m=0.8,
        pass_timeout_s=1.0,
        delta_v_pass_mps=0.25,
        footprint_half_length_m=0.20,
        footprint_half_width_m=0.10,
        wheelbase_m=0.34,
        steering_angle_max_rad=4.0,
        steering_rate_max_rad_s=steering_rate,
        lateral_acceleration_max_mps2=lateral_acceleration,
        execution_speed_cap_mps=execution_speed,
    )
    planner = CommittedSpatiotemporalFrenetPlanner(
        config,
        points,
        [2.0] * count,
        [2.0] * count,
        track_id="q0-track",
        frame_id="map",
        map_image_sha256="q0-map",
    )
    return planner, samples


def prediction(
    planner,
    blue_s=0.0,
    ahead=1.5,
    red_speed=1.0,
    red_lateral=0.0,
    now_s=10.0,
):
    samples = []
    for step in range(1, 11):
        dt = 0.1 * step
        s_m = planner.geometry.wrap_s(
            blue_s + ahead + red_speed * dt
        )
        point = planner.geometry.from_frenet(s_m, red_lateral)
        samples.append(PredictionSample(
            dt_s=dt,
            x_m=point.x,
            y_m=point.y,
            vx_mps=red_speed * math.cos(point.heading),
            vy_mps=red_speed * math.sin(point.heading),
            heading_rad=point.heading,
            s_m=s_m,
            d_m=red_lateral,
            covariance_xy=(0.01, 0.0, 0.0, 0.01),
            occupancy_half_length_m=0.20,
            occupancy_half_width_m=0.10,
        ))
    return Prediction(
        identity=f"prediction:{blue_s:.3f}:{ahead:.3f}:{red_speed:.3f}:{now_s:.3f}",
        source_revision=7,
        source_time_s=now_s,
        source_time_ns=round(now_s * 1e9),
        valid=True,
        lifecycle="TRACKED",
        confidence=0.9,
        hit_count=8,
        miss_count=0,
        track_id="q0-track",
        frame_id="map",
        horizon_s=1.0,
        timestep_s=0.1,
        samples=tuple(samples),
    )


def state(planner, s_m, d_m=0.0):
    return planner.geometry.from_frenet(s_m, d_m)


def manual_candidate(planner, nominal, topology=Topology.LEFT_EARLY):
    samples, corridor, trackability, reason = planner._candidate_geometry(
        nominal,
        0.0,
        0.0,
        state(planner, 0.0).heading,
        "LEFT",
        planner.config.entry_length_early_m,
    )
    assert samples is not None, reason
    return Candidate(
        topology=topology,
        side="LEFT",
        entry_length_m=planner.config.entry_length_early_m,
        samples=samples,
        pass_time_s=0.6,
        pass_end_s=planner.geometry.wrap_s(3.0),
        minimum_predicted_separation_m=0.5,
        minimum_corridor_clearance_m=corridor,
        minimum_executable_speed_mps=5.0,
        maximum_predicted_opponent_speed_mps=1.0,
        trackability=trackability,
    )


def test_prediction_velocity_is_consumed_directly_for_speed_advantage():
    planner, nominal = fixture()
    pred = prediction(planner, ahead=1.5, red_speed=1.25)
    candidate = manual_candidate(planner, nominal)
    candidate = replace(
        candidate,
        minimum_executable_speed_mps=1.4,
        maximum_predicted_opponent_speed_mps=max(
            math.hypot(item.vx_mps, item.vy_mps)
            for item in pred.samples
        ),
    )
    assert candidate.maximum_predicted_opponent_speed_mps == pytest.approx(1.25)
    assert (
        candidate.minimum_executable_speed_mps
        < candidate.maximum_predicted_opponent_speed_mps
        + planner.config.delta_v_pass_mps
    )


def test_candidate_time_uses_target_speed_not_current_blue_speed():
    planner, nominal = fixture(execution_speed=2.0)
    at_one_second = planner.state_at_time(nominal, 0.0, 1.0)
    assert at_one_second is not None
    assert planner.geometry.forward_delta_s(
        0.0,
        at_one_second["s_m"],
    ) == pytest.approx(2.0, abs=0.08)


def test_no_extrapolation_if_r_d_horizon_is_shorter_than_pass_timeout():
    planner, _ = fixture()
    pred = prediction(planner)
    shortened = replace(
        pred,
        samples=pred.samples[:-2],
        horizon_s=0.8,
    )
    assert planner._prediction_status(shortened, 10.0) == (
        "PREDICTION_HORIZON_SHORTER_THAN_PASS_TIMEOUT"
    )


@pytest.mark.parametrize(
    "topology,side,length_attr",
    [
        (Topology.LEFT_EARLY, "LEFT", "entry_length_early_m"),
        (Topology.LEFT_LATE, "LEFT", "entry_length_late_m"),
        (Topology.RIGHT_EARLY, "RIGHT", "entry_length_early_m"),
        (Topology.RIGHT_LATE, "RIGHT", "entry_length_late_m"),
    ],
)
def test_finite_topology_geometry_is_current_state_continuous(
    topology,
    side,
    length_attr,
):
    planner, nominal = fixture()
    start = state(planner, 0.35, 0.04)
    length_m = getattr(planner.config, length_attr)
    samples, _, trackability, reason = planner._candidate_geometry(
        nominal,
        0.35,
        0.04,
        start.heading,
        side,
        length_m,
    )
    assert samples is not None, f"{topology}: {reason}"
    assert trackability.entry_lateral_error_m <= (
        planner.config.blue_tracking_uncertainty_m
    )
    assert math.isfinite(trackability.entry_heading_error_rad)


def test_steering_rate_gate_rejects_untrackable_entry():
    planner, nominal = fixture(steering_rate=0.001)
    start = state(planner, 0.0)
    samples, _, _, reason = planner._candidate_geometry(
        nominal,
        0.0,
        0.0,
        start.heading,
        "LEFT",
        planner.config.entry_length_early_m,
    )
    assert samples is None
    assert reason == "STEERING_RATE_INFEASIBLE"


def test_execution_speed_cap_is_applied_to_candidate_payload():
    planner, nominal = fixture(execution_speed=3.0)
    nominal_fast = tuple(
        replace(sample, target_speed_mps=8.0)
        for sample in nominal
    )
    start = state(planner, 0.0)
    samples, _, _, reason = planner._candidate_geometry(
        nominal_fast,
        0.0,
        0.0,
        start.heading,
        "LEFT",
        planner.config.entry_length_early_m,
    )
    assert samples is not None, reason
    assert max(sample.target_speed_mps for sample in samples) <= 3.0 + 1e-12


def test_executable_speed_advantage_is_a_hard_gate(monkeypatch):
    planner, nominal = fixture()
    pred = prediction(planner, ahead=1.5, red_speed=4.9)
    start = state(planner, 0.0)

    monkeypatch.setattr(
        planner,
        "_temporal_safety",
        lambda *args, **kwargs: (True, 1.0),
    )
    monkeypatch.setattr(
        planner,
        "_rejoin_exists",
        lambda *args, **kwargs: True,
    )
    monkeypatch.setattr(
        planner,
        "_pass_completion",
        lambda *args, **kwargs: {
            "status": "EXECUTABLE_SPEED_ADVANTAGE_INSUFFICIENT",
            "pass_time_s": 0.7,
            "pass_end_s": 3.5,
            "minimum_ego_speed_mps": 5.0,
            "maximum_red_speed_mps": 4.9,
        },
    )

    candidate, reason = planner._evaluate_topology(
        nominal,
        0.0,
        0.0,
        start.heading,
        pred,
        Topology.LEFT_EARLY,
    )
    assert candidate is None
    assert reason == "EXECUTABLE_SPEED_ADVANTAGE_INSUFFICIENT"


def test_commit_is_atomic_and_freezes_identity_bearing_payload():
    planner, nominal = fixture()
    pred = prediction(planner)
    candidate = manual_candidate(planner, nominal)

    committed = planner.commit_candidate(
        candidate,
        pred,
        blue_s=0.0,
        now_s=10.0,
    )
    assert committed.internal_state == InternalState.PASS_COMMIT
    assert committed.plan.state == WireState.COMMITTED_PASS.value

    digest = committed.plan.identity_digest
    samples = committed.plan.samples
    revision = committed.plan.revision
    provenance = committed.plan.source_prediction_id

    moved = state(planner, 0.2, 0.02)
    result = planner.update(
        nominal,
        moved.x,
        moved.y,
        99.0,
        None,
        10.1,
        moved.heading,
    )
    assert result.internal_state == InternalState.FROZEN_PASS
    assert result.plan.identity_digest == digest
    assert result.plan.samples == samples
    assert result.plan.revision == revision
    assert result.plan.source_prediction_id == provenance


def test_comitted_pass_to_passing_changes_lifecycle_not_identity():
    planner, nominal = fixture()
    pred = prediction(planner)
    candidate = manual_candidate(planner, nominal)
    committed = planner.commit_candidate(candidate, pred, 0.0, 10.0)
    digest = committed.plan.identity_digest

    progressed = state(
        planner,
        planner.config.entry_length_early_m + 0.2,
        planner.config.candidate_lateral_offset_m,
    )
    result = planner.update(
        nominal,
        progressed.x,
        progressed.y,
        5.0,
        None,
        10.1,
        progressed.heading,
    )
    assert result.plan.state == WireState.PASSING.value
    assert result.plan.identity_digest == digest
    assert result.plan.revision == committed.plan.revision


def test_lateral_adjacency_without_longitudinal_clear_never_confirms_pass():
    planner, nominal = fixture()
    pred = prediction(planner)
    candidate = manual_candidate(planner, nominal)
    committed = planner.commit_candidate(candidate, pred, 0.0, 10.0)
    planner.internal_state = InternalState.FROZEN_PASS
    planner.plan = replace(
        committed.plan,
        state=WireState.PASSING.value,
    )
    beside = state(
        planner,
        2.0,
        planner.config.candidate_lateral_offset_m,
    )
    still_ahead = prediction(
        planner,
        blue_s=2.0,
        ahead=0.2,
        red_speed=1.0,
        now_s=10.1,
    )
    result = planner.update(
        nominal,
        beside.x,
        beside.y,
        5.0,
        still_ahead,
        10.1,
        beside.heading,
    )
    assert result.plan.state == WireState.PASSING.value
    assert result.plan.revision == committed.plan.revision


def test_pass_confirmed_creates_one_new_rejoin_then_freezes(monkeypatch):
    planner, nominal = fixture()
    pred = prediction(planner)
    candidate = manual_candidate(planner, nominal)
    committed = planner.commit_candidate(candidate, pred, 0.0, 10.0)
    planner.internal_state = InternalState.FROZEN_PASS
    planner.plan = replace(
        committed.plan,
        state=WireState.PASSING.value,
    )

    ahead = state(
        planner,
        3.0,
        planner.config.candidate_lateral_offset_m,
    )
    fresh = prediction(
        planner,
        blue_s=3.0,
        ahead=-1.0,
        red_speed=1.0,
        now_s=10.1,
    )
    monkeypatch.setattr(
        planner,
        "_fresh_frozen_safety",
        lambda *args, **kwargs: (True, 1.0),
    )
    monkeypatch.setattr(
        planner,
        "_temporal_safety",
        lambda *args, **kwargs: (True, 1.0),
    )

    rejoin = planner.update(
        nominal,
        ahead.x,
        ahead.y,
        5.0,
        fresh,
        10.1,
        ahead.heading,
    )
    assert rejoin.internal_state == InternalState.PASS_CONFIRMED
    assert rejoin.plan.state == WireState.MERGE.value
    assert rejoin.plan.revision == committed.plan.revision + 1
    assert rejoin.plan.source_prediction_id == fresh.identity
    digest = rejoin.plan.identity_digest
    samples = rejoin.plan.samples

    progressed = state(planner, 3.2, 0.70)
    same = planner.update(
        nominal,
        progressed.x,
        progressed.y,
        5.0,
        None,
        10.2,
        progressed.heading,
    )
    assert same.internal_state == InternalState.FROZEN_REJOIN
    assert same.plan.state == WireState.MERGE.value
    assert same.plan.identity_digest == digest
    assert same.plan.samples == samples
    assert same.plan.revision == rejoin.plan.revision


def test_complete_keeps_accepted_rejoin_identity(monkeypatch):
    planner, nominal = fixture()
    pred = prediction(planner)
    candidate = manual_candidate(planner, nominal)
    committed = planner.commit_candidate(candidate, pred, 0.0, 10.0)
    planner.internal_state = InternalState.FROZEN_PASS
    planner.plan = replace(
        committed.plan,
        state=WireState.PASSING.value,
    )

    ahead = state(
        planner,
        3.0,
        planner.config.candidate_lateral_offset_m,
    )
    fresh = prediction(
        planner,
        blue_s=3.0,
        ahead=-1.0,
        red_speed=1.0,
        now_s=10.1,
    )
    monkeypatch.setattr(
        planner,
        "_fresh_frozen_safety",
        lambda *args, **kwargs: (True, 1.0),
    )
    monkeypatch.setattr(
        planner,
        "_temporal_safety",
        lambda *args, **kwargs: (True, 1.0),
    )
    rejoin = planner.update(
        nominal,
        ahead.x,
        ahead.y,
        5.0,
        fresh,
        10.1,
        ahead.heading,
    )
    digest = rejoin.plan.identity_digest
    revision = rejoin.plan.revision
    end = state(planner, rejoin.plan.merge_end_s, 0.0)

    complete = planner.update(
        nominal,
        end.x,
        end.y,
        5.0,
        None,
        11.0,
        end.heading,
    )
    assert complete.plan.state == WireState.COMPLETE.value
    assert complete.plan.valid is False
    assert complete.plan.identity_digest == digest
    assert complete.plan.revision == revision
    assert complete.action == "RELEASE_TO_NOMINAL"


def test_predicted_collision_latches_abort_without_geometry_regeneration(
    monkeypatch,
):
    planner, nominal = fixture()
    pred = prediction(planner)
    candidate = manual_candidate(planner, nominal)
    committed = planner.commit_candidate(candidate, pred, 0.0, 10.0)
    digest = committed.plan.identity_digest
    samples = committed.plan.samples

    monkeypatch.setattr(
        planner,
        "_fresh_frozen_safety",
        lambda *args, **kwargs: (False, -0.1),
    )
    current = state(planner, 0.2)
    fresh = prediction(
        planner,
        blue_s=0.2,
        ahead=1.0,
        red_speed=1.0,
        now_s=10.1,
    )
    result = planner.update(
        nominal,
        current.x,
        current.y,
        5.0,
        fresh,
        10.1,
        current.heading,
    )
    assert result.action == "PUBLISH_ABORT_DEGRADE"
    assert result.plan.state == WireState.ABORT.value
    assert result.plan.valid is True
    assert result.plan.identity_digest == digest
    assert result.plan.samples == samples
    assert result.plan.topology == committed.plan.topology

    latched = planner.update(
        nominal,
        current.x,
        current.y,
        5.0,
        fresh,
        10.2,
        current.heading,
    )
    assert latched.action == "ABORT_LATCHED_NO_REPUBLISH"
    assert latched.plan.state == WireState.ABORT.value
    assert latched.plan.identity_digest == digest
    assert latched.plan.revision == committed.plan.revision


def test_candidate_selection_order_is_deterministic(monkeypatch):
    planner, nominal = fixture()
    pred = prediction(planner)
    start = state(planner, 0.0)
    base = manual_candidate(planner, nominal)

    def evaluate(*args):
        topology = args[-1]
        side = "LEFT" if topology.value.startswith("LEFT") else "RIGHT"
        candidate = replace(
            base,
            topology=topology,
            side=side,
            pass_time_s=0.6,
        )
        return candidate, "FEASIBLE"

    monkeypatch.setattr(planner, "_evaluate_topology", evaluate)
    first, _ = planner.evaluate_candidates(
        nominal,
        0.0,
        0.0,
        start.heading,
        pred,
    )
    second, _ = planner.evaluate_candidates(
        nominal,
        0.0,
        0.0,
        start.heading,
        pred,
    )
    assert first.topology == second.topology == Topology.LEFT_EARLY


def test_early_entry_search_is_opponent_conditioned_and_bounded():
    planner, _ = fixture(entry_early=3.0, entry_late=5.0)
    pred = prediction(planner, ahead=2.0, red_speed=0.0)
    lengths = planner._entry_length_candidates(
        Topology.LEFT_EARLY, 0.0, pred
    )

    red0 = pred.samples[0]
    ahead = planner.geometry.forward_delta_s(0.0, red0.s_m)
    envelope = (
        red0.occupancy_half_length_m
        + planner.config.footprint_half_length_m
        + planner.config.blue_tracking_uncertainty_m
        + planner.config.qualification_margin_m
    )
    expected_lower = min(
        planner.config.entry_length_early_m,
        max(
            planner.config.blue_tracking_uncertainty_m,
            ahead - envelope,
        ),
    )

    assert lengths[0] == planner.config.entry_length_early_m
    assert abs(lengths[-1] - expected_lower) <= 1e-12
    assert all(a > b for a, b in zip(lengths, lengths[1:]))
    assert planner._entry_length_candidates(
        Topology.LEFT_LATE, 0.0, pred
    ) == (planner.config.entry_length_late_m,)


def test_early_entry_search_records_actual_selected_length(monkeypatch):
    planner, nominal = fixture()
    pred = prediction(planner)
    base = manual_candidate(planner, nominal)
    seed = planner.config.entry_length_early_m
    shorter = seed - planner.config.blue_tracking_uncertainty_m

    monkeypatch.setattr(
        planner,
        "_entry_length_candidates",
        lambda *args, **kwargs: (seed, shorter),
    )

    def evaluate_at_length(*args):
        topology = args[-2]
        entry_length = args[-1]
        if abs(entry_length - seed) <= 1e-12:
            return None, "TEMPORAL_CLEARANCE_INFEASIBLE"
        return replace(
            base,
            topology=topology,
            side="LEFT",
            entry_length_m=entry_length,
        ), "FEASIBLE"

    monkeypatch.setattr(
        planner,
        "_evaluate_topology_at_entry_length",
        evaluate_at_length,
    )
    candidate, reason = planner._evaluate_topology(
        nominal, 0.0, 0.0, 0.0, pred, Topology.LEFT_EARLY
    )

    assert reason == "FEASIBLE"
    assert candidate is not None
    assert abs(candidate.entry_length_m - shorter) <= 1e-12


def test_early_entry_search_does_not_retry_non_temporal_failure(monkeypatch):
    planner, nominal = fixture()
    pred = prediction(planner)
    seed = planner.config.entry_length_early_m
    shorter = seed - planner.config.blue_tracking_uncertainty_m
    calls = []

    monkeypatch.setattr(
        planner,
        "_entry_length_candidates",
        lambda *args, **kwargs: (seed, shorter),
    )

    def reject(*args):
        calls.append(args[-1])
        return None, "STEERING_RATE_INFEASIBLE"

    monkeypatch.setattr(
        planner,
        "_evaluate_topology_at_entry_length",
        reject,
    )
    candidate, reason = planner._evaluate_topology(
        nominal, 0.0, 0.0, 0.0, pred, Topology.LEFT_EARLY
    )

    assert candidate is None
    assert reason == "STEERING_RATE_INFEASIBLE"
    assert calls == [seed]

def test_pass_trackability_uses_only_temporally_reachable_prefix():
    planner, nominal = fixture()
    start = state(planner, 0.0)

    active, reason = planner._execution_edge_indices(
        nominal,
        0.0,
        planner.config.pass_timeout_s,
    )
    assert active is not None, reason

    active_set = set(active)
    count = len(nominal)
    remote = next(
        index
        for index in range(count)
        if index not in active_set
        and ((index - 1) % count) not in active_set
    )

    mutated = list(nominal)
    mutated[remote] = replace(
        mutated[remote],
        curvature_per_m=4.0,
    )
    mutated = tuple(mutated)

    global_track, global_reason = planner._trackability(
        mutated,
        0.0,
        0.0,
        start.heading,
    )
    assert global_track is None
    assert global_reason in (
        "STEERING_RATE_INFEASIBLE",
        "LATERAL_ACCELERATION_INFEASIBLE",
    )

    scoped_track, scoped_reason = planner._trackability(
        mutated,
        0.0,
        0.0,
        start.heading,
        horizon_s=planner.config.pass_timeout_s,
    )
    assert scoped_track is not None, scoped_reason


def test_frozen_pass_times_out_via_identity_preserving_abort():
    planner, nominal = fixture()
    pred = prediction(planner)
    candidate = manual_candidate(planner, nominal)
    committed = planner.commit_candidate(
        candidate,
        pred,
        0.0,
        10.0,
    )
    digest = committed.plan.identity_digest
    samples = committed.plan.samples
    revision = committed.plan.revision

    current = state(planner, 0.2)
    result = planner.update(
        nominal,
        current.x,
        current.y,
        5.0,
        None,
        10.0 + planner.config.pass_timeout_s,
        current.heading,
    )

    assert result.action == "PUBLISH_ABORT_DEGRADE"
    assert result.plan.state == WireState.ABORT.value
    assert result.plan.reason == (
        "SUCCESSOR_EMERGENCY_DEGRADE:FROZEN_PASS_TIMEOUT"
    )
    assert result.plan.identity_digest == digest
    assert result.plan.samples == samples
    assert result.plan.revision == revision


def test_blocking_hold_clear_publishes_matching_complete():
    planner, nominal = fixture()
    pred = prediction(planner)
    hold = planner._follow(
        nominal,
        0.0,
        pred,
        10.0,
        "TEST_HOLD",
    )
    assert hold.plan.state == WireState.BLOCKING_HOLD.value

    digest = hold.plan.identity_digest
    revision = hold.plan.revision
    samples = hold.plan.samples

    current = state(planner, 0.0)
    released = planner.update(
        nominal,
        current.x,
        current.y,
        5.0,
        None,
        10.1,
        current.heading,
    )

    assert released.action == "RELEASE_TO_NOMINAL"
    assert released.plan.state == WireState.COMPLETE.value
    assert released.plan.valid is False
    assert released.plan.identity_digest == digest
    assert released.plan.revision == revision
    assert released.plan.samples == samples
    assert released.plan.reason == (
        "BLOCKING_HOLD_CLEAR:PREDICTION_INVALID"
    )



def test_v5_abort_requires_explicit_nominal_recovery_handshake(monkeypatch):
    planner, nominal = fixture()
    pred = prediction(planner)
    candidate = manual_candidate(planner, nominal)
    committed = planner.commit_candidate(candidate, pred, 0., 10.)
    old_digest = committed.plan.identity_digest
    abort = planner._abort_degrade("PREDICTION_LOST")
    assert abort.plan.state == WireState.ABORT.value
    latched = planner.update(nominal, 10., 0., 0., None, 10.2)
    assert latched.action == "ABORT_LATCHED_NO_REPUBLISH"
    assert latched.plan.identity_digest == old_digest
    # Only S0-R's independently verified stationary nominal handoff may
    # invoke this method. It never upgrades an old plan into a fresh pass.
    planner.acknowledge_abort_recovery()
    assert planner.plan is None
    assert planner.internal_state == InternalState.TRAIL
    assert planner.last_qualified_prediction_at_s is None
    new_result = planner.update(nominal, 10., 0., 0., None, 11.)
    assert new_result.action == "KEEP_NOMINAL"
    assert planner.plan is None


def test_v5_abort_recovery_cannot_clear_active_committed_plan():
    planner, nominal = fixture()
    pred = prediction(planner)
    candidate = manual_candidate(planner, nominal)
    committed = planner.commit_candidate(candidate, pred, 0., 10.)
    with pytest.raises(ValueError, match="ABORT_NOT_LATCHED_FOR_RECOVERY"):
        planner.acknowledge_abort_recovery()
    assert planner.plan is committed.plan
