from dataclasses import replace
import math
from types import SimpleNamespace

from f1tenth_racing_overtaking.core import (
    OvertakingConfig, OvertakingSupervisor, Prediction, PredictionSample, TrajectorySample,
    continuation_deadline_s,
)
from f1tenth_racing_overtaking.node import OvertakingPlannerNode


def fixture():
    count, radius = 40, 10.0
    points = tuple((radius * math.cos(2 * math.pi * i / count), radius * math.sin(2 * math.pi * i / count)) for i in range(count))
    geometry_length = 2 * count * radius * math.sin(math.pi / count)
    samples = []
    for i, (x, y) in enumerate(points):
        heading = 2 * math.pi * i / count + math.pi / 2
        samples.append(TrajectorySample(i, geometry_length * i / count, x, y, heading, 0.1, 0.0, 3.0, 0.0, geometry_length / count))
    config = OvertakingConfig(0.5, 0.6, 3, 4, 0.2, 0.8, 6.0, 0.25, 0.75,
                               3.0, 7.0, 3.0, 0.2, 0.2, 0.08, 0.1, 0.05, 1.2,
                               0.34, 4.0, 4.0, 3.0, 5.0, 1.0, 0.05)
    return OvertakingSupervisor(config, points, [1.5] * count, [1.5] * count), tuple(samples)


def test_only_active_moving_containment_renews_bounded_liveness_lease():
    deadline = continuation_deadline_s
    assert deadline(
        "COMMITTED_PASS", "SAFE_CANDIDATE_COMMITTED", 10.0, 12.0, 12.5, 15.0,
    ) == 25.0
    assert deadline(
        "ABORT", "COMMITTED_ABORT:PREDICTION_INVALID", 10.0, 12.0, 12.5, 15.0,
    ) == 25.0
    assert deadline(
        "ABORT", "COMMITTED_ABORT:TEMPORAL_CLEARANCE_LOST:MERGING",
        10.0, 12.0, 12.5, 15.0,
    ) == 25.0
    assert deadline(
        "ABORT", "COMMITTED_ABORT:TEMPORAL_CLEARANCE_LOST", 10.0, 12.0, 12.5, 15.0,
    ) == 27.0
    assert deadline(
        "ABORT", "COMMITTED_ABORT:TRACKING_ERROR_EXCEEDED", 10.0, 12.0, 12.5, 15.0,
    ) == 27.0
    assert deadline(
        "ABORT", "COMMITTED_ABORT:TEMPORAL_CLEARANCE_LOST", 10.0, 26.8, 27.3, 15.0,
    ) == 41.8
    assert deadline(
        "BLOCKING_HOLD", "BLOCKING_HOLD:NO_SAFE_CANDIDATE", 10.0, 12.0, 12.5, 15.0,
    ) == 27.0


def prediction(supervisor, blue_s=0.0, ahead=4.0, lateral=0.0, valid=True, age=0.0):
    items = []
    for step in range(1, 11):
        dt = step * 0.1
        s = supervisor.geometry.wrap_s(blue_s + ahead + 2.0 * dt)
        point = supervisor.geometry.from_frenet(s, lateral)
        items.append(PredictionSample(dt, point.x, point.y, point.heading, s, lateral,
                                      (0.01, 0.0, 0.0, 0.01), 0.3, 0.3))
    return Prediction("prediction", 7, 10.0 - age, round((10.0 - age) * 1e9), 10.0, valid, "TRACKED", 0.9, 8, 0,
                      "iccas2025", "map", tuple(items))


def test_latest_invalid_prediction_is_not_hidden_from_new_plan_eligibility():
    supervisor, _ = fixture()
    node = SimpleNamespace(
        prediction=prediction(supervisor),
        settings={"prediction_max_age_s": 0.5},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_100_000_000)),
    )
    message = SimpleNamespace(
        schema_version=1, valid=False,
        source_observation_time=SimpleNamespace(sec=10, nanosec=100_000_000),
        source_opponent_id="red", source_track_revision=8, predictor="TRACK_CV",
        source_lifecycle="LOST", source_confidence=0.0,
        source_hit_count=0, source_miss_count=1,
        horizon_s=0.0, timestep_s=0.1, track_id="iccas2025", frame_id="map",
        samples=[],
    )

    OvertakingPlannerNode.receive_prediction(node, message)

    assert node.prediction.valid is False


def test_recent_blocking_opponent_prediction_loss_enters_moving_containment():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    seen = prediction(supervisor)
    supervisor.update(samples, blue.x, blue.y, 3.0, seen, 10.0)
    supervisor.plan = None

    lost = replace(seen, valid=False, samples=())
    contained = supervisor.update(samples, blue.x, blue.y, 3.0, lost, 10.1)

    assert contained.valid and contained.state == "BLOCKING_HOLD"
    assert contained.reason == "BLOCKING_HOLD:PREDICTION_INVALID"
    assert all(0.0 < sample.target_speed_mps <= 2.0 for sample in contained.samples)


def test_commit_is_deterministic_smooth_and_fail_closed():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    plan = supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    assert plan.valid and plan.state == "COMMITTED_PASS" and plan.side in ("LEFT", "RIGHT")
    assert max(sample.target_speed_mps for sample in plan.samples) <= supervisor.config.candidate_speed_cap_mps
    assert plan.minimum_corridor_clearance_m >= 0.08
    assert supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.1).state == "COMMITTED_PASS"
    passing = [sample for sample in plan.samples
               if supervisor.config.prepare_length_m <=
               supervisor.geometry.forward_delta_s(plan.start_s, sample.s_m) <
               supervisor.config.prepare_length_m + supervisor.config.pass_length_m]
    expected_d = 0.75 if plan.side == "LEFT" else -0.75
    assert passing and all(math.isclose(
        sample.lateral_offset_m,
        expected_d,
    ) for sample in passing)
    repeat, repeat_samples = fixture()
    assert repeat.update(repeat_samples, blue.x, blue.y, 3.0, prediction(repeat), 10.0) == plan
    stale, stale_samples = fixture()
    rejected = stale.update(stale_samples, blue.x, blue.y, 3.0, prediction(stale, age=1.0), 10.0)
    assert not rejected.valid and rejected.reason == "PREDICTION_STALE"


def test_candidate_is_checked_at_its_target_speed_not_current_acceleration_speed():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    checked = []
    supervisor._safe = lambda samples, blue_s, speed, prediction, *args: (
        checked.append(speed) is None, 1.0,
    )
    assert supervisor.update(samples, blue.x, blue.y, 2.5, prediction(supervisor), 10.0).valid
    assert checked == [supervisor.config.candidate_speed_cap_mps] * 2


def test_parity_releases_interaction_cap_without_acquiring_a_new_candidate():
    supervisor, samples = fixture()
    supervisor.config = replace(supervisor.config, candidate_speed_cap_mps=4.0)
    samples = tuple(replace(sample, target_speed_mps=5.0) for sample in samples)
    blue = supervisor.geometry.interpolate(0.0)
    supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    supervisor._candidate = lambda *args: (_ for _ in ()).throw(
        AssertionError("committed execution must not acquire a new candidate")
    )
    passing = supervisor.geometry.from_frenet(4.0, 0.75)

    plan = supervisor.update(
        samples, passing.x, passing.y, 4.0,
        prediction(supervisor, blue_s=4.0, ahead=-0.2), 10.1,
    )

    assert plan.state in ("COMMITTED_PASS", "PASSING")
    assert max(sample.target_speed_mps for sample in plan.samples) == 5.0


def test_safely_ahead_merge_keeps_geometry_and_releases_interaction_cap():
    supervisor, samples = fixture()
    supervisor.config = replace(supervisor.config, candidate_speed_cap_mps=4.0)
    samples = tuple(replace(sample, target_speed_mps=5.0) for sample in samples)
    blue = supervisor.geometry.interpolate(0.0)
    supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    safely_ahead = supervisor.geometry.from_frenet(6.0, 0.75)

    plan = supervisor.update(
        samples, safely_ahead.x, safely_ahead.y, 4.0,
        prediction(supervisor, blue_s=6.0, ahead=-2.0), 10.1,
    )

    assert plan.state == "MERGE"
    assert math.isclose(
        max(abs(sample.lateral_offset_m) for sample in plan.samples), 0.75,
        abs_tol=0.01,
    )
    assert max(sample.target_speed_mps for sample in plan.samples) == 5.0

    tracking_error = supervisor.geometry.from_frenet(6.5, 0.0)
    recovery = supervisor.update(
        samples, tracking_error.x, tracking_error.y, 4.0, None, 10.2,
    )
    assert recovery.state in ("PASSING", "MERGE")
    assert max(sample.target_speed_mps for sample in recovery.samples) == 5.0


def test_forensic_safe_ahead_prediction_loss_cannot_recreate_red_following_speed():
    supervisor, samples = fixture()
    supervisor.config = replace(
        supervisor.config, candidate_speed_cap_mps=4.0, racing_speed_cap_mps=5.0,
    )
    samples = tuple(replace(sample, target_speed_mps=5.0) for sample in samples)
    blue = supervisor.geometry.interpolate(0.0)
    committed = supervisor.update(
        samples, blue.x, blue.y, 4.0, prediction(supervisor), 10.0,
    )
    passing = supervisor.geometry.from_frenet(6.0, 0.75)
    safely_ahead = prediction(supervisor, blue_s=6.0, ahead=-2.0)
    supervisor.plan = replace(committed, state="PASSING")
    supervisor.last_opponent_speed_mps = 2.0
    accepted = supervisor.update(
        samples, passing.x, passing.y, 4.0, safely_ahead, 10.1,
    )

    plan = supervisor.update(
        samples, passing.x, passing.y, 4.0, None, 10.2,
    )

    assert plan == accepted
    assert plan.state in ("PASSING", "MERGE")
    assert max(sample.target_speed_mps for sample in plan.samples) == 5.0


def test_safe_ahead_waits_on_pass_corridor_when_racing_merge_is_not_trackable():
    supervisor, samples = fixture()
    supervisor.config = replace(
        supervisor.config, candidate_speed_cap_mps=4.0, racing_speed_cap_mps=5.0,
    )
    samples = tuple(replace(sample, target_speed_mps=5.0) for sample in samples)
    blue = supervisor.geometry.interpolate(0.0)
    supervisor.update(samples, blue.x, blue.y, 4.0, prediction(supervisor), 10.0)
    supervisor._rejoin_candidate = lambda *args: (
        None, -math.inf, "STEERING_RATE_INFEASIBLE",
    )
    safely_ahead = supervisor.geometry.from_frenet(6.0, 0.75)

    plan = supervisor.update(
        samples, safely_ahead.x, safely_ahead.y, 4.0,
        prediction(supervisor, blue_s=6.0, ahead=-2.0), 10.1,
    )

    assert plan.state == "PASSING"
    assert plan.reason == "REJOIN_PENDING:STEERING_RATE_INFEASIBLE"
    assert max(sample.target_speed_mps for sample in plan.samples) == 5.0


def test_clean_committed_maneuver_reaches_nominal_without_abort():
    supervisor, samples = fixture()
    supervisor.config = replace(
        supervisor.config, candidate_speed_cap_mps=4.0, racing_speed_cap_mps=5.0,
    )
    samples = tuple(replace(sample, target_speed_mps=5.0) for sample in samples)
    blue = supervisor.geometry.interpolate(0.0)
    states = [supervisor.update(
        samples, blue.x, blue.y, 4.0, prediction(supervisor), 10.0,
    ).state]

    passing = supervisor.geometry.from_frenet(5.0, 0.75)
    states.append(supervisor.update(
        samples, passing.x, passing.y, 4.0,
        prediction(supervisor, blue_s=5.0, ahead=1.0), 10.1,
    ).state)
    parity = supervisor.geometry.from_frenet(6.0, 0.75)
    parity_plan = supervisor.update(
        samples, parity.x, parity.y, 4.0,
        prediction(supervisor, blue_s=6.0, ahead=-0.2), 10.2,
    )
    states.append(parity_plan.state)
    assert max(sample.target_speed_mps for sample in parity_plan.samples) == 5.0

    safely_ahead = supervisor.geometry.from_frenet(7.0, 0.75)
    merge = supervisor.update(
        samples, safely_ahead.x, safely_ahead.y, 4.5,
        prediction(supervisor, blue_s=7.0, ahead=-2.0), 10.3,
    )
    states.append(merge.state)
    assert merge.state == "MERGE"
    assert max(sample.target_speed_mps for sample in merge.samples) == 5.0

    merged_s = supervisor.geometry.wrap_s(merge.start_s + supervisor.config.merge_length_m)
    merged = supervisor.geometry.from_frenet(merged_s, 0.0)
    complete = supervisor.update(samples, merged.x, merged.y, 5.0, None, 11.3)
    states.append(complete.state)
    nominal = supervisor.update(samples, merged.x, merged.y, 5.0, None, 11.4)
    states.append(nominal.state)

    assert states == [
        "COMMITTED_PASS", "PASSING", "PASSING", "MERGE", "COMPLETE", "NOMINAL",
    ]
    assert "ABORT" not in states


def test_real_blocking_red_without_a_feasible_candidate_remains_contained():
    supervisor, samples = fixture()
    supervisor.left_widths = supervisor.right_widths = (0.2,) * len(samples)
    blue = supervisor.geometry.interpolate(0.0)

    plan = supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)

    assert plan.state == "BLOCKING_HOLD"
    assert plan.reason.startswith("BLOCKING_HOLD:NO_FEASIBLE_CANDIDATE")
    assert max(sample.target_speed_mps for sample in plan.samples) <= 2.0


def test_candidate_steering_rate_uses_actual_entry_speed_before_deceleration():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    original = supervisor._candidate
    checked = []

    def candidate(nominal, start_s, side, entry_speed=0.0, current_d=None, current_yaw=None):
        checked.append(entry_speed)
        return original(nominal, start_s, side, entry_speed, current_d, current_yaw)

    supervisor._candidate = candidate
    supervisor.update(samples, blue.x, blue.y, 5.0, prediction(supervisor), 10.0)

    assert checked == [5.0, 5.0]


def test_pass_side_accounts_for_lateral_occupancy_beyond_the_short_time_horizon():
    supervisor, samples = fixture()
    supervisor.left_widths = (2.0,) * len(samples)
    supervisor.right_widths = (1.5,) * len(samples)
    blue = supervisor.geometry.interpolate(0.0)
    left_biased = prediction(supervisor, ahead=4.0, lateral=0.25)
    left_biased = replace(left_biased, samples=tuple(replace(
        item, occupancy_half_width_m=0.33,
    ) for item in left_biased.samples))

    plan = supervisor.update(samples, blue.x, blue.y, 3.0, left_biased, 10.0)

    assert plan.valid and plan.side == "RIGHT"


def test_downward_speed_cap_does_not_keep_higher_speed_feedforward_acceleration():
    supervisor, samples = fixture()
    faster = tuple(replace(
        sample, target_speed_mps=7.0, target_accel_mps2=40.0,
    ) for sample in samples)

    candidate, _, reason = supervisor._candidate(faster, 0.0, "LEFT")

    assert reason == "VEHICLE_FEASIBLE"
    assert all(sample.target_speed_mps == supervisor.config.candidate_speed_cap_mps
               for sample in candidate)
    assert all(sample.target_accel_mps2 == 0.0 for sample in candidate)


def test_uncertainty_rejects_and_committed_loss_aborts_without_snapping():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    uncertain = prediction(supervisor)
    uncertain = replace(uncertain, samples=tuple(replace(sample, covariance_xy=(1.0, 0.0, 0.0, 1.0)) for sample in uncertain.samples))
    assert supervisor.update(samples, blue.x, blue.y, 3.0, uncertain, 10.0).reason == "PREDICTION_UNCERTAINTY_HIGH"
    supervisor, samples = fixture()
    committed = supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    assert supervisor.update(samples, blue.x, blue.y, 3.0, None, 10.1).state == "COMMITTED_PASS"
    aborted = supervisor.update(samples, blue.x, blue.y, 3.0, None, 10.7)
    assert aborted.state == "ABORT" and aborted.valid
    assert aborted.committed_at_s == 10.7
    assert all(sample.target_speed_mps <= 2.0 + 1e-9 for sample in aborted.samples)
    merging = supervisor.update(samples, blue.x, blue.y, 2.0,
                                prediction(supervisor, ahead=8.0, age=-0.8), 10.8)
    assert merging.state == "ABORT" and merging.reason.endswith(":MERGING")
    assert merging.committed_at_s == 10.7
    assert all(sample.target_speed_mps <= 2.0 for sample in merging.samples)
    assert any(sample.target_speed_mps > 1.0 for sample in merging.samples)
    merged = supervisor.geometry.from_frenet(supervisor.config.merge_length_m, 0.0)
    complete = supervisor.update(samples, merged.x, merged.y, 1.0, None, 10.9)
    assert complete.state == "COMPLETE" and not complete.valid


def test_prediction_loss_starts_a_bounded_abort_without_a_new_prediction():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    passing = supervisor.geometry.from_frenet(4.0, 0.75)
    supervisor.update(samples, passing.x, passing.y, 3.0,
                      prediction(supervisor, blue_s=4.0), 10.1)
    supervisor.update(samples, passing.x, passing.y, 3.0, None, 10.2)
    aborted = supervisor.update(samples, passing.x, passing.y, 3.0, None, 10.8)

    assert aborted.state == "ABORT" and aborted.reason.endswith(":MERGING")
    assert math.isclose(aborted.start_s, 4.0, abs_tol=0.05)
    assert math.isclose(
        supervisor.geometry.forward_delta_s(aborted.start_s, aborted.merge_end_s),
        supervisor.config.merge_length_m,
        abs_tol=1e-9,
    )
    assert all(sample.target_speed_mps <= 2.0 for sample in aborted.samples)
    abort_offsets = [abs(sample.lateral_offset_m) for sample in aborted.samples
                     if supervisor.geometry.forward_delta_s(aborted.start_s, sample.s_m)
                     <= supervisor.config.merge_length_m]
    assert all(a >= b for a, b in zip(abort_offsets, abort_offsets[1:]))
    assert all(math.isfinite(sample.heading_rad) and math.isfinite(sample.curvature_per_m)
               for sample in aborted.samples)
    assert math.isclose(
        min(aborted.samples, key=lambda sample: supervisor.geometry.forward_delta_s(
            aborted.start_s, sample.s_m,
        )).lateral_offset_m,
        0.75,
        abs_tol=0.1,
    )

    for progress in (1.0, 2.5, 4.0):
        d = 0.75 * (1.0 - (progress / supervisor.config.merge_length_m))
        point = supervisor.geometry.from_frenet(4.0 + progress, d)
        plan = supervisor.update(samples, point.x, point.y, 2.0, None, 10.8 + progress / 2.0)
        assert plan.state == "ABORT"

    merged = supervisor.geometry.from_frenet(
        4.0 + supervisor.config.merge_length_m, 0.0,
    )
    complete = supervisor.update(samples, merged.x, merged.y, 2.0, None, 13.3)
    assert complete.state == "COMPLETE" and not complete.valid
    released = supervisor.update(samples, merged.x, merged.y, 2.0, None, 13.4)
    assert released.state == "NOMINAL" and not released.valid


def test_abort_merge_does_not_release_nominal_into_too_close_opponent():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    committed = supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    supervisor.plan = replace(
        committed, state="ABORT", reason="COMMITTED_ABORT:TRACKING_ERROR_EXCEEDED:MERGING",
        start_s=4.0, merge_end_s=8.0,
    )
    merged = supervisor.geometry.from_frenet(8.0, 0.0)
    too_close = prediction(supervisor, blue_s=8.0, ahead=0.4, age=-1.0)

    contained = supervisor.update(samples, merged.x, merged.y, 2.0, too_close, 11.0)

    assert contained.valid and contained.state == "BLOCKING_HOLD"
    assert contained.reason == "BLOCKING_HOLD:ABORT_NOMINAL_UNSAFE"
    assert all(0.0 < sample.target_speed_mps <= 2.0 for sample in contained.samples)


def test_active_abort_merge_returns_to_lateral_hold_when_clearance_is_lost():
    supervisor, samples = fixture()
    blue_s, current_d = 4.0, 0.4
    blue = supervisor.geometry.from_frenet(blue_s, current_d)
    committed = supervisor.update(
        samples, supervisor.geometry.interpolate(0.0).x,
        supervisor.geometry.interpolate(0.0).y, 3.0, prediction(supervisor), 10.0,
    )
    merging, corridor, _ = supervisor._abort_candidate(samples, blue_s, current_d, 2.0)
    supervisor.plan = replace(
        committed, state="ABORT", reason="COMMITTED_ABORT:TRACKING_ERROR_EXCEEDED:MERGING",
        samples=merging, start_s=blue_s, merge_end_s=blue_s + supervisor.config.merge_length_m,
        minimum_corridor_clearance_m=corridor,
    )
    conflict = prediction(supervisor, blue_s=blue_s, ahead=0.5)
    conflict = replace(conflict, samples=tuple(
        replace(item, occupancy_half_length_m=2.0, occupancy_half_width_m=2.0)
        for item in conflict.samples
    ))

    held = supervisor.update(samples, blue.x, blue.y, 2.0, conflict, 10.1)

    assert held.state == "ABORT" and not held.reason.endswith(":MERGING")
    assert all(math.isclose(sample.lateral_offset_m, current_d, abs_tol=0.05)
               for sample in held.samples)


def test_committed_prediction_quality_and_temporal_safety_are_rechecked():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    degraded = replace(prediction(supervisor), confidence=0.1)
    assert supervisor._prediction_status(degraded, 10.1) == "TRACK_CONFIDENCE_LOW"
    assert supervisor.update(samples, blue.x, blue.y, 3.0, degraded, 10.1).state == "COMMITTED_PASS"
    assert supervisor.update(samples, blue.x, blue.y, 3.0, degraded, 10.7).reason == \
        "COMMITTED_ABORT:PREDICTION_STALE:MERGING"

    supervisor, samples = fixture()
    committed = supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    conflict = prediction(supervisor)
    conflict = replace(conflict, samples=tuple(
        replace(item, occupancy_half_length_m=100.0, occupancy_half_width_m=100.0)
        for item in conflict.samples
    ))
    assert supervisor.update(samples, blue.x, blue.y, 3.0, conflict, 10.1).state == "COMMITTED_PASS"
    conflict = replace(prediction(supervisor, age=-1.2), samples=tuple(
        replace(item, occupancy_half_length_m=100.0, occupancy_half_width_m=100.0)
        for item in prediction(supervisor, age=-1.2).samples
    ))
    aborted = supervisor.update(samples, blue.x, blue.y, 3.0, conflict, 10.7)
    assert committed.valid and aborted.reason == "COMMITTED_ABORT:TEMPORAL_CLEARANCE_LOST"
    assert all(sample.target_speed_mps <= 2.0 for sample in aborted.samples)


def test_temporal_clearance_abort_uses_moving_containment_then_merges():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    passing = supervisor.geometry.from_frenet(4.0, 0.75)
    supervisor.update(samples, passing.x, passing.y, 3.0,
                      prediction(supervisor, blue_s=4.0), 10.1)
    conflict = prediction(supervisor, blue_s=4.0, age=-1.2)
    conflict = replace(conflict, samples=tuple(
        replace(item, occupancy_half_length_m=100.0, occupancy_half_width_m=100.0)
        for item in conflict.samples
    ))
    supervisor.update(samples, passing.x, passing.y, 3.0, conflict, 10.2)
    held = supervisor.update(samples, passing.x, passing.y, 3.0, conflict, 10.8)

    assert held.state == "ABORT"
    assert all(0.0 < sample.target_speed_mps < 2.0 and sample.target_accel_mps2 == 0.0
               for sample in held.samples)
    assert math.isclose(
        abs(min(held.samples, key=lambda sample: supervisor.geometry.forward_delta_s(
            4.0, sample.s_m,
        )).lateral_offset_m),
        0.75,
        abs_tol=0.1,
    )
    assert supervisor.update(samples, passing.x, passing.y, 1.0, None, 10.9) == held

    merging = supervisor.update(
        samples, passing.x, passing.y, 1.0,
        prediction(supervisor, blue_s=4.0, ahead=8.0, age=-1.0), 11.0,
    )
    assert merging.reason.endswith(":MERGING")
    merged = supervisor.geometry.from_frenet(
        4.0 + supervisor.config.merge_length_m, 0.0,
    )
    complete = supervisor.update(samples, merged.x, merged.y, 1.0, None, 11.1)
    assert complete.state == "COMPLETE" and not complete.valid
    released = supervisor.update(samples, merged.x, merged.y, 1.0, None, 11.2)
    assert released.state == "NOMINAL" and not released.valid


def test_abort_hold_merges_when_the_candidate_is_safe_inside_eligibility_range():
    supervisor, samples = fixture()
    blue_s, current_d = 4.0, 0.75
    blue = supervisor.geometry.from_frenet(blue_s, current_d)
    committed = supervisor.update(
        samples, supervisor.geometry.interpolate(0.0).x,
        supervisor.geometry.interpolate(0.0).y, 3.0, prediction(supervisor), 10.0,
    )
    hold, corridor, _ = supervisor._temporal_hold_candidate(
        samples, blue_s, current_d, 2.0,
    )
    supervisor.plan = replace(
        committed, state="ABORT", reason="COMMITTED_ABORT:TRACKING_ERROR_EXCEEDED",
        samples=hold, start_s=blue_s, minimum_corridor_clearance_m=corridor,
    )
    red = prediction(supervisor, blue_s=blue_s, ahead=4.0)
    merge, _, _ = supervisor._abort_candidate(samples, blue_s, current_d, 2.0)
    assert supervisor._safe(merge, blue_s, 2.0, red)[0]

    result = supervisor.update(samples, blue.x, blue.y, 2.0, red, 10.1)

    assert result.reason.endswith(":MERGING")


def test_temporal_clearance_hold_is_anchored_to_the_current_lateral_state():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    passing = supervisor.geometry.from_frenet(4.0, 0.4)
    supervisor.update(samples, passing.x, passing.y, 3.0,
                      prediction(supervisor, blue_s=4.0), 10.1)
    conflict = prediction(supervisor, blue_s=4.0, age=-1.2)
    conflict = replace(conflict, samples=tuple(
        replace(item, occupancy_half_length_m=100.0, occupancy_half_width_m=100.0)
        for item in conflict.samples
    ))
    supervisor.update(samples, passing.x, passing.y, 3.0, conflict, 10.2)
    held = supervisor.update(samples, passing.x, passing.y, 3.0, conflict, 10.8)
    start = min(held.samples, key=lambda sample: supervisor.geometry.forward_delta_s(
        4.0, sample.s_m,
    ))
    assert math.isclose(start.lateral_offset_m, 0.4, abs_tol=0.05)
    assert all(math.isclose(sample.lateral_offset_m, 0.4, abs_tol=0.05)
               for sample in held.samples)


def test_temporal_clearance_loss_immediately_uses_a_safe_abort_path():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    passing = supervisor.geometry.from_frenet(4.0, 0.75)
    supervisor.update(samples, passing.x, passing.y, 3.0,
                      prediction(supervisor, blue_s=4.0), 10.1)
    supervisor._safe = lambda *args: ((True, 1.0) if len(args) == 4 else (False, 0.0))
    supervisor.update(samples, passing.x, passing.y, 3.0,
                      prediction(supervisor, blue_s=4.0), 10.2)
    aborted = supervisor.update(samples, passing.x, passing.y, 3.0,
                                prediction(supervisor, blue_s=4.0), 10.8)

    assert aborted.state == "ABORT" and aborted.reason.endswith(":MERGING")
    assert all(sample.target_speed_mps <= supervisor.last_opponent_speed_mps + 1e-9
               for sample in aborted.samples)


def test_confirmed_merge_continues_through_rear_blind_prediction_loss():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    ahead = supervisor.geometry.from_frenet(5.0, 0.75)
    merged = supervisor.update(samples, ahead.x, ahead.y, 3.0,
                               prediction(supervisor, ahead=4.0, age=-0.1), 10.1)
    merged = replace(merged, state="MERGE", committed_at_s=10.1)
    supervisor.plan = merged
    supervisor.parity_reached = supervisor.safe_ahead = True
    continued = supervisor.update(samples, ahead.x, ahead.y, 3.0, None, 10.2)
    assert continued.state == "MERGE"
    assert max(sample.target_speed_mps for sample in continued.samples) == 3.0


def test_committed_safety_uses_the_qualification_margin_as_hysteresis():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    committed = supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    threshold = supervisor.config.blue_tracking_uncertainty_m
    supervisor._safe = lambda *args: (True, threshold + 0.01) if args[-1] == 0.0 else (False, threshold + 0.01)
    assert supervisor.update(samples, blue.x, blue.y, 3.0,
                             prediction(supervisor), 10.1).state == "COMMITTED_PASS"
    assert committed.valid


def test_passing_tracking_error_aborts_before_reference_divergence_can_continue():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    committed = supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    supervisor.plan = replace(committed, state="PASSING")
    diverged = supervisor.geometry.from_frenet(4.0, 0.4)

    aborted = supervisor.update(
        samples, diverged.x, diverged.y, 3.0,
        prediction(supervisor, blue_s=4.0), 10.1,
    )

    assert aborted.state == "ABORT"
    assert aborted.reason == "COMMITTED_ABORT:TRACKING_ERROR_EXCEEDED:MERGING"
    start = min(aborted.samples, key=lambda sample: supervisor.geometry.forward_delta_s(
        4.0, sample.s_m,
    ))
    assert math.isclose(start.lateral_offset_m, 0.4, abs_tol=0.05)


def test_tracking_error_does_not_merge_through_predicted_opponent_occupancy():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    committed = supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    supervisor.plan = replace(committed, state="PASSING")
    diverged = supervisor.geometry.from_frenet(4.0, 0.4)
    blocked = prediction(supervisor, blue_s=4.0, ahead=0.8)

    aborted = supervisor.update(samples, diverged.x, diverged.y, 3.0, blocked, 10.1)

    assert aborted.state == "ABORT"
    assert not aborted.reason.endswith(":MERGING")
    start = min(aborted.samples, key=lambda sample: supervisor.geometry.forward_delta_s(
        4.0, sample.s_m,
    ))
    assert math.isclose(start.lateral_offset_m, 0.4, abs_tol=0.05)


def test_vehicle_steering_angle_and_rate_are_required_beyond_corridor_legality():
    supervisor, samples = fixture()
    legal, clearance, reason = supervisor._candidate(samples, 0.0, "LEFT")
    assert legal and clearance >= supervisor.config.corridor_clearance_m and reason == "VEHICLE_FEASIBLE"
    angle_limited = OvertakingSupervisor(
        replace(supervisor.config, steering_angle_max_rad=0.01),
        tuple(supervisor.geometry.points), [1.5] * len(samples), [1.5] * len(samples),
    )
    assert angle_limited._candidate(samples, 0.0, "LEFT")[2] == "STEERING_ANGLE_INFEASIBLE"
    rate_limited = OvertakingSupervisor(
        replace(supervisor.config, steering_rate_max_rad_s=0.001),
        tuple(supervisor.geometry.points), [1.5] * len(samples), [1.5] * len(samples),
    )
    assert rate_limited._candidate(samples, 0.0, "LEFT")[2] == "STEERING_RATE_INFEASIBLE"


def test_vehicle_feasibility_handles_zero_speed_and_rejects_nonfinite_geometry():
    supervisor, samples = fixture()
    stopped = tuple(replace(sample, target_speed_mps=0.0) for sample in samples)
    assert supervisor._steering_feasible(stopped) == (True, "VEHICLE_FEASIBLE")
    invalid = (replace(samples[0], curvature_per_m=math.inf),) + samples[1:]
    assert supervisor._steering_feasible(invalid) == (False, "STEERING_ANGLE_INFEASIBLE")


def test_temporal_safety_uses_oriented_blue_footprint_projection():
    supervisor, samples = fixture()
    samples = tuple(replace(sample, heading_rad=math.pi / 4.0) for sample in samples)
    blue = supervisor.geometry.interpolate(0.0)
    red = PredictionSample(
        0.1, blue.x, blue.y + 0.66, 0.0, 0.0, 0.0,
        (0.0, 0.0, 0.0, 0.0), 0.3, 0.3,
    )
    observed = replace(prediction(supervisor), samples=(red,))

    assert supervisor._safe(samples, 0.0, 0.0, observed)[0] is False


def test_corridor_clearance_uses_oriented_blue_footprint_projection():
    supervisor, samples = fixture()
    track_heading = supervisor.geometry.interpolate(samples[0].s_m).heading
    edge = replace(
        samples[0], lateral_offset_m=1.21,
        heading_rad=track_heading + math.pi / 4.0,
    )

    assert supervisor._corridor_clearance((edge,)) < supervisor.config.corridor_clearance_m


def test_no_safe_candidate_uses_moving_containment_not_stationary_obstacle():
    supervisor, samples = fixture()
    supervisor.config = replace(supervisor.config, qualification_margin_m=100.0)
    blue = supervisor.geometry.interpolate(0.0)
    observed = prediction(supervisor)
    plan = supervisor.update(samples, blue.x, blue.y, 3.0, observed, 10.0)
    assert plan.valid and plan.state == "BLOCKING_HOLD"
    assert plan.reason.startswith("BLOCKING_HOLD:")
    first, last = observed.samples[0], observed.samples[-1]
    red_speed = math.hypot(last.x_m - first.x_m, last.y_m - first.y_m) / (
        last.dt_s - first.dt_s
    )
    assert plan.samples and all(
        math.isclose(sample.target_speed_mps, red_speed) for sample in plan.samples
    )
    assert 0.0 < red_speed <= supervisor.config.candidate_speed_cap_mps
    assert supervisor.update(samples, blue.x, blue.y, 1.0, None, 10.2) == plan


def test_blocking_containment_rejects_nominal_release_while_red_closes_from_rear():
    supervisor, samples = fixture()
    original = supervisor.config
    supervisor.config = replace(original, qualification_margin_m=100.0)
    blue = supervisor.geometry.interpolate(0.0)
    plan = supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    supervisor.config = original
    rear = prediction(supervisor, ahead=-8.0, age=-15.0)

    continued = supervisor.update(samples, blue.x, blue.y, 0.0, rear, 25.0)
    assert continued.state == plan.state == "BLOCKING_HOLD"
    assert continued.plan_id == plan.plan_id and continued.samples == plan.samples
    clear = supervisor.update(samples, blue.x, blue.y, 2.0, rear, 25.1)
    assert clear.state == "COMPLETE" and not clear.valid


def test_v3_candidate_ledger_proves_left_and_right_were_executed():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)

    supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)

    assert set(supervisor.candidate_results) == {"LEFT", "RIGHT"}


def test_v3_parity_removes_opponent_cap_and_preserves_r_a_speed_profile():
    supervisor, samples = fixture()
    samples = tuple(replace(
        sample, target_speed_mps=7.0, target_accel_mps2=0.4,
    ) for sample in samples)
    blue = supervisor.geometry.interpolate(0.0)
    committed = supervisor.update(
        samples, blue.x, blue.y, 4.0, prediction(supervisor), 10.0,
    )
    supervisor.plan = replace(committed, state="PASSING")
    parity = supervisor.geometry.from_frenet(4.0, 0.75)

    released = supervisor.update(
        samples, parity.x, parity.y, 4.0,
        prediction(supervisor, blue_s=4.0, ahead=-0.2), 10.1,
    )

    assert supervisor.opponent_relation in ("PARITY", "AHEAD", "CLEAR")
    assert supervisor.speed_constraint_owner == "NONE"
    assert released.state == "PASSING"
    assert released.samples
    assert all(sample.target_speed_mps == 7.0 for sample in released.samples)


def test_v3_forensic_post_parity_prediction_loss_keeps_exact_accepted_payload():
    """Freeze the 191919Z failure without using evaluator truth as product input."""
    supervisor, samples = fixture()
    samples = tuple(replace(sample, target_speed_mps=5.0) for sample in samples)
    blue = supervisor.geometry.interpolate(0.0)
    committed = supervisor.update(
        samples, blue.x, blue.y, 4.0, prediction(supervisor), 10.0,
    )
    supervisor.plan = replace(committed, state="PASSING")
    parity = supervisor.geometry.from_frenet(4.0, 0.75)
    accepted = supervisor.update(
        samples, parity.x, parity.y, 4.0,
        prediction(supervisor, blue_s=4.0, ahead=-0.2), 10.1,
    )
    progressed = supervisor.geometry.from_frenet(4.4, 0.75)

    continued = supervisor.update(
        samples, progressed.x, progressed.y, 4.0, None, 10.2,
    )

    assert continued == accepted
    assert supervisor.opponent_relation in ("PARITY", "AHEAD", "CLEAR")
    assert supervisor.speed_constraint_owner == "NONE"
    assert min(sample.target_speed_mps for sample in continued.samples) > 2.0


def test_v3_follow_episode_reports_liveness_without_weakening_containment():
    supervisor, samples = fixture()
    supervisor.config = replace(supervisor.config, qualification_margin_m=100.0)
    blue = supervisor.geometry.interpolate(0.0)

    first = supervisor.update(samples, blue.x, blue.y, 3.0, prediction(supervisor), 10.0)
    repeated = supervisor.update(samples, blue.x, blue.y, 2.0, prediction(supervisor), 15.1)

    assert first.state == repeated.state == "BLOCKING_HOLD"
    assert supervisor.same_strategy_follow_duration_s >= 5.0
    assert repeated.reason.startswith("BLOCKING_HOLD:PASS_PLANNING_STALLED:")


def test_v3_fresh_parity_during_abort_exits_opponent_following_recovery():
    supervisor, samples = fixture()
    samples = tuple(replace(sample, target_speed_mps=5.0) for sample in samples)
    blue = supervisor.geometry.interpolate(0.0)
    committed = supervisor.update(
        samples, blue.x, blue.y, 4.0, prediction(supervisor), 10.0,
    )
    supervisor.plan = replace(
        committed, state="ABORT", reason="COMMITTED_ABORT:PREDICTION_INVALID",
        samples=tuple(replace(sample, target_speed_mps=1.0)
                      for sample in committed.samples),
    )
    ahead = supervisor.geometry.from_frenet(6.0, 0.75)

    recovered = supervisor.update(
        samples, ahead.x, ahead.y, 4.0,
        prediction(supervisor, blue_s=6.0, ahead=-2.0), 10.1,
    )

    assert recovered.state in ("PASSING", "MERGE")
    assert supervisor.opponent_relation == "CLEAR"
    assert supervisor.speed_constraint_owner == "NONE"
    assert min(sample.target_speed_mps for sample in recovered.samples) == 5.0


def test_v3_fresh_post_parity_collision_threat_applies_a_real_safety_limit():
    supervisor, samples = fixture()
    samples = tuple(replace(sample, target_speed_mps=5.0) for sample in samples)
    blue = supervisor.geometry.interpolate(0.0)
    committed = supervisor.update(
        samples, blue.x, blue.y, 4.0, prediction(supervisor), 10.0,
    )
    supervisor.plan = replace(committed, state="PASSING")
    parity = supervisor.geometry.from_frenet(4.0, 0.75)
    supervisor.update(
        samples, parity.x, parity.y, 4.0,
        prediction(supervisor, blue_s=4.0, ahead=-0.2), 10.1,
    )
    supervisor._safe = lambda *args: (False, 0.0)

    limited = supervisor.update(
        samples, parity.x, parity.y, 4.0,
        prediction(supervisor, blue_s=4.0, ahead=-0.2), 10.2,
    )

    assert limited.state == "ABORT"
    assert supervisor.speed_constraint_owner == "CURRENT_COLLISION_THREAT"
    assert max(sample.target_speed_mps for sample in limited.samples) <= 2.0


def test_v3_fresh_merge_collision_threat_cannot_bypass_the_safety_limit():
    supervisor, samples = fixture()
    samples = tuple(replace(sample, target_speed_mps=5.0) for sample in samples)
    blue = supervisor.geometry.interpolate(0.0)
    committed = supervisor.update(
        samples, blue.x, blue.y, 4.0, prediction(supervisor), 10.0,
    )
    supervisor.plan = replace(committed, state="PASSING")
    ahead = supervisor.geometry.from_frenet(6.0, 0.75)
    merged = supervisor.update(
        samples, ahead.x, ahead.y, 4.0,
        prediction(supervisor, blue_s=6.0, ahead=-2.0), 10.1,
    )
    assert merged.state == "MERGE"
    supervisor._safe = lambda *args: (False, 0.0)
    supervisor._rejoin_candidate = lambda *args: (
        None, -math.inf, "STEERING_RATE_INFEASIBLE",
    )

    limited = supervisor.update(
        samples, ahead.x, ahead.y, 4.0,
        prediction(supervisor, blue_s=6.0, ahead=-2.0), 10.2,
    )

    assert limited.state == "ABORT"
    assert supervisor.speed_constraint_owner == "CURRENT_COLLISION_THREAT"
    assert max(sample.target_speed_mps for sample in limited.samples) <= 2.0


def test_v3_pre_parity_pass_corridor_does_not_rejoin_or_complete_by_distance():
    supervisor, samples = fixture()
    blue = supervisor.geometry.interpolate(0.0)
    committed = supervisor.update(
        samples, blue.x, blue.y, 4.0, prediction(supervisor), 10.0,
    )
    supervisor._safe = lambda *args: (True, 1.0)
    still_behind = prediction(supervisor, blue_s=13.1, ahead=1.0, age=-0.1)
    progressed = supervisor.geometry.from_frenet(13.1, 0.75)

    plan = supervisor.update(
        samples, progressed.x, progressed.y, 4.0, still_behind, 10.1,
    )
    target_d = min(
        plan.samples,
        key=lambda sample: abs(supervisor.geometry.signed_delta_s(13.1, sample.s_m)),
    ).lateral_offset_m

    assert plan.state == "PASSING"
    assert supervisor.opponent_relation == "BEHIND"
    assert math.isclose(abs(target_d), 0.75, abs_tol=0.02)
