from dataclasses import dataclass
import hashlib
import json
import math

from f1tenth_racing_raceline import RacingTrajectory, validate_racing_trajectory


ACTIVE_SCHEMA_VERSION = 1
NOMINAL_SOURCE = "NOMINAL"
OVERTAKE_SOURCE = "OVERTAKE_OVERRIDE"


@dataclass(frozen=True)
class ActiveTargetTrajectory:
    trajectory: RacingTrajectory | None
    active_reference_id: str
    active_revision: int
    source_kind: str
    source_trajectory_id: str
    source_generation_method: str
    source_qualification: str
    source_product_valid: bool
    source_runtime_tracking_qualified: bool
    active_reference_valid: bool
    status_reason: str
    track_id: str
    frame_id: str
    map_image_sha256: str
    closed_loop: bool
    station_period_m: float
    path_length_m: float

    @property
    def sample_count(self):
        return len(self.trajectory.samples) if self.trajectory is not None else 0


def _active_reference_id(source_trajectory_id, source_kind=NOMINAL_SOURCE):
    identity = json.dumps(
        {
            "schema_version": ACTIVE_SCHEMA_VERSION,
            "source_kind": source_kind,
            "source_trajectory_id": source_trajectory_id,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    return "sha256:" + hashlib.sha256(identity).hexdigest()


def prediction_identity(prediction):
    source_time = prediction.source_observation_time
    identity = {
        "schema_version": prediction.schema_version,
        "frame_id": prediction.frame_id,
        "opponent_id": prediction.source_opponent_id,
        "predictor": prediction.predictor,
        "source_time_ns": source_time.sec * 1_000_000_000 + source_time.nanosec,
        "track_id": prediction.track_id,
        "track_revision": prediction.source_track_revision,
        "valid": prediction.valid,
        "lifecycle": prediction.source_lifecycle,
        "confidence": prediction.source_confidence,
        "hit_count": prediction.source_hit_count,
        "miss_count": prediction.source_miss_count,
        "horizon_s": prediction.horizon_s,
        "timestep_s": prediction.timestep_s,
        "samples": [[
            sample.dt_s, sample.x_m, sample.y_m, sample.vx_mps, sample.vy_mps,
            sample.heading_rad, sample.s_m, sample.d_m, list(sample.covariance_xy),
            sample.occupancy_half_length_m, sample.occupancy_half_width_m,
        ] for sample in prediction.samples],
    }
    return "sha256:" + hashlib.sha256(json.dumps(
        identity, sort_keys=True, separators=(",", ":"),
    ).encode()).hexdigest()


def overtaking_plan_identity(plan):
    identity = {
        "source_prediction_id": plan.source_prediction_id,
        "source_track_revision": plan.source_track_revision,
        "source_observation_time_ns": (plan.source_observation_time.sec * 1_000_000_000 +
                                       plan.source_observation_time.nanosec),
        "plan_revision": plan.plan_revision,
        "passing_side": plan.passing_side,
        "track_id": plan.track_id,
        "frame_id": plan.frame_id,
        "map_image_sha256": plan.map_image_sha256,
        "start_progress_m": plan.start_progress_m,
        "commit_end_progress_m": plan.commit_end_progress_m,
        "pass_end_progress_m": plan.pass_end_progress_m,
        "merge_end_progress_m": plan.merge_end_progress_m,
        "samples": [[getattr(sample, field) for field in (
            "sample_index", "s_m", "x_m", "y_m", "heading_rad", "curvature_per_m",
            "lateral_offset_m", "target_speed_mps", "target_accel_mps2", "ds_to_next_m",
        )] for sample in plan.samples],
    }
    return "sha256:" + hashlib.sha256(json.dumps(
        identity, sort_keys=True, separators=(",", ":"),
    ).encode()).hexdigest()


def override_expiry_action(plan, now_ns):
    valid_until_ns = plan.valid_until.sec * 1_000_000_000 + plan.valid_until.nanosec
    if now_ns < valid_until_ns:
        return "FRESH"
    continuation_ns = (plan.continuation_until.sec * 1_000_000_000 +
                       plan.continuation_until.nanosec)
    return "CONTINUE" if now_ns < continuation_ns else "DEGRADE"


def matching_complete(plan, accepted_plan):
    return bool(
        accepted_plan is not None and not plan.valid and
        plan.maneuver_state == "COMPLETE" and
        plan.plan_id == accepted_plan.plan_id and
        plan.plan_revision == accepted_plan.plan_revision and
        plan.plan_id == overtaking_plan_identity(plan)
    )


def select_override(nominal, plan, prediction, now_s, prediction_max_age_s=0.5,
                    committed_continuation_max_s=15.0, committed_plan=None):
    """Select an R-E payload without interpreting its maneuver strategy."""
    try:
        if plan.schema_version != 2 or not plan.valid:
            raise ValueError("override plan is invalid or unsupported")
        if plan.maneuver_state not in ("COMMITTED_PASS", "PASSING", "MERGE", "ABORT", "BLOCKING_HOLD"):
            raise ValueError("override lifecycle is not active")
        if plan.passing_side not in ("LEFT", "RIGHT") and not (
                plan.maneuver_state == "BLOCKING_HOLD" and plan.passing_side == "NONE"):
            raise ValueError("override passing side invalid")
        expected_safety = ("FAIL_CLOSED" if plan.maneuver_state in ("ABORT", "BLOCKING_HOLD")
                           else "TEMPORAL_SAFE")
        if plan.safety_state != expected_safety or not plan.plan_id:
            raise ValueError("override safety/identity invalid")
        # Once ABORT has actually been committed, a newly generated MERGE
        # cannot bypass physical parking and S0-R's Red-aware recovery gate.
        # Moving avoidance is allowed only BEFORE a committed ABORT exists.
        if (committed_plan is not None and
                committed_plan.maneuver_state == 'ABORT' and
                plan.maneuver_state != 'ABORT'):
            raise ValueError('ABORT_LATCHED_NEW_MERGE_REQUIRES_PARKED_RECOVERY')
        # Emergency moving rejoin is NOT committed continuation. It is a NEW
        # prediction-bound MERGE candidate, admitted only when the prior plan
        # was a live pass, source is fresher, and all normal payload/trajectory
        # checks below succeed. The independent Guard still has final veto.
        emergency_rejoin = bool(
            committed_plan is not None and
            committed_plan.maneuver_state in ('COMMITTED_PASS', 'PASSING') and
            plan.maneuver_state == 'MERGE' and
            str(plan.status_reason).startswith(
                'EMERGENCY_AVOIDANCE:FRESH_R_D_SAFE_REJOIN') and
            plan.plan_id != committed_plan.plan_id and
            plan.plan_revision > committed_plan.plan_revision and
            plan.source_observation_time.sec * 1_000_000_000 +
                plan.source_observation_time.nanosec >
                committed_plan.source_observation_time.sec * 1_000_000_000 +
                committed_plan.source_observation_time.nanosec
        )
        committed_abort = (
            committed_plan is not None and (
                plan.maneuver_state == "ABORT" or
                (plan.maneuver_state == "BLOCKING_HOLD" and
                 plan.plan_id == committed_plan.plan_id and
                 plan.plan_revision == committed_plan.plan_revision)
            )
        )
        lifecycle_order = {"COMMITTED_PASS": 0, "PASSING": 1, "MERGE": 2}
        committed_continuation = (
            committed_plan is not None and not emergency_rejoin and
            plan.maneuver_state in lifecycle_order and
            committed_plan.maneuver_state in lifecycle_order and
            lifecycle_order[plan.maneuver_state] >= lifecycle_order[committed_plan.maneuver_state] and
            plan.plan_id == committed_plan.plan_id
        )
        source_time_ns = (plan.source_observation_time.sec * 1_000_000_000 +
                          plan.source_observation_time.nanosec)
        generated_ns = plan.header.stamp.sec * 1_000_000_000 + plan.header.stamp.nanosec
        valid_until_ns = plan.valid_until.sec * 1_000_000_000 + plan.valid_until.nanosec
        continuation_ns = (plan.continuation_until.sec * 1_000_000_000 +
                           plan.continuation_until.nanosec)
        if not generated_ns < valid_until_ns or not generated_ns < continuation_ns:
            raise ValueError("override lifetime is incoherent")
        if continuation_ns - generated_ns > round(committed_continuation_max_s * 1_000_000_000):
            raise ValueError("override continuation lifetime exceeds authority")
        if committed_abort or committed_continuation:
            previous_time_ns = (committed_plan.source_observation_time.sec * 1_000_000_000 +
                                committed_plan.source_observation_time.nanosec)
            if (plan.plan_revision != committed_plan.plan_revision or
                    plan.source_prediction_id != committed_plan.source_prediction_id or
                    plan.source_track_revision != committed_plan.source_track_revision or
                    source_time_ns != previous_time_ns):
                raise ValueError("committed continuation provenance mismatch")
        else:
            if prediction is None or prediction.schema_version != 1 or not prediction.valid:
                raise ValueError("override source prediction unavailable")
            prediction_time_ns = (prediction.source_observation_time.sec * 1_000_000_000 +
                                  prediction.source_observation_time.nanosec)
            if not plan.source_prediction_id or plan.source_prediction_id != prediction_identity(prediction):
                raise ValueError("override source prediction identity mismatch")
            if plan.source_track_revision != prediction.source_track_revision:
                raise ValueError("override source prediction revision mismatch")
            if source_time_ns != prediction_time_ns:
                raise ValueError("override source observation time mismatch")
            if now_s - source_time_ns * 1e-9 > prediction_max_age_s:
                raise ValueError("override source prediction is stale")
            if prediction.track_id != plan.track_id or prediction.frame_id != plan.frame_id:
                raise ValueError("override source prediction track/frame mismatch")
        if source_time_ns > generated_ns:
            raise ValueError("override source observation time mismatch")
        if source_time_ns > round(now_s * 1_000_000_000):
            raise ValueError("override source observation is in the future")
        if plan.track_id != nominal.track_id or plan.frame_id != nominal.frame_id:
            raise ValueError("override track/frame mismatch")
        if plan.map_image_sha256 != nominal.map_image_sha256:
            raise ValueError("override map mismatch")
        if plan.sample_count != len(plan.samples) or plan.sample_count != nominal.sample_count:
            raise ValueError("override payload sample count mismatch")
        if not math.isclose(plan.station_period_m, nominal.station_period_m, abs_tol=1e-9):
            raise ValueError("override station period mismatch")
        if any(sample.sample_index != index or not math.isfinite(sample.s_m) for index, sample in enumerate(plan.samples)):
            raise ValueError("override sample ordering/numerics invalid")
        if any(a.s_m >= b.s_m for a, b in zip(plan.samples, plan.samples[1:])):
            raise ValueError("override stations are not increasing")
        if any(not math.isfinite(value) for sample in plan.samples for value in (
                sample.x_m, sample.y_m, sample.heading_rad, sample.curvature_per_m,
                sample.lateral_offset_m, sample.target_speed_mps, sample.target_accel_mps2,
                sample.ds_to_next_m)):
            raise ValueError("override payload contains non-finite values")
        if any(sample.target_speed_mps < 0.0 or sample.ds_to_next_m <= 0.0 for sample in plan.samples):
            raise ValueError("override speed/segment invalid")
        if plan.plan_id != overtaking_plan_identity(plan):
            raise ValueError("override plan content identity mismatch")
        from f1tenth_racing_raceline import RacingTrajectory, TrajectorySample
        trajectory = RacingTrajectory(
            tuple(TrajectorySample(**{
                field: getattr(sample, field) for field in (
                    "sample_index", "s_m", "x_m", "y_m", "heading_rad", "curvature_per_m",
                    "lateral_offset_m", "target_speed_mps", "target_accel_mps2", "ds_to_next_m",
                )
            }) for sample in plan.samples),
            {"trajectory_id": plan.plan_id},
        )
    except (AttributeError, TypeError, ValueError) as error:
        return invalid_active_reference(f"OVERRIDE_INVALID: {error}")
    return ActiveTargetTrajectory(
        trajectory=trajectory,
        active_reference_id=_active_reference_id(plan.plan_id, OVERTAKE_SOURCE),
        active_revision=plan.plan_revision,
        source_kind=OVERTAKE_SOURCE,
        source_trajectory_id=plan.plan_id,
        source_generation_method="R_E_FRENET_TEMPORAL_OVERRIDE",
        source_qualification="H2H_RUNTIME_SAFETY_CHECKED",
        source_product_valid=True,
        source_runtime_tracking_qualified=False,
        active_reference_valid=True,
        status_reason=plan.status_reason,
        track_id=plan.track_id,
        frame_id=plan.frame_id,
        map_image_sha256=plan.map_image_sha256,
        closed_loop=True,
        station_period_m=plan.station_period_m,
        path_length_m=plan.path_length_m,
    )


def invalid_active_reference(reason):
    return ActiveTargetTrajectory(
        trajectory=None,
        active_reference_id="",
        active_revision=0,
        source_kind=NOMINAL_SOURCE,
        source_trajectory_id="",
        source_generation_method="",
        source_qualification="",
        source_product_valid=False,
        source_runtime_tracking_qualified=False,
        active_reference_valid=False,
        status_reason=str(reason),
        track_id="",
        frame_id="",
        map_image_sha256="",
        closed_loop=False,
        station_period_m=0.0,
        path_length_m=0.0,
    )


def select_nominal(
    trajectory,
    expected_track_id="iccas2025",
    expected_frame_id="map",
    expected_map_image_sha256=None,
    expected_source_trajectory_id=None,
    allow_preliminary_spielberg=False,
):
    try:
        validate_racing_trajectory(trajectory)
        metadata = trajectory.metadata
        if metadata.get("track_id") != expected_track_id:
            raise ValueError("nominal trajectory track identity mismatch")
        if metadata.get("frame_id") != expected_frame_id:
            raise ValueError("nominal trajectory frame identity mismatch")
        preliminary = (metadata.get("qualification_state") ==
            "OFFLINE_CENTERLINE_PRELIMINARY_UNQUALIFIED")
        if preliminary:
            # An explicit experimental launch may transport this *unqualified*
            # reference. Never rewrite its source qualification or treat it
            # as a runtime certificate; Guard still decides every command.
            if not (allow_preliminary_spielberg and expected_track_id == 'spielberg' and
                    metadata.get('track_id') == 'spielberg' and
                    metadata.get('runtime_tracking_qualified') is False and
                    metadata.get('trajectory_tier') == 'UNQUALIFIED_EXPERIMENTAL'):
                raise ValueError('preliminary Spielberg requires explicit experimental admission')
        elif metadata.get("qualification_state") not in (
            "OFFLINE_BASELINE_QUALIFIED",
            "OFFLINE_PRIMARY_CANDIDATE_QUALIFIED",
        ):
            raise ValueError("nominal trajectory qualification is not eligible")
        map_hash = metadata.get("source", {}).get("map_image_sha256")
        if not map_hash:
            raise ValueError("nominal trajectory map identity missing")
        if expected_map_image_sha256 is not None and map_hash != expected_map_image_sha256:
            raise ValueError("nominal trajectory map identity mismatch")
        source_mismatch = expected_source_trajectory_id is not None
        source_mismatch = source_mismatch and (
            trajectory.trajectory_id != expected_source_trajectory_id
        )
        if source_mismatch:
            raise ValueError("nominal trajectory source identity mismatch")
    except (AttributeError, KeyError, TypeError, ValueError) as error:
        return invalid_active_reference(f"NOMINAL_SOURCE_INVALID: {error}")
    source_id = trajectory.trajectory_id
    return ActiveTargetTrajectory(
        trajectory=trajectory,
        active_reference_id=_active_reference_id(source_id),
        active_revision=1,
        source_kind=NOMINAL_SOURCE,
        source_trajectory_id=source_id,
        source_generation_method=metadata["generation_method"],
        source_qualification=metadata["qualification_state"],
        source_product_valid=metadata["product_valid"],
        source_runtime_tracking_qualified=metadata["runtime_tracking_qualified"],
        active_reference_valid=True,
        status_reason="NOMINAL_SELECTED",
        track_id=metadata["track_id"],
        frame_id=metadata["frame_id"],
        map_image_sha256=map_hash,
        closed_loop=metadata["closed_loop"],
        station_period_m=metadata["station_period_m"],
        path_length_m=metadata["path_length_m"],
    )


def validate_active_target(active):
    if active.source_kind not in (NOMINAL_SOURCE, OVERTAKE_SOURCE):
        raise ValueError("unsupported active source kind")
    if not active.active_reference_valid:
        if active.trajectory is not None:
            raise ValueError("invalid active reference must not carry a payload")
        return False
    if active.trajectory is None:
        raise ValueError("valid active reference has no payload")
    if active.source_kind == NOMINAL_SOURCE:
        validate_racing_trajectory(active.trajectory)
    if active.source_trajectory_id != active.trajectory.trajectory_id:
        raise ValueError("active/source trajectory identity mismatch")
    if active.active_reference_id != _active_reference_id(active.source_trajectory_id, active.source_kind):
        raise ValueError("active reference identity mismatch")
    if active.active_revision < 1 or active.sample_count != len(active.trajectory.samples):
        raise ValueError("active reference revision/sample count mismatch")
    if active.source_kind == NOMINAL_SOURCE:
        length_metadata_matches = (
            active.station_period_m == active.trajectory.metadata["station_period_m"]
        ) and active.path_length_m == active.trajectory.metadata["path_length_m"]
        if not length_metadata_matches:
            raise ValueError("active reference track-length metadata mismatch")
    if active.source_runtime_tracking_qualified is not False:
        raise ValueError("nominal source runtime qualification was relabeled")
    return True
