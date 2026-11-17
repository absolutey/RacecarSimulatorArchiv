from dataclasses import replace
import math

from f1tenth_racing_geometry import interpolate_periodic_angle, interpolate_periodic_scalar, wrap_angle
from f1tenth_racing_overtaking.core import OvertakingConfig, OvertakingSupervisor, TrajectorySample


def fixture():
    count, radius = 80, 10.0
    points = tuple(
        (radius * math.cos(2.0 * math.pi * index / count),
         radius * math.sin(2.0 * math.pi * index / count))
        for index in range(count)
    )
    ds = 2.0 * radius * math.sin(math.pi / count)
    samples = tuple(TrajectorySample(
        index, ds * index, x, y,
        2.0 * math.pi * index / count + math.pi / 2.0,
        1.0 / radius, 0.0, 5.0, 0.0, ds,
    ) for index, (x, y) in enumerate(points))
    config = OvertakingConfig(
        0.5, 0.6, 3, 4, 0.2, 0.8, 6.0, 0.25, 0.75,
        3.0, 7.0, 3.0, 0.2, 0.2, 0.08, 0.1, 0.05, 1.2,
        0.34, 4.0, 4.0, 3.0, 5.0, 1.0, 0.05,
    )
    return OvertakingSupervisor(config, points, [1.5] * count, [1.5] * count), samples


def entry(samples, start_s, track_length):
    stations = tuple(sample.s_m for sample in samples)
    d = interpolate_periodic_scalar(
        stations, tuple(sample.lateral_offset_m for sample in samples), start_s, track_length,
    )
    heading = interpolate_periodic_angle(
        stations, tuple(sample.heading_rad for sample in samples), start_s, track_length,
    )
    return d, heading


def test_forensic_safe_candidate_is_also_current_state_trackable():
    """Regression for 20260901T000418Z commit at 1788221071.953025818."""
    supervisor, samples = fixture()
    start_s = 3.5776648657382557
    current_d = 0.0074158618775248585
    track_heading = supervisor.geometry.interpolate(start_s).heading
    # The forensic vehicle was nearly aligned with the centerline at commit.
    current_yaw = track_heading + 0.028265586747061366

    candidate, clearance, reason = supervisor._candidate(
        samples, start_s, "LEFT", 4.705316600203514,
        current_d=current_d, current_yaw=current_yaw,
    )

    assert candidate is not None and clearance >= supervisor.config.corridor_clearance_m
    assert reason == "VEHICLE_FEASIBLE"
    candidate_d, candidate_heading = entry(candidate, start_s, supervisor.geometry.length)
    assert abs(candidate_d - current_d) < supervisor.config.blue_tracking_uncertainty_m
    assert abs(wrap_angle(candidate_heading - current_yaw)) < 0.357456
    assert math.isclose(
        supervisor._entry_slope(start_s, current_yaw),
        math.tan(wrap_angle(current_yaw - track_heading)),
    )


def test_entry_tangent_changes_smoothly_into_pass_corridor():
    supervisor, samples = fixture()
    start_s, current_d = 2.3, 0.12
    current_yaw = supervisor.geometry.interpolate(start_s).heading + 0.04

    candidate, _, _ = supervisor._candidate(
        samples, start_s, "LEFT", 4.0,
        current_d=current_d, current_yaw=current_yaw,
    )

    assert candidate is not None
    start_d, start_heading = entry(candidate, start_s, supervisor.geometry.length)
    assert abs(start_d - current_d) < supervisor.config.blue_tracking_uncertainty_m
    assert abs(wrap_angle(start_heading - current_yaw)) < 0.357456
    forward_d = [entry(candidate, start_s + delta, supervisor.geometry.length)[0]
                 for delta in (0.0, 0.5, 1.0, 2.0, 3.0)]
    assert all(a <= b for a, b in zip(forward_d, forward_d[1:]))
    assert math.isclose(forward_d[-1], supervisor.config.candidate_lateral_offset_m,
                        abs_tol=0.02)


def test_rejoin_and_abort_begin_at_current_state_without_a_second_jump():
    supervisor, samples = fixture()
    start_s, current_d = 4.1, 0.75
    current_yaw = supervisor.geometry.interpolate(start_s).heading - 0.03

    rejoin, _, _ = supervisor._rejoin_candidate(
        samples, start_s, current_d, supervisor.config.merge_length_m,
        current_yaw=current_yaw,
    )
    abort, _, _ = supervisor._abort_candidate(
        samples, start_s, current_d, 2.0, current_yaw=current_yaw,
    )

    for candidate in (rejoin, abort):
        assert candidate is not None
        candidate_d, candidate_heading = entry(candidate, start_s, supervisor.geometry.length)
        assert abs(candidate_d - current_d) < supervisor.config.blue_tracking_uncertainty_m
        assert abs(wrap_angle(candidate_heading - current_yaw)) < 0.357456


def test_existing_vehicle_feasibility_remains_the_entry_gate():
    supervisor, samples = fixture()
    supervisor.config = replace(supervisor.config, steering_rate_max_rad_s=0.01)
    start_s = 0.0
    yaw = supervisor.geometry.interpolate(start_s).heading

    candidate, _, reason = supervisor._candidate(
        samples, start_s, "LEFT", 4.0, current_d=0.0, current_yaw=yaw,
    )

    assert candidate is None
    assert reason == "STEERING_RATE_INFEASIBLE"


def test_each_side_independently_passes_the_entry_trackability_gate():
    supervisor, samples = fixture()
    supervisor.config = replace(supervisor.config, steering_rate_max_rad_s=0.2)
    yaw = supervisor.geometry.interpolate(0.0).heading + 0.2

    left = supervisor._candidate(
        samples, 0.0, "LEFT", 4.0, current_d=0.0, current_yaw=yaw,
    )
    right = supervisor._candidate(
        samples, 0.0, "RIGHT", 4.0, current_d=0.0, current_yaw=yaw,
    )

    assert left[0] is not None and left[2] == "VEHICLE_FEASIBLE"
    assert right[0] is None and right[2] == "STEERING_RATE_INFEASIBLE"


def test_neither_side_is_committable_when_both_entries_are_untrackable():
    supervisor, samples = fixture()
    supervisor.config = replace(supervisor.config, steering_rate_max_rad_s=0.01)
    yaw = supervisor.geometry.interpolate(0.0).heading

    results = [supervisor._candidate(
        samples, 0.0, side, 4.0, current_d=0.0, current_yaw=yaw,
    ) for side in ("LEFT", "RIGHT")]

    assert all(candidate is None for candidate, _, _ in results)
    assert {reason for _, _, reason in results} == {"STEERING_RATE_INFEASIBLE"}
