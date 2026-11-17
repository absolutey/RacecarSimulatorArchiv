import math
from pathlib import Path
from types import SimpleNamespace

from f1tenth_racing_telemetry.hud import (
    SpectatorHud, active_reference_color, active_reference_points, active_target_speed,
    action_for, board_text, format_duration, opponent_estimate_point, opponent_status,
    prediction_points, prediction_status, short_reference_id, start_finish_points,
)
from f1tenth_racing_telemetry.review_validity import LapCollisionReview


def test_spectator_board_is_simple_and_hides_engineering_internals():
    text = board_text(2, 12.3, 11.8, 8.1, 2.0, 1, 3.2, 1, "OVERTAKING", True)
    for label in ("LAP", "BEST", "TOTAL", "BLUE", "RED", "POSITION", "GAP", "OVERTAKES", "RACE VALID"):
        assert label in text
    for hidden in ("revision", "covariance", "provenance", "sha256"):
        assert hidden not in text.lower()


def test_total_uses_race_epoch_while_current_starts_at_lap_one():
    hud = SpectatorHud.__new__(SpectatorHud)
    hud.laps = [0, 0]
    hud.elapsed = hud.lap_start_elapsed = 1.45
    hud.current_lap = hud.total_elapsed = 0.0
    hud.total_start_elapsed = None
    hud.review_validity = LapCollisionReview()

    hud.set_lap(0, 1)
    hud.elapsed_time(SimpleNamespace(data=18.026))
    assert math.isclose(hud.current_lap, 16.576)
    assert math.isclose(hud.total_elapsed, 18.026)
    assert format_duration(hud.total_elapsed) == "00:18.026"

    hud.valid = True
    hud.best_lap = 0.0
    hud.lap_time(SimpleNamespace(data=16.576))
    hud.elapsed_time(SimpleNamespace(data=20.0))
    assert math.isclose(hud.current_lap, 1.974)
    assert math.isclose(hud.total_elapsed, 20.0)


def test_invalid_lap_cannot_replace_best_or_reset_current():
    hud = SpectatorHud.__new__(SpectatorHud)
    hud.valid = False
    hud.best_lap = 8.95
    hud.elapsed = 100.0
    hud.lap_start_elapsed = 90.0

    hud.lap_time(SimpleNamespace(data=8.0))

    assert hud.best_lap == 8.95
    assert hud.lap_start_elapsed == 90.0


def test_action_mapping_is_spectator_facing():
    assert action_for("PASSING", True) == "OVERTAKING"
    assert action_for("MERGE", True) == "MERGING"
    assert action_for("COMPLETE", True) == "RACING"
    assert action_for("PASSING", False) == "INVALID"


def test_active_reference_visualization_preserves_reference_geometry():
    message = SimpleNamespace(samples=[
        SimpleNamespace(x_m=1.0, y_m=2.0), SimpleNamespace(x_m=3.0, y_m=4.0),
    ])
    # Active references are deliberately OPEN and do not append the first sample.
    assert active_reference_points(message) == [(1.0, 2.0, 0.025), (3.0, 4.0, 0.025)]
    assert active_reference_points(message, z=0.12) == [
        (1.0, 2.0, 0.12), (3.0, 4.0, 0.12)
    ]
    # Complete nominal line is blue; an active/local nominal line is dim gray.
    assert active_reference_color("NOMINAL") == (0.55, 0.55, 0.55)
    assert active_reference_color("OVERTAKE_OVERRIDE") == (1.0, 0.45, 0.0)


def test_full_lap_nominal_closes_without_corrupting_active_reference():
    from f1tenth_racing_telemetry.hud import closed_nominal_points

    samples = [SimpleNamespace(x_m=x, y_m=y) for x, y in
               ((1.0, 2.0), (3.0, 4.0), (2.0, 5.0))]
    trajectory = SimpleNamespace(metadata={"closed_loop": True}, samples=samples)
    full = closed_nominal_points(trajectory)
    assert full == [(1.0, 2.0, 0.085), (3.0, 4.0, 0.085),
                    (2.0, 5.0, 0.085), (1.0, 2.0, 0.085)]
    active = active_reference_points(SimpleNamespace(samples=samples))
    assert len(active) == 3 and active[0] != active[-1]
    # An OPEN trajectory must never masquerade as a full-lap racing line.
    import pytest
    with pytest.raises(ValueError, match="not a valid closed loop"):
        closed_nominal_points(SimpleNamespace(metadata={"closed_loop": False}, samples=samples))


def test_r_a_target_uses_uncapped_active_reference_and_short_identity():
    active = SimpleNamespace(
        active_reference_id="sha256:1234567890abcdef", station_period_m=10.0,
        samples=[
            SimpleNamespace(s_m=0.0, target_speed_mps=6.0),
            SimpleNamespace(s_m=5.0, target_speed_mps=8.0),
        ],
    )
    controller = SimpleNamespace(target_progress_m=2.5)
    assert math.isclose(active_target_speed(active, controller), 7.0)
    assert short_reference_id(active.active_reference_id) == "12345678"


def test_lap_validity_and_collision_episode_contract():
    review = LapCollisionReview()
    review.set_lap(1)
    assert review.lap_valid

    entered = review.update(wall=True)
    assert entered.kind == "ENTER" and entered.episode_id == 1
    assert not review.lap_valid
    assert review.update(wall=True) is None
    assert review.episode_id == 1

    exited = review.update(wall=False)
    assert exited.kind == "EXIT"
    review.update(wall=True)
    assert review.episode_id == 2

    review.set_lap(2)
    assert review.lap_valid
    assert review.episode_id == 2


def test_overlapping_collision_types_are_one_episode_with_distinct_type_counts():
    review = LapCollisionReview()
    review.update(wall=True)
    review.update(vehicle=True)
    assert review.episode_id == 1
    assert review.wall_episodes == 1
    assert review.vehicle_episodes == 1
    review.update(wall=False)
    assert review.update(vehicle=False).kind == "EXIT"


def test_phase_statuses_are_explicit_when_producers_are_inactive_or_invalid():
    assert opponent_status("FOUNDATION", None) == "INACTIVE"
    assert opponent_status("R-C", None) == "NO TARGET"
    lost = SimpleNamespace(valid=False, lifecycle="LOST")
    assert opponent_status("R-C", lost) == "LOST / INVALID"
    assert prediction_status("R-C", None) == "INACTIVE"
    assert prediction_status("R-D", None) == "NOT_AVAILABLE"


def test_r_c_and_r_d_geometry_uses_only_algorithm_messages():
    estimate = SimpleNamespace(x_m=1.5, y_m=-2.0)
    prediction = SimpleNamespace(samples=[
        SimpleNamespace(x_m=2.0, y_m=3.0), SimpleNamespace(x_m=4.0, y_m=5.0),
    ])
    assert opponent_estimate_point(estimate) == (1.5, -2.0)
    assert prediction_points(prediction) == [(2.0, 3.0, 0.12), (4.0, 5.0, 0.12)]


def test_hud_algorithm_overlays_do_not_read_red_truth():
    source = (Path(__file__).parents[1] / "f1tenth_racing_telemetry/hud.py").read_text()
    overlay = source[source.index("opponent_delete = Marker()"):
                     source.index("if self.active is not None:")]
    assert "self.opponent" in overlay
    assert "self.prediction" in overlay
    assert "self.red" not in overlay
    assert 'ActiveTargetTrajectory, "active_target_trajectory"' in source


def test_start_finish_visual_is_perpendicular_to_racing_heading():
    sample = SimpleNamespace(x_m=2.0, y_m=3.0, heading_rad=0.0)
    assert start_finish_points(sample) == [(2.0, 2.0, 0.06), (2.0, 4.0, 0.06)]
