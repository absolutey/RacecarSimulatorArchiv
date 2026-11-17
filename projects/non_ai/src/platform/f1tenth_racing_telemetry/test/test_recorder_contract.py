import ast
from pathlib import Path
from types import SimpleNamespace

from f1tenth_racing_telemetry.recorder import FlightRecorder, r_b_dynamic_fields
from f1tenth_racing_telemetry.review_validity import LapCollisionReview


def test_recorder_has_no_ros_control_output():
    source = Path(__file__).parents[1] / "f1tenth_racing_telemetry/recorder.py"
    tree = ast.parse(source.read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    assert not any(isinstance(call.func, ast.Attribute) and call.func.attr == "create_publisher"
                   for call in calls)


def test_prestart_events_and_control_rows_are_gated_by_common_start_time():
    class Writer:
        def __init__(self):
            self.events = []

        def event(self, *values):
            self.events.append(values)

    recorder = FlightRecorder.__new__(FlightRecorder)
    recorder.writer = Writer()
    recorder.start_time = None
    recorder.now = lambda: 10.0
    recorder.emit("R-B", "FALLBACK")
    recorder.receive_control(SimpleNamespace(receipt_time_ns=10_000_000_000))
    assert recorder.writer.events == []

    recorder.start_time = 11.0
    recorder.emit("R-B", "FALLBACK")
    recorder.receive_control(SimpleNamespace(receipt_time_ns=10_999_999_999))
    assert recorder.writer.events == []

    recorder.emit("COORDINATOR", "START", timestamp_s=11.0)
    assert recorder.writer.events[0][:2] == (11.0, 0.0)


def test_start_resets_prerace_duration_state_and_persists_authoritative_epoch():
    class Writer:
        start = None

        def set_start_time(self, value):
            self.start = value

        def event(self, *args):
            pass

    recorder = FlightRecorder.__new__(FlightRecorder)
    recorder.writer = Writer()
    recorder.start_time = None
    recorder.last_control_time = 9.0
    recorder.last_fallback = True
    recorder.follow_episode_started_s = 8.0
    recorder.command_gap_count = 4
    recorder.was_behind = True
    recorder.pass_count = recorder.lap = recorder.wall_collisions = 3
    recorder.last_lap_time = 2.0
    recorder.race_valid = False
    recorder.car_collision = True
    recorder.emit = lambda *args, **kwargs: None
    recorder.receive_start(SimpleNamespace(sec=12, nanosec=500_000_000))
    assert recorder.start_time == recorder.writer.start == 12.5
    assert recorder.last_control_time is None
    assert recorder.command_gap_count == recorder.pass_count == recorder.lap == 0
    assert recorder.race_valid and not recorder.car_collision
    assert recorder.review_validity.episode_id == 0


def test_recorder_collision_episode_is_edge_counted_and_lap_identity_is_preserved():
    recorder = FlightRecorder.__new__(FlightRecorder)
    recorder.start_time = 10.0
    recorder.now = iter((11.0, 11.1, 12.0)).__next__
    recorder.lap = 2
    recorder.review_validity = LapCollisionReview()
    recorder.collision_episodes = []

    recorder.update_collision_episode(wall=True)
    recorder.update_collision_episode(wall=True)
    recorder.update_collision_episode(wall=False)

    assert recorder.review_validity.episode_id == 1
    assert recorder.collision_episodes == [{
        "episode_id": 1, "enter_timestamp_s": 11.0, "enter_run_elapsed_s": 1.0,
        "lap": 2, "wall_collision": True, "vehicle_collision": False,
        "cumulative_wall_episodes": 1, "cumulative_vehicle_episodes": 0,
        "cumulative_total_episodes": 1, "exit_timestamp_s": 12.0,
        "exit_run_elapsed_s": 2.0,
    }]


def test_foundation_records_state_only_after_start_with_algorithm_fields_unavailable():
    class Writer:
        rows = []

        def telemetry(self, row):
            self.rows.append(row)

    recorder = FlightRecorder.__new__(FlightRecorder)
    recorder.writer = Writer()
    recorder.review_phase = "FOUNDATION"
    recorder.track = SimpleNamespace(project=lambda x, y: SimpleNamespace(s=x, d=y))
    recorder.red = None
    recorder.red_projection = None
    recorder.was_behind = False
    recorder.pass_count = 0
    recorder.emit = lambda *args: None
    recorder.lap = 0
    recorder.last_lap_time = 0.0
    recorder.race_valid = True
    recorder.wall_collisions = 0
    recorder.car_collision = False
    recorder.now = lambda: 10.0
    recorder.start_time = None
    state = SimpleNamespace(px=1.0, py=2.0, yaw=0.3, v=0.0)
    recorder.receive_blue(state)
    assert recorder.writer.rows == []
    recorder.start_time = 9.5
    recorder.receive_blue(state)
    assert recorder.writer.rows[0]["run_elapsed_s"] == 0.5
    assert "r_b_mode" not in recorder.writer.rows[0]


def test_bag_topic_set_records_blue_command_as_read_only_evidence():
    source = Path(__file__).parents[1] / "f1tenth_racing_telemetry/recorder.py"
    module = ast.parse(source.read_text())
    assignment = next(node for node in module.body if isinstance(node, ast.Assign)
                      and any(isinstance(target, ast.Name) and target.id == "BAG_TOPICS"
                              for target in node.targets))
    topics = ast.literal_eval(assignment.value)
    # The only product command recorded is Blue's raw command. Recording is
    # observational and MUST NOT add another control-command publisher.
    assert topics.count("ackermann_cmd0") == 1
    assert "ackermann_cmd1" not in topics
    assert "state0" in topics and "state1" in topics
    assert "active_target_trajectory" in topics
    assert "controller_status0" in topics
    assert len(topics) == len(set(topics))
    assert not any(isinstance(call, ast.Call)
                   and isinstance(call.func, ast.Attribute)
                   and call.func.attr == "create_publisher"
                   for call in ast.walk(module))


def test_pass_checks_reuse_state_callback_projections():
    class Track:
        def __init__(self):
            self.calls = 0

        def project(self, x, y):
            self.calls += 1
            return SimpleNamespace(s=x)

        def signed_delta_s(self, start, end):
            return end - start

    recorder = FlightRecorder.__new__(FlightRecorder)
    recorder.review_phase = "R-A"
    recorder.track = Track()
    recorder.blue_projection = recorder.red_projection = None
    recorder.was_behind = False
    recorder.pass_count = 0
    recorder.emit = lambda *args: None

    recorder.receive_blue(SimpleNamespace(px=0.0, py=0.0))
    recorder.receive_red(SimpleNamespace(px=2.0, py=0.0))
    recorder.check_pass()

    assert recorder.track.calls == 2


def test_recorder_captures_v2_maneuver_authority_fields():
    source = (Path(__file__).parents[1] / "f1tenth_racing_telemetry/recorder.py").read_text()
    for field in (
        "relative_progress_m", "maneuver_relation", "speed_authority_mode",
        "parity_reached", "safe_ahead", "merge_feasible", "merge_phase",
        "merge_progress_m", "selected_speed_authority_mps",
        "last_qualified_opponent_relation",
        "same_strategy_follow_duration_s", "left_candidate_result",
        "right_candidate_result",
    ):
        assert f'"{field}"' in source


def test_qualified_relation_comes_from_r_e_transition_not_raw_prediction_validity():
    source = (Path(__file__).parents[1] / "f1tenth_racing_telemetry/recorder.py").read_text()
    relation_block = source[source.index('reason = self.plan.status_reason'):
                            source.index('speed_authority_mode =')]
    assert "relative_progress >=" not in relation_block
    assert all(token in relation_block for token in (
        'reason.startswith((', '"POST_PARITY_"', '"REJOIN_PENDING:"',
        '"PASS_CONFIRMED"',
    ))


def test_finalization_ignores_repeated_ctrl_c():
    source = (Path(__file__).parents[1] / "f1tenth_racing_telemetry/recorder.py").read_text()
    assert "signal.signal(signal.SIGINT, signal.SIG_IGN)\n        node.close" in source


def test_r_b_dynamic_fields_are_direct_car_state_values_and_limit_derived():
    state = SimpleNamespace(vx=5.0, vy=0.5, omega=1.1, slip_angle=0.1,
                            ax=2.0, ay=6.0, steer=0.2, steer_vel=0.3)
    fields = r_b_dynamic_fields(state, 4.0, -40.0, 4.0, 40.0, 40.0)
    assert fields["blue_body_vx_mps"] == state.vx
    assert fields["blue_body_vy_mps"] == state.vy
    assert fields["blue_yaw_rate_rad_s"] == state.omega
    assert fields["blue_slip_angle_rad"] == state.slip_angle
    assert fields["blue_longitudinal_acceleration_mps2"] == state.ax
    assert fields["blue_lateral_acceleration_mps2"] == state.ay
    assert fields["blue_steering_angle_rad"] == state.steer
    assert fields["blue_steering_rate_rad_s"] == state.steer_vel
    assert fields["r_b_steering_saturated"]
    assert fields["r_b_acceleration_saturated"]
