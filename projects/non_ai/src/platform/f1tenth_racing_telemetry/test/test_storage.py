import csv
import hashlib
import json
import sqlite3
from types import SimpleNamespace

from f1tenth_racing_telemetry.storage import RunWriter


def test_run_writer_flushes_manifest_rows_events_summary_and_hashes(tmp_path):
    run = tmp_path / "run"
    writer = RunWriter(run, {"run_id": "R-A-TEST", "review_phase": "R-A",
                             "head": "abc", "truth_fields": "EVALUATOR_ONLY_TRUTH"})
    writer.r_a_trajectory(SimpleNamespace(
        trajectory_id="trajectory", samples=[SimpleNamespace(
            sample_index=0, s_m=0.0, x_m=1.0, y_m=2.0, heading_rad=0.3,
            curvature_per_m=0.1, target_speed_mps=7.0, target_accel_mps2=0.5,
        )],
    ))
    writer.set_start_time(1.0)
    writer.telemetry({"timestamp_s": 1.0, "run_elapsed_s": 0.0,
                      "r_b_requested_speed_mps": 3.0, "r_c_confidence": float("nan")})
    writer.r_a_runtime({
        "timestamp_s": 1.0, "run_elapsed_s": 0.0, "r_a_trajectory_id": "trajectory",
        "r_b_target_progress_m": 0.0, "r_a_nominal_speed_mps": 7.0,
        "r_a_nominal_accel_mps2": 0.5, "execution_speed_cap_mps": 5.0,
        "r_b_requested_speed_mps": 5.0, "blue_actual_speed_mps": 4.9,
        "r_b_qualification_cap_active": True,
    })
    writer.event(1.0, 0.0, "R-E", "COMMIT", "SAFE", "plan", 2)
    receipt = writer.close({"valid": True, "termination_reason": "USER_CTRL_C"})
    assert list(csv.DictReader((run / "telemetry.csv").open()))[0]["r_b_requested_speed_mps"] == "3.0"
    assert json.loads((run / "events.jsonl").read_text())["event"] == "COMMIT"
    summary = json.loads((run / "summary.json").read_text())
    assert summary["telemetry_rows"] == 1
    assert summary["first_run_elapsed_s"] == summary["last_run_elapsed_s"] == 0.0
    assert summary["telemetry_duration_s"] == 0.0
    assert json.loads((run / "manifest.json").read_text())["termination_reason"] == "USER_CTRL_C"
    bak = run / "R-A-TEST_phase_review.sqlite.bak"
    database = sqlite3.connect(f"file:{bak}?mode=ro", uri=True)
    assert database.execute("PRAGMA integrity_check").fetchone()[0] == "ok"
    assert database.execute(
        "SELECT run_elapsed_s, r_b_requested_speed_mps, r_c_confidence FROM telemetry"
    ).fetchone() == (0.0, 3.0, None)
    assert database.execute("SELECT run_elapsed_s, event FROM events").fetchone() == (0.0, "COMMIT")
    assert database.execute(
        "SELECT trajectory_id, nominal_speed_mps FROM r_a_trajectory"
    ).fetchone() == ("trajectory", 7.0)
    assert database.execute(
        "SELECT execution_cap_mps, requested_speed_mps, capped FROM r_a_runtime"
    ).fetchone() == (5.0, 5.0, 1)
    database.close()
    assert receipt["sqlite_integrity"] == "ok"
    assert bak.stat().st_mode & 0o222 == 0
    assert json.loads((run / "manifest.json").read_text())["lifecycle"] == "FINALIZED"
    for line in (run / "SHA256SUMS").read_text().splitlines():
        digest, relative = line.split("  ", 1)
        assert hashlib.sha256((run / relative).read_bytes()).hexdigest() == digest


def test_event_timestamps_must_be_ordered(tmp_path):
    writer = RunWriter(tmp_path / "run", {"run_id": "FOUNDATION-TEST",
                                           "review_phase": "FOUNDATION"})
    writer.event(2.0, 1.0, "R-E", "PASSING")
    try:
        writer.event(1.0, 0.0, "R-E", "COMMIT")
    except ValueError as error:
        assert "nondecreasing" in str(error)
    else:
        raise AssertionError("out-of-order event accepted")
    writer.close({})


def test_r_b_tables_preserve_dynamic_and_solver_semantics(tmp_path):
    run = tmp_path / "run"
    writer = RunWriter(run, {"run_id": "R-B-TEST", "review_phase": "R-B"})
    writer.telemetry({"timestamp_s": 2.0, "run_elapsed_s": 1.0})
    values = {
        "timestamp_s": 2.0, "run_elapsed_s": 1.0, "controller_kind": "mpc",
        "s0_r_reference_id": "ref", "s0_r_revision": 3, "s0_r_source": "NOMINAL",
        "r_b_target_progress_m": 4.0, "r_b_target_curvature_per_m": 0.2,
        "r_a_target_speed_mps": 8.0, "r_b_local_target_speed_mps": 6.0,
        "r_b_requested_speed_mps": 6.0, "blue_actual_speed_mps": 5.9,
        "r_b_raw_dynamic_limit_mps": 7.0,
        "r_b_backward_reachable_limit_mps": 6.2,
        "r_b_execution_envelope_mps": 6.0,
        "r_b_effective_preview_distance_m": 8.0,
        "r_b_required_braking_distance_m": 8.0,
        "r_b_fallback_cap_mps": 5.6,
        "r_b_bounded_fallback_target_mps": 5.4,
        "r_b_primary_admission_state": "PRIMARY_ADMITTED",
        "r_b_feasibility_duration_s": 0.1,
        "r_b_predicted_terminal_recoverable": True,
        "r_b_recoverability_state": "RECOVERABLE",
        "r_b_recoverability_reason": "B0_TERMINAL_MARGIN_NONDECREASING",
        "r_b_recoverability_margin": 0.2,
        "r_b_terminal_recoverability_margin": 0.3,
        "r_b_predicted_terminal_x_m": 1.0,
        "r_b_predicted_terminal_y_m": 2.0,
        "r_b_predicted_terminal_yaw_rad": 0.1,
        "r_b_predicted_terminal_speed_mps": 5.5,
        "r_b_predicted_terminal_slip_rad": 0.05,
        "r_b_predicted_terminal_yaw_rate_rad_s": 1.0,
        "blue_body_vx_mps": 5.8, "blue_body_vy_mps": 0.4,
        "blue_yaw_rate_rad_s": 1.2, "blue_slip_angle_rad": 0.07,
        "blue_longitudinal_acceleration_mps2": 1.0,
        "blue_lateral_acceleration_mps2": 7.0, "blue_steering_angle_rad": 0.1,
        "blue_steering_rate_rad_s": 0.2, "r_b_lateral_error_m": 0.03,
        "r_b_heading_error_rad": 0.04, "r_b_steering_rad": 0.11,
        "r_b_acceleration_mps2": 1.1, "r_b_steering_saturated": False,
        "r_b_acceleration_saturated": False, "r_b_mode": "PRIMARY_VALID",
        "r_b_fallback": False, "r_b_fallback_reason": "",
        "r_b_recovery_active": False, "r_b_recovery_reason": "",
        "r_b_command_gap_s": 0.025, "r_b_compute_us": 3200.0,
        "solver_formulation_id": "LTV_PACEJKA_ADMM_V1", "solver_horizon_steps": 8,
        "solver_timestep_s": 0.025, "r_b_primary_status": "PRIMARY_VALID",
        "r_b_solver_iterations": 12, "r_b_primary_solve_us": 3000.0,
        "r_b_deadline_met": True, "r_b_solver_objective": float("nan"),
        "r_b_constraint_residual": float("nan"),
        "r_b_hard_valid": True,
    }
    writer.r_b_runtime(values)
    writer.r_b_solver(values)
    writer.close({"valid": True})
    database = sqlite3.connect(f"file:{run / 'R-B-TEST_phase_review.sqlite.bak'}?mode=ro",
                               uri=True)
    assert database.execute(
        "SELECT body_vx_mps, body_vy_mps, slip_angle_rad, controller_mode "
        "FROM r_b_runtime"
    ).fetchone() == (5.8, 0.4, 0.07, "PRIMARY_VALID")
    assert database.execute(
        "SELECT execution_envelope_mps, bounded_fallback_target_mps, "
        "primary_admission_state, recoverability_state, recoverability_margin, "
        "hard_valid FROM r_b_runtime"
    ).fetchone() == (6.0, 5.4, "PRIMARY_ADMITTED", "RECOVERABLE", 0.2, 1)
    assert database.execute(
        "SELECT formulation_id, horizon_steps, deadline_met, objective, constraint_residual "
        "FROM r_b_solver"
    ).fetchone() == ("LTV_PACEJKA_ADMM_V1", 8, 1, None, None)
    database.close()


def test_b0_has_runtime_row_but_no_fake_solver_row(tmp_path):
    run = tmp_path / "run"
    writer = RunWriter(run, {"run_id": "R-B-B0", "review_phase": "R-B"})
    writer.telemetry({"timestamp_s": 1.0, "run_elapsed_s": 0.0})
    values = {"timestamp_s": 1.0, "run_elapsed_s": 0.0,
              "controller_kind": "pursuit"}
    writer.r_b_runtime(values)
    writer.r_b_solver(values)
    writer.close({})
    database = sqlite3.connect(f"file:{run / 'R-B-B0_phase_review.sqlite.bak'}?mode=ro",
                               uri=True)
    assert database.execute("SELECT COUNT(*) FROM r_b_runtime").fetchone()[0] == 1
    assert database.execute("SELECT COUNT(*) FROM r_b_solver").fetchone()[0] == 0
    database.close()


def test_collision_episode_table_preserves_enter_exit_and_type_counts(tmp_path):
    run = tmp_path / "run"
    writer = RunWriter(run, {"run_id": "R-B-COLLISION", "review_phase": "R-B"})
    writer.collision_episodes([{
        "episode_id": 1, "enter_timestamp_s": 10.0, "enter_run_elapsed_s": 2.0,
        "exit_timestamp_s": 11.0, "exit_run_elapsed_s": 3.0, "lap": 2,
        "wall_collision": True, "vehicle_collision": False,
        "cumulative_wall_episodes": 1, "cumulative_vehicle_episodes": 0,
        "cumulative_total_episodes": 1,
    }])
    writer.close({})
    database = sqlite3.connect(
        f"file:{run / 'R-B-COLLISION_phase_review.sqlite.bak'}?mode=ro", uri=True
    )
    assert database.execute(
        "SELECT episode_id, lap, wall_collision, vehicle_collision, "
        "exit_run_elapsed_s FROM collision_episodes"
    ).fetchone() == (1, 2, 1, 0, 3.0)
    database.close()
