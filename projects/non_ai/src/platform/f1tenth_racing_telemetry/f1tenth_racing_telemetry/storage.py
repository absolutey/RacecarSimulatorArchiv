import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3


TELEMETRY_FIELDS = (
    "timestamp_s", "run_elapsed_s", "blue_x_m", "blue_y_m", "blue_yaw_rad", "blue_s_m", "blue_d_m",
    "blue_actual_speed_mps", "red_truth_x_m", "red_truth_y_m", "red_truth_speed_mps",
    "r_a_trajectory_id", "r_a_nominal_speed_mps", "r_a_nominal_accel_mps2",
    "s0_r_source", "s0_r_reference_id", "s0_r_revision", "s0_r_target_speed_mps",
    "s0_r_target_accel_mps2", "r_b_requested_speed_mps", "r_b_steering_rad",
    "r_b_acceleration_mps2", "r_b_lateral_error_m", "r_b_heading_error_rad",
    "r_b_mode", "r_b_fallback", "r_b_active_reference_id", "r_b_command_gap_s", "r_b_compute_us",
    "r_c_lifecycle", "r_c_confidence", "r_c_compute_us",
    "r_c_opponent_s_m", "r_c_opponent_speed_mps", "r_c_state_age_s", "r_d_prediction_id",
    "r_d_prediction_age_s", "r_d_prediction_valid", "r_d_opponent_s_m", "r_d_opponent_speed_mps",
    "r_d_max_covariance_m2", "r_d_compute_us", "r_e_state", "r_e_reason",
    "r_e_side", "r_e_plan_id", "r_e_compute_us",
    "r_e_revision", "r_e_min_predicted_clearance_m", "lap", "lap_time_s", "race_valid",
    "wall_collisions", "car_collision", "relative_progress_m", "maneuver_relation",
    "speed_authority_mode", "parity_reached", "safe_ahead", "merge_feasible",
    "merge_phase", "merge_progress_m", "selected_speed_authority_mps",
    "last_qualified_opponent_relation", "same_strategy_follow_duration_s",
    "left_candidate_result", "right_candidate_result",
)

TEXT_FIELDS = {
    "r_a_trajectory_id", "s0_r_source", "s0_r_reference_id", "r_b_mode",
    "r_b_active_reference_id", "r_c_lifecycle", "r_d_prediction_id", "r_e_state",
    "r_e_reason", "r_e_side", "r_e_plan_id", "maneuver_relation",
    "speed_authority_mode", "merge_phase", "last_qualified_opponent_relation",
    "left_candidate_result", "right_candidate_result",
}


def sqlite_value(value):
    if value == "":
        return None
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, bool):
        return int(value)
    return value


def sync_file(path):
    with Path(path).open("rb") as stream:
        os.fsync(stream.fileno())


class RunWriter:
    def __init__(self, run_dir, manifest):
        self.run_dir = Path(run_dir)
        self.analysis_dir = self.run_dir / "analysis"
        self.analysis_dir.mkdir(parents=True, exist_ok=False)
        self.telemetry_file = (self.run_dir / "telemetry.csv").open("w", newline="")
        self.events_file = (self.run_dir / "events.jsonl").open("w")
        self.csv = csv.DictWriter(self.telemetry_file, fieldnames=TELEMETRY_FIELDS)
        self.csv.writeheader()
        self.last_event_time = None
        self.manifest = dict(manifest)
        self.manifest_path = self.run_dir / "manifest.json"
        self._write_manifest()
        self.rows = 0
        self.events = 0
        self.first_run_elapsed_s = self.last_run_elapsed_s = None
        run_id = self.manifest["run_id"]
        self.sqlite_path = self.run_dir / f"{run_id}_phase_review.sqlite.bak"
        self.sqlite = sqlite3.connect(self.sqlite_path)
        self.sqlite.execute("PRAGMA journal_mode=DELETE")
        self.sqlite.execute("CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)")
        columns = ", ".join(
            f'"{field}" {"TEXT" if field in TEXT_FIELDS else "REAL"}'
            for field in TELEMETRY_FIELDS
        )
        self.sqlite.execute(f"CREATE TABLE telemetry (row_no INTEGER PRIMARY KEY, {columns})")
        self.sqlite.execute(
            "CREATE TABLE events (line_no INTEGER PRIMARY KEY, timestamp_s REAL, "
            "run_elapsed_s REAL, source_owner TEXT, event TEXT, reason TEXT, "
            "plan_id TEXT, revision INTEGER)"
        )
        self.sqlite.execute(
            "CREATE TABLE collision_episodes (episode_id INTEGER PRIMARY KEY, "
            "enter_timestamp_s REAL NOT NULL, enter_run_elapsed_s REAL NOT NULL, "
            "exit_timestamp_s REAL, exit_run_elapsed_s REAL, lap INTEGER NOT NULL, "
            "wall_collision INTEGER NOT NULL, vehicle_collision INTEGER NOT NULL, "
            "cumulative_wall_episodes INTEGER NOT NULL, "
            "cumulative_vehicle_episodes INTEGER NOT NULL, "
            "cumulative_total_episodes INTEGER NOT NULL)"
        )
        self.sqlite.execute(
            "INSERT INTO metadata VALUES (?, ?)",
            ("manifest_json", json.dumps(self.manifest, sort_keys=True)),
        )
        if self.manifest.get("review_phase") == "R-A":
            self.sqlite.execute(
                "CREATE TABLE r_a_trajectory (trajectory_id TEXT NOT NULL, "
                "sample_index INTEGER NOT NULL, s_m REAL NOT NULL, x_m REAL NOT NULL, "
                "y_m REAL NOT NULL, heading_rad REAL NOT NULL, curvature_per_m REAL NOT NULL, "
                "nominal_speed_mps REAL NOT NULL, nominal_acceleration_mps2 REAL NOT NULL, "
                "PRIMARY KEY (trajectory_id, sample_index))"
            )
            self.sqlite.execute(
                "CREATE TABLE r_a_runtime (row_no INTEGER PRIMARY KEY, timestamp_s REAL NOT NULL, "
                "run_elapsed_s REAL NOT NULL, trajectory_id TEXT, target_progress_m REAL, "
                "nominal_speed_mps REAL, nominal_acceleration_mps2 REAL, execution_cap_mps REAL, "
                "requested_speed_mps REAL, blue_actual_speed_mps REAL, capped INTEGER, "
                "lateral_error_m REAL, heading_error_rad REAL, steering_command_rad REAL, "
                "acceleration_command_mps2 REAL, command_gap_s REAL, controller_compute_us REAL)"
            )
        if self.manifest.get("review_phase") == "R-B":
            self.sqlite.execute(
                "CREATE TABLE r_b_runtime (row_no INTEGER PRIMARY KEY, timestamp_s REAL NOT NULL, "
                "run_elapsed_s REAL NOT NULL, controller_kind TEXT NOT NULL, active_reference_id TEXT, "
                "active_revision INTEGER, active_source TEXT, target_progress_m REAL, "
                "target_curvature_per_m REAL, r_a_target_speed_mps REAL, "
                "local_target_speed_mps REAL, local_speed_reason TEXT, requested_speed_mps REAL, "
                "raw_dynamic_limit_mps REAL, backward_reachable_limit_mps REAL, "
                "execution_envelope_mps REAL, effective_preview_distance_m REAL, "
                "required_braking_distance_m REAL, fallback_cap_mps REAL, "
                "bounded_fallback_target_mps REAL, primary_admission_state TEXT, "
                "feasibility_duration_s REAL, predicted_terminal_recoverable INTEGER, "
                "recoverability_state TEXT, recoverability_reason TEXT, "
                "recoverability_margin REAL, terminal_recoverability_margin REAL, "
                "predicted_terminal_x_m REAL, predicted_terminal_y_m REAL, "
                "predicted_terminal_yaw_rad REAL, predicted_terminal_speed_mps REAL, "
                "predicted_terminal_slip_rad REAL, predicted_terminal_yaw_rate_rad_s REAL, "
                "actual_speed_mps REAL, "
                "body_vx_mps REAL, body_vy_mps REAL, yaw_rate_rad_s REAL, slip_angle_rad REAL, "
                "longitudinal_acceleration_mps2 REAL, lateral_acceleration_mps2 REAL, "
                "steering_angle_rad REAL, steering_rate_rad_s REAL, lateral_error_m REAL, "
                "heading_error_rad REAL, steering_command_rad REAL, acceleration_command_mps2 REAL, "
                "steering_saturated INTEGER, acceleration_saturated INTEGER, controller_mode TEXT, "
                "fallback_active INTEGER, fallback_reason TEXT, recovery_active INTEGER, "
                "recovery_reason TEXT, command_gap_s REAL, controller_compute_us REAL, "
                "hard_valid INTEGER)"
            )
            self.sqlite.execute(
                "CREATE TABLE r_b_solver (row_no INTEGER PRIMARY KEY, timestamp_s REAL NOT NULL, "
                "run_elapsed_s REAL NOT NULL, controller_kind TEXT NOT NULL, formulation_id TEXT, "
                "horizon_steps INTEGER, timestep_s REAL, solve_status TEXT, iterations INTEGER, "
                "solve_time_us REAL, deadline_met INTEGER, objective REAL, "
                "constraint_residual REAL, fallback_invoked INTEGER, fallback_reason TEXT)"
            )

    def _write_manifest(self):
        self.manifest_path.write_text(
            json.dumps(self.manifest, sort_keys=True, indent=2) + "\n"
        )

    def set_start_time(self, timestamp_s):
        self.manifest["race_start_time_s"] = timestamp_s
        self.manifest["lifecycle"] = "RECORDING"
        self._write_manifest()

    def set_prearmed(self):
        self.sqlite.commit()
        self.manifest["lifecycle"] = "PREARMED"
        self._write_manifest()

    def r_a_trajectory(self, trajectory):
        self.sqlite.executemany(
            "INSERT INTO r_a_trajectory VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
            ((
                trajectory.trajectory_id, sample.sample_index, sample.s_m, sample.x_m,
                sample.y_m, sample.heading_rad, sample.curvature_per_m,
                sample.target_speed_mps, sample.target_accel_mps2,
            ) for sample in trajectory.samples),
        )

    def r_a_runtime(self, values):
        self.sqlite.execute(
            "INSERT INTO r_a_runtime VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (self.rows, *(
                sqlite_value(values.get(field)) for field in (
                    "timestamp_s", "run_elapsed_s", "r_a_trajectory_id",
                    "r_b_target_progress_m", "r_a_nominal_speed_mps",
                    "r_a_nominal_accel_mps2", "execution_speed_cap_mps",
                    "r_b_requested_speed_mps", "blue_actual_speed_mps",
                    "r_b_qualification_cap_active", "r_b_lateral_error_m",
                    "r_b_heading_error_rad", "r_b_steering_rad",
                    "r_b_acceleration_mps2", "r_b_command_gap_s", "r_b_compute_us",
                )
            )),
        )

    def r_b_runtime(self, values):
        fields = (
            "timestamp_s", "run_elapsed_s", "controller_kind", "s0_r_reference_id",
            "s0_r_revision", "s0_r_source", "r_b_target_progress_m",
            "r_b_target_curvature_per_m", "r_a_target_speed_mps",
            "r_b_local_target_speed_mps", "r_b_local_speed_reason",
            "r_b_requested_speed_mps",
            "r_b_raw_dynamic_limit_mps", "r_b_backward_reachable_limit_mps",
            "r_b_execution_envelope_mps", "r_b_effective_preview_distance_m",
            "r_b_required_braking_distance_m", "r_b_fallback_cap_mps",
            "r_b_bounded_fallback_target_mps", "r_b_primary_admission_state",
            "r_b_feasibility_duration_s", "r_b_predicted_terminal_recoverable",
            "r_b_recoverability_state", "r_b_recoverability_reason",
            "r_b_recoverability_margin", "r_b_terminal_recoverability_margin",
            "r_b_predicted_terminal_x_m", "r_b_predicted_terminal_y_m",
            "r_b_predicted_terminal_yaw_rad", "r_b_predicted_terminal_speed_mps",
            "r_b_predicted_terminal_slip_rad", "r_b_predicted_terminal_yaw_rate_rad_s",
            "blue_actual_speed_mps", "blue_body_vx_mps", "blue_body_vy_mps",
            "blue_yaw_rate_rad_s", "blue_slip_angle_rad",
            "blue_longitudinal_acceleration_mps2", "blue_lateral_acceleration_mps2",
            "blue_steering_angle_rad", "blue_steering_rate_rad_s",
            "r_b_lateral_error_m", "r_b_heading_error_rad", "r_b_steering_rad",
            "r_b_acceleration_mps2", "r_b_steering_saturated",
            "r_b_acceleration_saturated", "r_b_mode", "r_b_fallback",
            "r_b_fallback_reason", "r_b_recovery_active", "r_b_recovery_reason",
            "r_b_command_gap_s", "r_b_compute_us", "r_b_hard_valid",
        )
        placeholders = ", ".join("?" for _ in fields)
        self.sqlite.execute(
            f"INSERT INTO r_b_runtime VALUES (?, {placeholders})",
            (self.rows, *(sqlite_value(values.get(field)) for field in fields)),
        )

    def r_b_solver(self, values):
        if values.get("controller_kind") == "pursuit":
            return
        fields = (
            "timestamp_s", "run_elapsed_s", "controller_kind", "solver_formulation_id",
            "solver_horizon_steps", "solver_timestep_s", "r_b_primary_status",
            "r_b_solver_iterations", "r_b_primary_solve_us", "r_b_deadline_met",
            "r_b_solver_objective", "r_b_constraint_residual", "r_b_fallback",
            "r_b_fallback_reason",
        )
        placeholders = ", ".join("?" for _ in fields)
        self.sqlite.execute(
            f"INSERT INTO r_b_solver VALUES (?, {placeholders})",
            (self.rows, *(sqlite_value(values.get(field)) for field in fields)),
        )

    def telemetry(self, values):
        row = {field: values.get(field, "") for field in TELEMETRY_FIELDS}
        self.csv.writerow(row)
        self.rows += 1
        elapsed = sqlite_value(row["run_elapsed_s"])
        if self.first_run_elapsed_s is None:
            self.first_run_elapsed_s = elapsed
        self.last_run_elapsed_s = elapsed
        placeholders = ", ".join("?" for _ in TELEMETRY_FIELDS)
        columns = ", ".join(f'"{field}"' for field in TELEMETRY_FIELDS)
        self.sqlite.execute(
            f"INSERT INTO telemetry (row_no, {columns}) VALUES (?, {placeholders})",
            (self.rows, *(sqlite_value(row[field]) for field in TELEMETRY_FIELDS)),
        )

    def event(self, timestamp_s, run_elapsed_s, source, event, reason="", plan_id="", revision=0):
        if self.last_event_time is not None and timestamp_s < self.last_event_time:
            raise ValueError("event timestamps must be nondecreasing")
        self.last_event_time = timestamp_s
        record = {
            "timestamp_s": timestamp_s, "run_elapsed_s": run_elapsed_s,
            "source_owner": source, "event": event,
            "reason": reason, "plan_id": plan_id, "revision": revision,
        }
        self.events_file.write(json.dumps(record, sort_keys=True) + "\n")
        self.events_file.flush()
        self.events += 1
        self.sqlite.execute(
            "INSERT INTO events VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
            (self.events, timestamp_s, run_elapsed_s, source, event, reason, plan_id, revision),
        )

    def collision_episodes(self, episodes):
        self.sqlite.executemany(
            "INSERT INTO collision_episodes VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            ((episode["episode_id"], episode["enter_timestamp_s"],
              episode["enter_run_elapsed_s"], episode.get("exit_timestamp_s"),
              episode.get("exit_run_elapsed_s"), episode["lap"],
              int(episode["wall_collision"]), int(episode["vehicle_collision"]),
              episode["cumulative_wall_episodes"],
              episode["cumulative_vehicle_episodes"],
              episode["cumulative_total_episodes"])
             for episode in episodes),
        )

    def close(self, summary):
        if self.telemetry_file.closed:
            return
        self.telemetry_file.flush(); os.fsync(self.telemetry_file.fileno())
        self.events_file.flush(); os.fsync(self.events_file.fileno())
        self.telemetry_file.close()
        self.events_file.close()
        telemetry_duration = (
            None if self.first_run_elapsed_s is None or self.last_run_elapsed_s is None
            else self.last_run_elapsed_s - self.first_run_elapsed_s
        )
        summary = dict(
            summary, telemetry_rows=self.rows, event_rows=self.events,
            first_run_elapsed_s=self.first_run_elapsed_s,
            last_run_elapsed_s=self.last_run_elapsed_s,
            telemetry_duration_s=telemetry_duration,
        )
        self.manifest.update({
            "lifecycle": "FINALIZED",
            "termination_reason": summary.get("termination_reason", "UNKNOWN"),
            "finalized_unix_s": summary.get("finalized_unix_s"),
        })
        self._write_manifest()
        summary_path = self.run_dir / "summary.json"
        summary_path.write_text(
            json.dumps(summary, sort_keys=True, indent=2) + "\n"
        )
        self.sqlite.execute(
            "INSERT OR REPLACE INTO metadata VALUES (?, ?)",
            ("manifest_json", json.dumps(self.manifest, sort_keys=True)),
        )
        self.sqlite.execute(
            "INSERT OR REPLACE INTO metadata VALUES (?, ?)",
            ("summary_json", json.dumps(summary, sort_keys=True)),
        )
        self.sqlite.commit()
        integrity = self.sqlite.execute("PRAGMA integrity_check").fetchone()[0]
        self.sqlite.close()
        sync_file(self.sqlite_path)
        self.sqlite_path.chmod(0o444)
        sync_file(self.manifest_path)
        sync_file(summary_path)
        files = sorted(path for path in self.run_dir.rglob("*")
                       if path.is_file() and path.name != "SHA256SUMS")
        sums_path = self.run_dir / "SHA256SUMS"
        sums_path.write_text("".join(
            f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(self.run_dir)}\n"
            for path in files
        ))
        sync_file(sums_path)
        return {
            "telemetry_rows": self.rows,
            "event_rows": self.events,
            "sqlite_path": str(self.sqlite_path),
            "sqlite_integrity": integrity,
            "sha256_status": "PASS",
        }
