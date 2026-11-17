import csv
import json

from f1tenth_racing_telemetry.analyzer import LIMITING_OWNERS, analyze, limiting_owner


def row(nominal=8.0, active=8.0, requested=8.0, actual=8.0, source="NOMINAL"):
    return {
        "r_a_nominal_speed_mps": str(nominal), "s0_r_target_speed_mps": str(active),
        "r_b_requested_speed_mps": str(requested), "blue_actual_speed_mps": str(actual),
        "s0_r_source": source, "r_b_mode": "LOCAL_SPEED_CAPPED", "r_e_reason": "CLEARANCE",
    }


def test_limiting_owner_follows_authority_chain():
    assert limiting_owner(row(active=3.0, requested=3.0, source="OVERTAKE_OVERRIDE")) == "R_E_INTERACTION_LIMIT"
    assert limiting_owner(row(requested=3.0)) == "PROFILE_GLOBAL_CAP"
    assert limiting_owner(row(actual=3.0)) == "R_B_TRACKING_LIMIT"
    assert limiting_owner(row()) == "R_A_NOMINAL_LIMIT"
    assert limiting_owner({}) == "UNKNOWN"
    assert "VEHICLE_PHYSICS_LIMIT" in LIMITING_OWNERS


def test_analyze_writes_required_outputs_and_recovery_metrics(tmp_path):
    run = tmp_path / "run"
    (run / "analysis").mkdir(parents=True)
    fields = ["timestamp_s", "r_a_nominal_speed_mps", "s0_r_target_speed_mps",
              "r_b_requested_speed_mps", "blue_actual_speed_mps", "s0_r_source",
              "r_b_mode", "r_e_reason", "r_e_state", "r_b_acceleration_mps2"]
    samples = [
        dict(row(active=3.0, requested=3.0, actual=3.0, source="OVERTAKE_OVERRIDE"), timestamp_s="1", r_e_state="PASSING", r_b_acceleration_mps2="0"),
        dict(row(requested=3.0, actual=3.0), timestamp_s="2", r_e_state="COMPLETE", r_b_acceleration_mps2="0"),
        dict(row(actual=3.2), timestamp_s="3", r_e_state="NOMINAL", r_b_acceleration_mps2="1"),
    ]
    with (run / "telemetry.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(samples)
    (run / "events.jsonl").write_text("\n".join(json.dumps(event) for event in [
        {"timestamp_s": 1, "event": "PHYSICAL_PASS"},
        {"timestamp_s": 2, "event": "INTERACTION_CAP_RELEASED"},
    ]) + "\n")
    result = analyze(run)
    assert result["slowdowns"] == 3
    recovery = json.loads((run / "analysis/overtake_events.json").read_text())[0]
    assert recovery["pass_to_interaction_release_s"] == 1
    assert recovery["interaction_release_to_racing_reference_s"] == 0
    assert recovery["interaction_release_to_actual_acceleration_s"] == 1
    assert (run / "analysis/speed_authority_timeline.csv").exists()
    assert (run / "analysis/run_report.md").exists()
