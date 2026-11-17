import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


EPSILON_MPS = 0.1
LIMITING_OWNERS = {
    "R_A_NOMINAL_LIMIT", "PROFILE_GLOBAL_CAP", "R_E_INTERACTION_LIMIT",
    "S0_R_REFERENCE_LIMIT", "R_B_TRACKING_LIMIT", "VEHICLE_PHYSICS_LIMIT", "UNKNOWN",
}


def number(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def limiting_owner(row):
    nominal = number(row, "r_a_nominal_speed_mps")
    active = number(row, "s0_r_target_speed_mps")
    requested = number(row, "r_b_requested_speed_mps")
    actual = number(row, "blue_actual_speed_mps")
    source = row.get("s0_r_source", "")
    if not all(math.isfinite(value) for value in (nominal, active, requested, actual)):
        return "UNKNOWN"
    if nominal - active > EPSILON_MPS:
        return ("R_E_INTERACTION_LIMIT" if source == "OVERTAKE_OVERRIDE"
                else "S0_R_REFERENCE_LIMIT")
    if active - requested > EPSILON_MPS:
        return "PROFILE_GLOBAL_CAP"
    if requested - actual > max(0.5, 0.15 * requested):
        return "R_B_TRACKING_LIMIT"
    return "R_A_NOMINAL_LIMIT"


def analyze(run_dir):
    run_dir = Path(run_dir)
    rows = list(csv.DictReader((run_dir / "telemetry.csv").open()))
    events = [json.loads(line) for line in (run_dir / "events.jsonl").read_text().splitlines()]
    if not rows:
        raise ValueError("telemetry.csv contains no rows")
    analysis_dir = run_dir / "analysis"
    analysis_dir.mkdir(exist_ok=True)

    timeline = []
    for row in rows:
        owner = limiting_owner(row)
        timeline.append({
            "timestamp_s": row["timestamp_s"],
            "r_a_nominal_speed_mps": row.get("r_a_nominal_speed_mps", ""),
            "s0_r_active_speed_mps": row.get("s0_r_target_speed_mps", ""),
            "r_b_requested_speed_mps": row.get("r_b_requested_speed_mps", ""),
            "blue_actual_speed_mps": row.get("blue_actual_speed_mps", ""),
            "limiting_owner": owner,
            "limiting_reason": (row.get("r_e_reason", "") if owner == "R_E_INTERACTION_LIMIT"
                                else row.get("r_b_mode", "") if owner == "PROFILE_GLOBAL_CAP"
                                else ""),
            "r_e_state": row.get("r_e_state", ""),
        })
    timeline_path = analysis_dir / "speed_authority_timeline.csv"
    with timeline_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=timeline[0])
        writer.writeheader()
        writer.writerows(timeline)

    slowdowns = []
    start = 0
    for index in range(1, len(timeline) + 1):
        if index < len(timeline) and timeline[index]["limiting_owner"] == timeline[start]["limiting_owner"]:
            continue
        owner = timeline[start]["limiting_owner"]
        if owner != "R_A_NOMINAL_LIMIT":
            segment = rows[start:index]
            slowdowns.append({
                "start_s": float(segment[0]["timestamp_s"]),
                "end_s": float(segment[-1]["timestamp_s"]),
                "duration_s": float(segment[-1]["timestamp_s"]) - float(segment[0]["timestamp_s"]),
                "r_a_nominal_speed_mps": number(segment[0], "r_a_nominal_speed_mps"),
                "active_speed_mps": number(segment[0], "s0_r_target_speed_mps"),
                "actual_speed_mps": number(segment[0], "blue_actual_speed_mps"),
                "limiting_owner": owner,
                "limiting_reason": timeline[start]["limiting_reason"],
                "r_e_state": segment[0].get("r_e_state", ""),
                "release_reason": timeline[index]["limiting_owner"] if index < len(timeline) else "RUN_END",
            })
        start = index
    (analysis_dir / "slowdown_events.json").write_text(json.dumps(slowdowns, indent=2) + "\n")

    overtakes = []
    for event in (item for item in events if item["event"] == "PHYSICAL_PASS"):
        pass_time = float(event["timestamp_s"])
        release = next((float(item["timestamp_s"]) for item in events
                        if item["event"] == "INTERACTION_CAP_RELEASED"
                        and float(item["timestamp_s"]) >= pass_time), None)
        authority = next((float(row["timestamp_s"]) for row in rows
                          if release is not None and float(row["timestamp_s"]) >= release
                          and row.get("s0_r_source") == "NOMINAL"
                          and abs(number(row, "s0_r_target_speed_mps")
                                  - number(row, "r_a_nominal_speed_mps")) <= EPSILON_MPS), None)
        acceleration = next((float(row["timestamp_s"]) for row in rows
                             if release is not None and float(row["timestamp_s"]) >= release
                             and number(row, "r_b_acceleration_mps2") > EPSILON_MPS), None)
        overtakes.append({
            "pass_time_s": pass_time,
            "interaction_release_time_s": release,
            "racing_reference_time_s": authority,
            "actual_acceleration_time_s": acceleration,
            "pass_to_interaction_release_s": None if release is None else release - pass_time,
            "interaction_release_to_racing_reference_s": None if authority is None else authority - release,
            "interaction_release_to_actual_acceleration_s": None if acceleration is None else acceleration - release,
        })
    (analysis_dir / "overtake_events.json").write_text(json.dumps(overtakes, indent=2) + "\n")

    counts = {owner: sum(item["limiting_owner"] == owner for item in timeline)
              for owner in sorted({item["limiting_owner"] for item in timeline})}
    (analysis_dir / "run_report.md").write_text(
        "# Run speed-authority report\n\n"
        f"Rows: {len(rows)}\n\nSlowdown segments: {len(slowdowns)}\n\n"
        f"Physical passes analyzed: {len(overtakes)}\n\n"
        "Limiting-owner rows:\n" + "".join(f"- {key}: {value}\n" for key, value in counts.items())
    )
    files = sorted(path for path in run_dir.rglob("*") if path.is_file() and path.name != "SHA256SUMS")
    (run_dir / "SHA256SUMS").write_text("".join(
        f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(run_dir)}\n" for path in files
    ))
    return {"rows": len(rows), "slowdowns": len(slowdowns), "overtakes": len(overtakes), "owner_rows": counts}


def main():
    parser = argparse.ArgumentParser(description="Analyze one F1TENTH flight-recorder run")
    parser.add_argument("run_dir")
    args = parser.parse_args()
    print(json.dumps(analyze(args.run_dir), sort_keys=True))


if __name__ == "__main__":
    main()
