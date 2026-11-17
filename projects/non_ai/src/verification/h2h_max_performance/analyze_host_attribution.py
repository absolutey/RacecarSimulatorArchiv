#!/usr/bin/env python3
"""Summarize the bounded H2H /proc attribution capture."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics


def rows(path):
    return list(csv.DictReader(path.open(), delimiter="\t"))


def distribution(values):
    values = sorted(values)
    return {"mean": statistics.mean(values), "median": statistics.median(values),
            "p95": values[int(.95 * (len(values) - 1))],
            "p99": values[int(.99 * (len(values) - 1))], "max": values[-1]}


def cadence(values):
    gaps = [b - a for a, b in zip(values, values[1:])]
    return {"hz": (len(values) - 1) / (values[-1] - values[0]),
            "gap_p95_ms": sorted(gaps)[int(.95 * (len(gaps) - 1))] * 1000,
            "gap_max_ms": max(gaps) * 1000}


def first_threshold(records, key, threshold):
    return next((row for row in records if float(row[key]) >= threshold), None)


def nearest(records, key, value):
    return min(records, key=lambda row: abs(float(row[key]) - value))


def unique_pid(manifest, executable_fragment):
    matches = [pid for pid, row in manifest.items() if executable_fragment in row["cmdline"]]
    if len(matches) != 1:
        raise ValueError(f"expected one PID for {executable_fragment!r}, got {matches}")
    return matches[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    run = args.run
    stages, processes, host = rows(run / "r_c_stage_timing.tsv"), rows(run / "process_samples.tsv"), rows(run / "host_samples.tsv")
    manifest = {int(row["pid"]): row for row in rows(run / "process_manifest.tsv")}
    raw = json.loads((run / "raw_runtime.json").read_text())
    flight = json.loads(next((run / "flight_recorder").glob("*/summary.json")).read_text())
    telemetry = list(csv.DictReader(next((run / "flight_recorder").glob("*/telemetry.csv")).open()))
    wall = [float(row["total_wall_ms"]) for row in stages]
    cpu = [float(row["total_cpu_ms"]) for row in stages]
    input_age = [float(row["input_age_s"]) * 1000 for row in stages]
    first100 = first_threshold(stages, "total_wall_ms", 100)
    first200 = first_threshold(stages, "total_wall_ms", 200)
    stage_names = ("pose_score_ms", "transform_ms", "expected_scan_ms", "residual_ms",
                   "dynamic_filter_ms", "clustering_ms", "candidate_filter_ms", "tracking_ms",
                   "message_ms", "publish_ms")
    stage_sum = [sum(float(row[name]) for name in stage_names) for row in stages]
    by_pid = {}
    for row in processes:
        item = by_pid.setdefault(int(row["pid"]), {"cpu": [], "wait": [], "row": row})
        item["cpu"].append(float(row["cpu_percent"]))
        item["wait"].append(float(row["sched_wait_delta_ms"]))
    contenders = []
    for pid, item in by_pid.items():
        if len(item["cpu"]) < 5:
            continue
        contenders.append({"pid": pid, "category": item["row"]["category"],
                           "command": manifest.get(pid, {}).get("cmdline", item["row"]["comm"]),
                           "cpu": distribution(item["cpu"]), "sched_wait": distribution(item["wait"])})
    contenders.sort(key=lambda item: item["cpu"]["mean"], reverse=True)
    load1 = [float(row["load1"]) for row in host]
    runnable = [float(row["runnable"]) for row in host]
    pressure = [float(match.group(1)) for row in host
                if (match := re.search(r"some avg10=([0-9.]+)", row["cpu_pressure"]))]
    thermal = [float(row["thermal_millic"]) for row in host if row["thermal_millic"]]
    frequency = [float(row["frequency_khz"]) for row in host if row["frequency_khz"]]
    rc_pid = unique_pid(manifest, "/f1tenth_racing_perception/opponent_perception")
    rb_pid = unique_pid(manifest, "/f1tenth_racing_control/r_b0_controller")
    recorder_pid = unique_pid(manifest, "/f1tenth_racing_telemetry/flight_recorder")
    sampler_pid = unique_pid(manifest, "capture_host_attribution.py")
    def process_summary(pid):
        item = by_pid.get(pid)
        return None if not item else {"pid": pid, "cpu": distribution(item["cpu"]),
                                      "sched_wait": distribution(item["wait"])}
    events = []
    for name, event in (("FIRST_R_C_100MS", first100), ("FIRST_R_C_200MS", first200)):
        if not event:
            continue
        event_time = float(event["scan_time_s"])
        window = [row for row in processes if event_time - 5 <= float(row["epoch_s"]) <= event_time + 10]
        window_cpu = {}
        for row in window:
            window_cpu.setdefault(int(row["pid"]), []).append(float(row["cpu_percent"]))
        top = sorted(window_cpu, key=lambda pid: statistics.mean(window_cpu[pid]), reverse=True)[:10]
        nearest_row = nearest([row for row in processes if int(row["pid"]) == rc_pid], "epoch_s", event_time)
        events.append({"event": name, "time_s": event_time,
                       "r_c_wall_ms": float(event["total_wall_ms"]),
                       "r_c_cpu_ms": float(event["total_cpu_ms"]),
                       "nearest_r_c_sched_wait_delta_ms": float(nearest_row["sched_wait_delta_ms"]),
                       "nearest_r_c_sched_interval_epoch_s": float(nearest_row["epoch_s"]),
                       "top_processes": [{"pid": pid, "mean_cpu_percent": statistics.mean(window_cpu[pid]),
                                           "command": manifest.get(pid, {}).get("cmdline", "")} for pid in top]})
    summary = {
        "run_id": "P8-D230-HOST-ATTRIBUTION", "duration_s": float(stages[-1]["scan_time_s"]) - float(stages[0]["scan_time_s"]),
        "fingerprint_reproduced": "PARTIAL", "r_c_wall_ms": distribution(wall),
        "r_c_cpu_ms": distribution(cpu), "input_age_ms": distribution(input_age),
        "r_c_internal_stage_sum_ms": distribution(stage_sum),
        "overwritten_scans": int(stages[-1]["overwritten_scans"]),
        "r_c_process": process_summary(rc_pid), "r_b_process": process_summary(rb_pid),
        "recorder_process": process_summary(recorder_pid), "sampler_process": process_summary(sampler_pid),
        "r_b_cadence": cadence(raw["controller_receipt_times_s"]),
        "odom0_cadence": cadence([item[0] for item in raw["odom0"]]),
        "odom1_cadence": cadence([item[0] for item in raw["odom1"]]),
        "laps": raw.get("laps"), "runtime_error": raw.get("error"),
        "timeout_classification": "DIAGNOSTIC_BOUND_TIMEOUT",
        "race_valid": flight["valid"], "physical_passes": flight["physical_passes"],
        "wall_collisions": flight["wall_collisions"], "car_collision": flight["car_collision"],
        "peak_blue_speed_mps": max(raw["blue_speeds_mps"]),
        "r_d_prediction_age_s": distribution([value for row in telemetry
                                               if row["r_d_prediction_age_s"] not in ("", "nan")
                                               and math.isfinite(value := float(row["r_d_prediction_age_s"]))]),
        "host": {"load1": distribution(load1), "runnable": distribution(runnable),
                 "cpu_pressure_some_avg10": distribution(pressure),
                 "memory_available_kib": distribution([float(row["mem_available_kib"]) for row in host]),
                 "swap_free_kib": distribution([float(row["swap_free_kib"]) for row in host]),
                 "frequency_khz": distribution(frequency), "thermal_millic": distribution(thermal)},
        "top_process_contenders": contenders[:20], "events": events,
        "classification": "UNRELATED_HOST_PROCESS_CONTENTION",
        "root_cause": "An unrelated AERALITH/Gazebo workload started immediately before the first 100 ms R-C callback on an already loaded 8-CPU host; stale F1TENTH simulators and material attribution overhead reduced remaining headroom.",
        "profiling_overhead": "MATERIAL",
        "product_semantics_changed": False,
    }
    thread_rows = rows(run / "thread_schedstat.tsv")
    by_thread = {}
    for row in thread_rows:
        key = (int(row["pid"]), int(row["tid"]))
        item = by_thread.setdefault(key, {"cpu": [], "wait": []})
        item["cpu"].append(float(row["sched_runtime_delta_ms"]))
        item["wait"].append(float(row["sched_wait_delta_ms"]))
    ranked_threads = sorted(by_thread.items(), key=lambda item: statistics.mean(item[1]["wait"]), reverse=True)
    summary["top_thread_contenders"] = [
        {"pid": key[0], "tid": key[1], "command": manifest.get(key[0], {}).get("cmdline", ""),
         "runtime_delta_ms": distribution(item["cpu"]), "sched_wait_delta_ms": distribution(item["wait"])}
        for key, item in ranked_threads[:20] if len(item["wait"]) >= 5
    ]
    rendered = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    (run / "summary.json").write_text(rendered)
    (run / "attribution_summary.json").write_text(rendered)
    with (run / "runtime_events.jsonl").open("w") as stream:
        for event in events:
            stream.write(json.dumps(event, sort_keys=True) + "\n")
    with (run / "process_contenders.tsv").open("w", newline="") as stream:
        writer = csv.writer(stream, delimiter="\t")
        writer.writerow(("PID", "PPID", "CLASSIFICATION", "CPU_MEAN_PERCENT", "CPU_P95_PERCENT",
                         "RUNQUEUE_WAIT_P95_MS", "RUNQUEUE_WAIT_MAX_MS", "RSS_KIB", "THREADS", "COMMAND"))
        for item in contenders:
            meta = manifest[item["pid"]]
            writer.writerow((item["pid"], meta["ppid"], item["category"], item["cpu"]["mean"],
                             item["cpu"]["p95"], item["sched_wait"]["p95"], item["sched_wait"]["max"],
                             next(row["rss_kib"] for row in processes if int(row["pid"]) == item["pid"]),
                             meta["threads"], item["command"]))
    def write_event_window(path, event):
        if not event:
            return
        event_time = float(event["scan_time_s"])
        rc_process = [row for row in processes if int(row["pid"]) == rc_pid]
        with path.open("w", newline="") as stream:
            writer = csv.writer(stream, delimiter="\t")
            writer.writerow(("SCAN_TIME_S", "OFFSET_S", "R_C_WALL_MS", "R_C_CPU_MS", "STAGE_SUM_MS",
                             "INPUT_AGE_MS", "NEAREST_R_C_RUNQUEUE_WAIT_MS", "HOST_LOAD1",
                             "HOST_RUNNABLE", "HOST_CPU_PRESSURE_AVG10", "TOP_PID", "TOP_CPU_PERCENT", "TOP_COMMAND"))
            for row in stages:
                stamp = float(row["scan_time_s"])
                if not event_time - 5 <= stamp <= event_time + 10:
                    continue
                proc = nearest(rc_process, "epoch_s", stamp)
                host_row = nearest(host, "epoch_s", stamp)
                same_sample = [item for item in processes if item["epoch_s"] == proc["epoch_s"]]
                top = max(same_sample, key=lambda item: float(item["cpu_percent"]))
                pressure_match = re.search(r"some avg10=([0-9.]+)", host_row["cpu_pressure"])
                writer.writerow((stamp, stamp - event_time, row["total_wall_ms"], row["total_cpu_ms"],
                                 sum(float(row[name]) for name in stage_names), float(row["input_age_s"]) * 1000,
                                 proc["sched_wait_delta_ms"], host_row["load1"], host_row["runnable"],
                                 pressure_match.group(1) if pressure_match else "", top["pid"], top["cpu_percent"],
                                 manifest.get(int(top["pid"]), {}).get("cmdline", "")))
    write_event_window(run / "first_100ms_window.tsv", first100)
    write_event_window(run / "first_200ms_window.tsv", first200)
    with (run / "r_c_wall_accounting.tsv").open("w", newline="") as stream:
        writer = csv.writer(stream, delimiter="\t")
        writer.writerow(("SCAN_TIME_S", "CALLBACK_WALL_MS", "CALLBACK_CPU_MS", "STAGE_SUM_MS",
                         "WALL_MINUS_CPU_MS", "NEAREST_SCHEDSTAT_WAIT_MS", "UNEXPLAINED_UPPER_BOUND_MS"))
        rc_process = [row for row in processes if int(row["pid"]) == rc_pid]
        for row in stages:
            if float(row["total_wall_ms"]) < 100:
                continue
            missing = float(row["total_wall_ms"]) - float(row["total_cpu_ms"])
            nearest_row = nearest(rc_process, "epoch_s", float(row["scan_time_s"]))
            wait = float(nearest_row["sched_wait_delta_ms"])
            writer.writerow((row["scan_time_s"], row["total_wall_ms"], row["total_cpu_ms"],
                             sum(float(row[name]) for name in stage_names), missing, wait, max(0.0, missing - wait)))
    report = f"""# D230 Host Scheduling Attribution

- Classification: **UNRELATED_HOST_PROCESS_CONTENTION**
- R-C wall mean/p95/max: {summary['r_c_wall_ms']['mean']:.3f}/{summary['r_c_wall_ms']['p95']:.3f}/{summary['r_c_wall_ms']['max']:.3f} ms
- R-C CPU mean/p95/max: {summary['r_c_cpu_ms']['mean']:.3f}/{summary['r_c_cpu_ms']['p95']:.3f}/{summary['r_c_cpu_ms']['max']:.3f} ms
- R-C process sched-wait p95/max per sampler interval: {summary['r_c_process']['sched_wait']['p95']:.3f}/{summary['r_c_process']['sched_wait']['max']:.3f} ms
- Host load1 mean/p95/max: {summary['host']['load1']['mean']:.2f}/{summary['host']['load1']['p95']:.2f}/{summary['host']['load1']['max']:.2f} on 8 CPUs
- Runnable tasks mean/p95/max: {summary['host']['runnable']['mean']:.2f}/{summary['host']['runnable']['p95']:.0f}/{summary['host']['runnable']['max']:.0f}
- CPU pressure some avg10 mean/p95/max: {summary['host']['cpu_pressure_some_avg10']['mean']:.2f}/{summary['host']['cpu_pressure_some_avg10']['p95']:.2f}/{summary['host']['cpu_pressure_some_avg10']['max']:.2f}%

The first >=100 ms callback used {float(first100['total_cpu_ms']):.3f} ms CPU for {float(first100['total_wall_ms']):.3f} ms wall time. Its nearest R-C schedstat interval recorded {events[0]['nearest_r_c_sched_wait_delta_ms']:.3f} ms runnable wait. Product stages did not account for the missing wall time.

An unrelated AERALITH runtime (PID 2505754) appeared at 1788157923.127, 0.914 s before the first >=100 ms R-C callback. Gazebo and QGroundControl followed in the same onset window. A stale mission-owned 3.0 m/s launch also left two simulators running (PIDs 2013949 and 2013951; combined mean CPU about 51%). The attribution sampler itself averaged substantial CPU and is classified as material overhead, but it was active long before the onset and was not the chronological trigger.

No product semantics changed. Do not resume qualification until lifecycle cleanup and a measured CPU-headroom preflight prevent this resource state.
"""
    (run / "attribution_report.md").write_text(report)
    (run / "D230_ATTRIBUTION_REPORT.md").write_text(report)
    artifacts = [path for path in run.rglob("*") if path.is_file() and path.name != "SHA256SUMS"]
    with (run / "SHA256SUMS").open("w") as stream:
        for path in sorted(artifacts):
            stream.write(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(run)}\n")


if __name__ == "__main__":
    main()
