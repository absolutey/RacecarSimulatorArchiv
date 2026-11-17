#!/usr/bin/env python3
"""Bounded, read-only /proc sampler for H2H scheduling attribution."""

import argparse
import csv
import os
from pathlib import Path
import signal
import time


HZ = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
PAGE_KIB = os.sysconf("SC_PAGE_SIZE") // 1024
MISSION_TERMS = ("f1tenth", "ros2", "rviz", "rosbag", "qualify_h2h", "capture_host_attribution")
running = True


def stop(*_):
    global running
    running = False


def counter_delta(current, previous):
    """Treat reset/reuse as a new counter, never as negative scheduler work."""
    return max(0, current - previous)


def nanoseconds_to_milliseconds(value):
    return value / 1e6


def read(path, default=""):
    try:
        return Path(path).read_text(errors="replace")
    except OSError:
        return default


def stat_fields(path):
    text = read(path)
    end = text.rfind(")")
    return text[:end + 1], text[end + 2:].split() if end >= 0 else ("", [])


def proc_sample(pid):
    name, fields = stat_fields(f"/proc/{pid}/stat")
    if len(fields) < 22:
        return None
    sched = read(f"/proc/{pid}/schedstat").split()
    status = read(f"/proc/{pid}/status")
    values = {}
    for line in status.splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            parts = value.strip().split()
            if parts:
                values[key] = parts[0]
    cmdline = read(f"/proc/{pid}/cmdline").replace("\0", " ").strip()
    return {
        "pid": pid, "ppid": int(fields[1]), "comm": name, "state": fields[0],
        "cpu_ticks": int(fields[11]) + int(fields[12]), "processor": int(fields[36]),
        "rss_kib": int(fields[21]) * PAGE_KIB, "threads": int(fields[17]),
        "voluntary": int(values.get("voluntary_ctxt_switches", 0)),
        "involuntary": int(values.get("nonvoluntary_ctxt_switches", 0)),
        "sched_runtime_ns": int(sched[0]) if len(sched) >= 3 else 0,
        "sched_wait_ns": int(sched[1]) if len(sched) >= 3 else 0,
        "sched_slices": int(sched[2]) if len(sched) >= 3 else 0,
        "cmdline": cmdline,
    }


def thread_sample(pid, tid):
    sched = read(f"/proc/{pid}/task/{tid}/schedstat").split()
    _, fields = stat_fields(f"/proc/{pid}/task/{tid}/stat")
    if len(sched) < 3 or len(fields) < 37:
        return None
    return tid, int(fields[11]) + int(fields[12]), int(fields[36]), *map(int, sched[:3])


def cpu_frequency_khz():
    values = []
    for path in Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq"):
        value = read(path).strip()
        if value.isdigit():
            values.append(int(value))
    return sum(values) / len(values) if values else ""


def thermal_millic():
    values = [int(value) for path in Path("/sys/class/thermal").glob("thermal_zone*/temp")
              if (value := read(path).strip()).lstrip("-").isdigit()]
    return max(values) if values else ""


def category(command):
    lower = command.lower()
    if "capture_host_attribution" in lower or "flight_recorder" in lower or "rosbag" in lower:
        return "OBSERVABILITY_RUNTIME"
    if "rviz" in lower:
        return "GUI"
    if any(term in lower for term in ("f1tenth", "qualify_h2h", "ros2 launch")):
        return "PRODUCT_RUNTIME"
    if any(term in lower for term in ("colcon", "pytest", "build")):
        return "BUILD_TEST_AUDIT"
    return "UNRELATED_USER_PROCESS"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--duration", type=float, default=140.0)
    parser.add_argument("--interval", type=float, default=0.2)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    process_path = args.output / "process_samples.tsv"
    thread_path = args.output / "thread_schedstat.tsv"
    host_path = args.output / "host_samples.tsv"
    manifest = {}
    previous = {}
    previous_threads = {}
    started = time.monotonic()
    with process_path.open("w", newline="") as ps, thread_path.open("w", newline="") as ts, host_path.open("w", newline="") as hs:
        pw, tw, hw = csv.writer(ps, delimiter="\t"), csv.writer(ts, delimiter="\t"), csv.writer(hs, delimiter="\t")
        pw.writerow(("monotonic_s", "epoch_s", "pid", "category", "cpu_percent", "cpu_delta_ms", "sched_wait_delta_ms", "sched_slices_delta", "rss_kib", "threads", "processor", "voluntary_delta", "involuntary_delta", "comm"))
        tw.writerow(("monotonic_s", "pid", "tid", "cpu_delta_ms", "sched_runtime_delta_ms", "sched_wait_delta_ms", "sched_slices_delta", "processor"))
        hw.writerow(("monotonic_s", "epoch_s", "load1", "load5", "load15", "runnable", "tasks", "cpu_pressure", "memory_pressure", "mem_available_kib", "swap_free_kib", "frequency_khz", "thermal_millic"))
        while running and time.monotonic() - started < args.duration:
            tick = time.monotonic()
            epoch = time.time()
            load = read("/proc/loadavg").split()
            runnable, tasks = (load[3].split("/") if len(load) > 3 else ("", ""))
            mem = {line.split(":", 1)[0]: line.split()[1] for line in read("/proc/meminfo").splitlines() if ":" in line}
            hw.writerow((tick, epoch, *(load[:3] or ("", "", "")), runnable, tasks,
                         read("/proc/pressure/cpu").strip().replace("\n", ";"),
                         read("/proc/pressure/memory").strip().replace("\n", ";"),
                         mem.get("MemAvailable", ""), mem.get("SwapFree", ""),
                         cpu_frequency_khz(), thermal_millic()))
            for entry in Path("/proc").iterdir():
                if not entry.name.isdigit() or not (sample := proc_sample(int(entry.name))):
                    continue
                pid = sample["pid"]
                manifest.setdefault(pid, sample.copy())
                old = previous.get(pid)
                if old:
                    interval = max(tick - old["time"], 1e-9)
                    cpu_delta = counter_delta(sample["cpu_ticks"], old["cpu_ticks"]) / HZ
                    pw.writerow((tick, epoch, pid, category(sample["cmdline"]), 100 * cpu_delta / interval,
                                 1000 * cpu_delta, nanoseconds_to_milliseconds(counter_delta(sample["sched_wait_ns"], old["sched_wait_ns"])),
                                 counter_delta(sample["sched_slices"], old["sched_slices"]), sample["rss_kib"],
                                 sample["threads"], sample["processor"], counter_delta(sample["voluntary"], old["voluntary"]),
                                 counter_delta(sample["involuntary"], old["involuntary"]), sample["comm"]))
                previous[pid] = {**sample, "time": tick}
                if any(term in sample["cmdline"].lower() for term in MISSION_TERMS):
                    for task in Path(f"/proc/{pid}/task").glob("[0-9]*"):
                        current = thread_sample(pid, int(task.name))
                        if not current:
                            continue
                        key, old_thread = (pid, current[0]), previous_threads.get((pid, current[0]))
                        if old_thread:
                            tw.writerow((tick, pid, current[0], counter_delta(current[1], old_thread[1]) / HZ * 1000,
                                         nanoseconds_to_milliseconds(counter_delta(current[3], old_thread[3])),
                                         nanoseconds_to_milliseconds(counter_delta(current[4], old_thread[4])),
                                         counter_delta(current[5], old_thread[5]), current[2]))
                        previous_threads[key] = current
            ps.flush(); ts.flush(); hs.flush()
            time.sleep(max(0.0, args.interval - (time.monotonic() - tick)))
    with (args.output / "process_manifest.tsv").open("w", newline="") as stream:
        writer = csv.writer(stream, delimiter="\t")
        writer.writerow(("pid", "ppid", "category", "comm", "threads", "cmdline"))
        for pid, item in sorted(manifest.items()):
            writer.writerow((pid, item["ppid"], category(item["cmdline"]), item["comm"], item["threads"], item["cmdline"]))


if __name__ == "__main__":
    main()
