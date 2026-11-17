#!/usr/bin/env python3
"""Fail-closed lifecycle and lightweight host-resource gate for H2H qualification."""

import argparse
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


MISSION_ID = "F1TENTH-H2H-MAX-PERFORMANCE-OBSERVABILITY-20260831"
OWNER_ROOT = Path("/tmp/f1tenth_racing_qualification_owners")
SAMPLE_DURATION_S = 3.0
SAMPLE_INTERVAL_S = 0.25
MAX_RUNNABLE_PER_CPU = 1.0
MAX_CPU_PSI_SOME_AVG10 = 15.0


@dataclass(frozen=True)
class ProcessIdentity:
    pid: int
    ppid: int
    pgid: int
    start_ticks: int
    command: str


@dataclass(frozen=True)
class ResourceSample:
    cpu_count: int
    idle_fraction: float
    runnable: int
    load1: float
    cpu_psi_some_avg10: float
    mem_available_kib: int
    swap_free_kib: int


def process_identity(pid):
    try:
        text = Path(f"/proc/{pid}/stat").read_text()
        end = text.rfind(")")
        fields = text[end + 2:].split()
        if fields[0] == "Z":
            return None
        return ProcessIdentity(
            pid, int(fields[1]), os.getpgid(pid), int(fields[19]),
            Path(f"/proc/{pid}/cmdline").read_text().replace("\0", " ").strip(),
        )
    except (OSError, ValueError, IndexError):
        return None


def same_process(left, right):
    return (left is not None and right is not None and left.pid == right.pid and
            left.pgid == right.pgid and left.start_ticks == right.start_ticks and
            left.command == right.command)


def is_owned_launch_command(command):
    return ("ros2 launch f1tenth_racing_bringup race_foundation.launch.py" in command or
            "ros2 run f1tenth_racing_tools f1sim" in command)


def process_group_members(pgid):
    members = []
    for entry in Path("/proc").iterdir():
        if entry.name.isdigit():
            identity = process_identity(int(entry.name))
            if identity is not None and identity.pgid == pgid:
                members.append(identity)
    return members


def register_runtime_owner(pid, repository, domain, run_id, owner_root=OWNER_ROOT,
                           controller_pid=None):
    identity = process_identity(pid)
    controller = process_identity(os.getpid() if controller_pid is None else controller_pid)
    if identity is None or identity.pid != identity.pgid:
        raise RuntimeError("qualification launch must own a new process group")
    if controller is None:
        raise RuntimeError("qualification controller identity unavailable")
    owner_root.mkdir(parents=True, exist_ok=True)
    record = {**asdict(identity), "controller_pid": controller.pid,
              "controller_start_ticks": controller.start_ticks,
              "repository": str(Path(repository).resolve()),
              "mission_id": MISSION_ID, "ros_domain": int(domain), "run_id": run_id}
    path = owner_root / f"{pid}.json"
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(record, sort_keys=True) + "\n")
    temporary.replace(path)
    return path


def unregister_runtime_owner(path):
    try:
        Path(path).unlink()
    except FileNotFoundError:
        pass


def classify_owner_record(path, repository):
    try:
        record = json.loads(path.read_text())
        identity = process_identity(int(record["pid"]))
        controller = process_identity(int(record["controller_pid"]))
    except (OSError, ValueError, KeyError, json.JSONDecodeError):
        return "AMBIGUOUS", None, "malformed owner manifest"
    if identity is None:
        path.unlink(missing_ok=True)
        return "CLEAR", None, "expired owner manifest"
    if controller is not None and controller.start_ticks == int(record.get("controller_start_ticks", -1)):
        return "CURRENT_MISSION_PROCESS", identity, "qualification controller is active"
    expected = (record.get("mission_id") == MISSION_ID and
                Path(record.get("repository", "")).resolve() == Path(repository).resolve() and
                identity.start_ticks == int(record.get("start_ticks", -1)) and
                identity.pgid == int(record.get("pgid", -1)) == identity.pid and
                is_owned_launch_command(identity.command))
    if not expected:
        return "AMBIGUOUS", identity, "owner manifest does not match live process identity"
    return "STALE_F1TENTH_MISSION_PROCESS", identity, "verified owner manifest"


def inspect_stale_runtime(repository, owner_root=OWNER_ROOT):
    stale, ambiguous = [], []
    if owner_root.exists():
        for path in sorted(owner_root.glob("*.json")):
            classification, identity, reason = classify_owner_record(path, repository)
            if classification == "STALE_F1TENTH_MISSION_PROCESS":
                stale.append((path, identity, reason))
            elif classification in ("AMBIGUOUS", "CURRENT_MISSION_PROCESS"):
                ambiguous.append((path, identity, reason))
    # An unregistered active race launch is detectable but never safe to kill.
    registered = {item[1].pid for item in stale + ambiguous if item[1] is not None}

    # Ignore this preflight process and its launcher ancestry.
    # Otherwise `ros2 run ... f1sim` itself is mistaken for an
    # unregistered pre-existing F1TENTH runtime.
    own_ancestry = set()
    pid = os.getpid()
    while pid > 1 and pid not in own_ancestry:
        own_ancestry.add(pid)
        status = Path(f"/proc/{pid}/status")
        if not status.exists():
            break
        ppid = None
        for line in status.read_text().splitlines():
            if line.startswith("PPid:"):
                ppid = int(line.split()[1])
                break
        if ppid is None or ppid <= 1:
            break
        pid = ppid

    for entry in Path("/proc").iterdir():
        if (not entry.name.isdigit() or int(entry.name) in own_ancestry or
                int(entry.name) in registered):
            continue
        identity = process_identity(int(entry.name))
        if identity and is_owned_launch_command(identity.command):
            ambiguous.append((None, identity, "active F1TENTH launch has no owner manifest"))
    return stale, ambiguous


def cleanup_stale_runtime(stale, killpg=os.killpg, sleep=time.sleep, timeout_s=5.0):
    cleaned = []
    for path, identity, _ in stale:
        if not same_process(process_identity(identity.pid), identity):
            raise RuntimeError(f"BLOCKED_AMBIGUOUS_PROCESS_OWNERSHIP pid={identity.pid} changed")
        killpg(identity.pgid, signal.SIGINT)
        deadline = time.monotonic() + timeout_s
        while process_group_members(identity.pgid) and time.monotonic() < deadline:
            sleep(0.05)
        if process_group_members(identity.pgid):
            killpg(identity.pgid, signal.SIGTERM)
            deadline = time.monotonic() + timeout_s
            while process_group_members(identity.pgid) and time.monotonic() < deadline:
                sleep(0.05)
        remaining = process_group_members(identity.pgid)
        if remaining:
            raise RuntimeError(
                f"BLOCKED_STALE_MISSION_RUNTIME pgid={identity.pgid} retained "
                f"{len(remaining)} processes"
            )
        path.unlink(missing_ok=True)
        cleaned.append(identity.pid)
    return cleaned


def stop_owned_group(pgid, timeout_s=5.0):
    if not process_group_members(pgid):
        return
    os.killpg(pgid, signal.SIGINT)
    deadline = time.monotonic() + timeout_s
    while process_group_members(pgid) and time.monotonic() < deadline:
        time.sleep(0.05)
    if process_group_members(pgid):
        os.killpg(pgid, signal.SIGTERM)
        deadline = time.monotonic() + timeout_s
        while process_group_members(pgid) and time.monotonic() < deadline:
            time.sleep(0.05)
    if process_group_members(pgid):
        raise RuntimeError(f"owned runtime process group {pgid} did not terminate")


def _cpu_totals():
    fields = Path("/proc/stat").read_text().splitlines()[0].split()[1:]
    values = [int(value) for value in fields]
    return sum(values), values[3] + (values[4] if len(values) > 4 else 0)


def _psi_avg10():
    for item in Path("/proc/pressure/cpu").read_text().splitlines()[0].split():
        if item.startswith("avg10="):
            return float(item.split("=", 1)[1])
    return 0.0


def sample_resources(duration_s=SAMPLE_DURATION_S, interval_s=SAMPLE_INTERVAL_S,
                     monotonic=time.monotonic, sleep=time.sleep):
    cpu_count = os.cpu_count() or 1
    deadline = monotonic() + duration_s
    previous_total, previous_idle = _cpu_totals()
    samples = []
    while True:
        sleep(min(interval_s, max(0.0, deadline - monotonic())))
        total, idle = _cpu_totals()
        delta_total, delta_idle = max(1, total - previous_total), max(0, idle - previous_idle)
        previous_total, previous_idle = total, idle
        load = Path("/proc/loadavg").read_text().split()
        runnable = int(load[3].split("/")[0])
        memory = {line.split(":", 1)[0]: int(line.split()[1])
                  for line in Path("/proc/meminfo").read_text().splitlines() if ":" in line}
        samples.append(ResourceSample(cpu_count, delta_idle / delta_total, runnable,
                                      float(load[0]), _psi_avg10(), memory["MemAvailable"],
                                      memory.get("SwapFree", 0)))
        if monotonic() >= deadline:
            return samples


def evaluate_resources(samples):
    at95 = lambda values: sorted(values)[int(.95 * (len(values) - 1))]
    cpu_count = samples[0].cpu_count
    runnable = [sample.runnable for sample in samples]
    psi = [sample.cpu_psi_some_avg10 for sample in samples]
    p95_runnable, p95_psi = at95(runnable), at95(psi)
    reasons = []
    if p95_runnable / cpu_count > MAX_RUNNABLE_PER_CPU:
        reasons.append(f"runnable_p95_per_cpu={p95_runnable / cpu_count:.3f}>{MAX_RUNNABLE_PER_CPU:.3f}")
    if p95_psi > MAX_CPU_PSI_SOME_AVG10:
        reasons.append(f"cpu_psi_some_avg10_p95={p95_psi:.3f}>{MAX_CPU_PSI_SOME_AVG10:.3f}")
    return {
        "status": "BLOCKED_INSUFFICIENT_HEADROOM" if reasons else "PASS",
        "reasons": reasons, "sample_count": len(samples),
        "duration_s": SAMPLE_DURATION_S,
        "cpu_count": cpu_count, "idle_fraction_min": min(sample.idle_fraction for sample in samples),
        "runnable_p95": p95_runnable, "runnable_max": max(runnable),
        "load1_max": max(sample.load1 for sample in samples),
        "cpu_psi_some_avg10_p95": p95_psi, "cpu_psi_some_avg10_max": max(psi),
        "mem_available_kib_min": min(sample.mem_available_kib for sample in samples),
        "swap_free_kib_min": min(sample.swap_free_kib for sample in samples),
        "clean_host_calibration_required": ["idle_fraction"],
    }


def require_qualification_preflight(repository, sample_function=sample_resources):
    stale, ambiguous = inspect_stale_runtime(repository)
    if ambiguous:
        raise RuntimeError("BLOCKED_AMBIGUOUS_PROCESS_OWNERSHIP " +
                           ",".join(str(item[1].pid) for item in ambiguous if item[1]))
    cleaned = cleanup_stale_runtime(stale)
    result = evaluate_resources(sample_function())
    result["stale_mission_runtime"] = "CLEANED" if cleaned else "CLEAR"
    result["cleaned_pids"] = cleaned
    result["unrelated_processes_terminated"] = 0
    if result["status"] != "PASS":
        raise RuntimeError("RESOURCE_PREFLIGHT=" + result["status"] + " " + ";".join(result["reasons"]))
    return result


def run_guarded(command, repository, domain, run_id,
                preflight=require_qualification_preflight, popen=subprocess.Popen):
    preflight(repository)
    process = popen(command, start_new_session=True)
    owner_path = None
    try:
        owner_path = register_runtime_owner(process.pid, repository, domain, run_id)
        return process.wait()
    except KeyboardInterrupt:
        stop_owned_group(process.pid)
        return 130
    finally:
        stop_owned_group(process.pid)
        if owner_path is not None and not process_group_members(process.pid):
            unregister_runtime_owner(owner_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0")))
    parser.add_argument("--run-id", default="F1SIM-MAX-PERFORMANCE")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a runtime command is required")
    location = Path(__file__).resolve()
    repository = location.parents[3] if (location.parents[3] / ".git").exists() else location.parents[2]
    try:
        return run_guarded(command, repository, args.domain, args.run_id)
    except RuntimeError as error:
        print("F1TENTH Max-Performance runtime not started.")
        print(str(error))
        print("No unrelated processes were terminated.")
        return 2


if __name__ == "__main__":
    sys.exit(main())
