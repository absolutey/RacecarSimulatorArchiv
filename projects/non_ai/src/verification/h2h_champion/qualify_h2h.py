#!/usr/bin/env python3
import argparse
import ast
import hashlib
import importlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import tempfile
import time

import rclpy
import yaml
import control_msgs.msg as control_messages
import f1tenth_racing_control.core as control_core
import f1tenth_racing_control.node as control_node
import f1tenth_racing_overtaking.core as overtaking_core
import f1tenth_racing_overtaking.node as overtaking_node
import f1tenth_racing_perception.core as perception_core
import f1tenth_racing_perception.node as perception_node
import f1tenth_racing_prediction.core as prediction_core
import f1tenth_racing_prediction.node as prediction_node
import f1tenth_racing_reference.core as reference_core
import f1tenth_racing_reference.node as reference_node
import f1tenth_racing_sim.simulator as simulator_node
import f1tenth_racing_sim.physics.vehicle as vehicle_dynamics
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from control_msgs.msg import ActiveTargetTrajectory, ControllerStatus, OpponentPrediction, OpponentState, OvertakingPlan
from f1tenth_racing_geometry import ClosedTrackGeometry, load_centerline_csv
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosidl_adapter.parser import parse_message_file
from std_msgs.msg import Bool, Float64, Int32, String
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'qualification_gates'))
from applied_lineage import AppliedLineageAudit
from std_srvs.srv import Trigger

from resource_preflight import (
    register_runtime_owner,
    require_qualification_preflight,
    unregister_runtime_owner,
)


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def distribution(values):
    if not values:
        return None
    ordered = sorted(values)
    at = lambda q: ordered[min(len(ordered) - 1, round(q * (len(ordered) - 1)))]
    return {"mean": statistics.fmean(values), "p95": at(0.95), "p99": at(0.99), "max": max(values)}


def yaw(message):
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def footprint(x, y, angle, half_length, half_width):
    c, s = math.cos(angle), math.sin(angle)
    return tuple((x + c * lx - s * ly, y + s * lx + c * ly)
                 for lx, ly in ((-half_length, -half_width), (half_length, -half_width),
                                (half_length, half_width), (-half_length, half_width)))


def footprint_clearance(first, second):
    def point_segment(point, start, end):
        dx, dy = end[0] - start[0], end[1] - start[1]
        scale = max(dx * dx + dy * dy, 1e-30)
        ratio = max(0.0, min(1.0, ((point[0] - start[0]) * dx +
                                    (point[1] - start[1]) * dy) / scale))
        return math.hypot(point[0] - start[0] - ratio * dx,
                          point[1] - start[1] - ratio * dy)

    def overlaps():
        for shape in (first, second):
            for index in range(2):
                start, end = shape[index], shape[index + 1]
                axis = (start[1] - end[1], end[0] - start[0])
                projections = [[x * axis[0] + y * axis[1] for x, y in item]
                               for item in (first, second)]
                if max(projections[0]) < min(projections[1]) or \
                        max(projections[1]) < min(projections[0]):
                    return False
        return True

    if overlaps():
        return 0.0
    return min(point_segment(point, shape[index], shape[(index + 1) % len(shape)])
               for point_set, shape in ((first, second), (second, first))
               for point in point_set for index in range(len(shape)))


def qualification_failures(result, requested_laps, requested_cap):
    states = set(result["plan_states"])
    interaction_cap = result.get("interaction_cap_mps", requested_cap)
    peak = result["peak_blue_speed_mps"]
    speed_low = (peak is None or (
        peak < requested_cap - 0.05 if interaction_cap >= requested_cap else
        peak <= interaction_cap + 0.05
    ))
    checks = {
        "launch_exit": result["launch_exit_code"] != 0,
        "residue": bool(result["residual_processes"]),
        "laps": result["laps"] is None or result["laps"] < requested_laps,
        "race_valid": result["race_valid"] is not True,
        "blue_wall": result["walls0"] != 0,
        "red_wall": result["walls1"] != 0,
        "vehicle_collision": bool(result["vehicle_collision"]),
        "override": not result["override_observed"],
        "late_join": not result["late_join_override_observed"],
        "physical_pass": result["truth_pass_count"] < 1,
        "nominal_return": not result["nominal_return_observed"],
        "lifecycle": (not {"COMMITTED_PASS", "PASSING", "COMPLETE"}.issubset(states) or
                      not {"MERGE", "ABORT"}.intersection(states)),
        "speed_low": speed_low,
        "speed_high": peak is None or peak > requested_cap + 1e-9,
    }
    return sorted(name for name, failed in checks.items() if failed)


def runtime_complete(laps, requested_laps, events):
    stages, complete_revision = {}, None
    for event in events:
        if event[0] == "PLAN":
            _, revision, state = event
            if state == "COMMITTED_PASS" and revision != complete_revision:
                complete_revision = None
            stage = stages.get(revision, 0)
            if stage == 0 and state == "COMMITTED_PASS":
                stages[revision] = 1
            elif stage == 1 and state == "PASSING":
                stages[revision] = 2
            elif stage in (1, 2) and state in ("MERGE", "ABORT"):
                stages[revision] = 3
            elif stage == 3 and state == "COMPLETE":
                stages[revision] = 4
                complete_revision = revision
        elif complete_revision is not None and event == ("REFERENCE", "NOMINAL"):
            return laps is not None and laps >= requested_laps
    return False


def update_pass_state(geometry, was_behind, blue, red):
    delta = geometry.signed_delta_s(
        geometry.project(red[1], red[2]).s,
        geometry.project(blue[1], blue[2]).s,
    )
    if was_behind and delta > 1.0:
        return False, True
    if delta < -1.0:
        return True, False
    return was_behind, False


def startup_wait_failure(process_returncode, now_s, deadline_s):
    if process_returncode is not None:
        return f"launch exited before start: {process_returncode}"
    if now_s >= deadline_s:
        return "race start service timed out"
    return None


def require_matching_install(repository, owner, source_relative, installed_module):
    source = repository / source_relative
    installed = Path(installed_module.__file__)
    if source.read_bytes() != installed.read_bytes():
        raise RuntimeError(f"stale installed {owner}: {installed} differs from {source}")
    return installed


def require_matching_installed_file(repository, owner, source_relative, installed):
    source = repository / source_relative
    if source.read_bytes() != Path(installed).read_bytes():
        raise RuntimeError(f"stale installed {owner}: {installed} differs from {source}")
    return Path(installed)


PYTHON_RUNTIME_TREES = (
    ("R-B", "src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control", "f1tenth_racing_control"),
    ("R-C", "src/driving/algorithms/f1tenth_racing_perception/f1tenth_racing_perception", "f1tenth_racing_perception"),
    ("R-D", "src/driving/algorithms/f1tenth_racing_prediction/f1tenth_racing_prediction", "f1tenth_racing_prediction"),
    ("R-E", "src/driving/algorithms/f1tenth_racing_overtaking/f1tenth_racing_overtaking", "f1tenth_racing_overtaking"),
    ("R-A", "src/driving/algorithms/f1tenth_racing_raceline/f1tenth_racing_raceline", "f1tenth_racing_raceline"),
    ("Red", "src/driving/f1tenth_racing_opponent/f1tenth_racing_opponent", "f1tenth_racing_opponent"),
    ("Foundation Blue", "src/driving/foundation_baseline/foundation_baseline", "foundation_baseline"),
    ("Foundation", "src/environment/f1tenth_racing_sim/f1tenth_racing_sim", "f1tenth_racing_sim"),
    ("Evaluator", "src/platform/f1tenth_racing_evaluator/f1tenth_racing_evaluator", "f1tenth_racing_evaluator"),
    ("S0-G", "src/platform/f1tenth_racing_geometry/f1tenth_racing_geometry", "f1tenth_racing_geometry"),
    ("S0-R", "src/platform/f1tenth_racing_reference/f1tenth_racing_reference", "f1tenth_racing_reference"),
)

SHARE_RUNTIME_TREES = (
    ("Bringup", "src/platform/f1tenth_racing_bringup/launch", "f1tenth_racing_bringup", "launch"),
    ("Bringup", "src/platform/f1tenth_racing_bringup/config", "f1tenth_racing_bringup", "config"),
    ("Bringup", "src/platform/f1tenth_racing_bringup/rviz", "f1tenth_racing_bringup", "rviz"),
    ("R-B", "src/driving/algorithms/f1tenth_racing_control/config", "f1tenth_racing_control", "config"),
    ("R-C", "src/driving/algorithms/f1tenth_racing_perception/config", "f1tenth_racing_perception", "config"),
    ("R-D", "src/driving/algorithms/f1tenth_racing_prediction/config", "f1tenth_racing_prediction", "config"),
    ("R-E", "src/driving/algorithms/f1tenth_racing_overtaking/config", "f1tenth_racing_overtaking", "config"),
    ("R-A", "src/driving/algorithms/f1tenth_racing_raceline/data", "f1tenth_racing_raceline", "data"),
    ("S0-R", "src/platform/f1tenth_racing_reference/config", "f1tenth_racing_reference", "config"),
    ("Foundation", "src/environment/f1tenth_racing_sim/config", "f1tenth_racing_sim", "config"),
    ("Foundation", "src/environment/f1tenth_racing_sim/maps/iccas2025", "f1tenth_racing_sim", "maps/iccas2025"),
    ("Foundation", "src/environment/f1tenth_racing_description/urdf", "f1tenth_racing_description", "urdf"),
    ("Interfaces", "src/interfaces/control_msgs/msg", "control_msgs", "msg"),
)


def _runtime_files(source_root):
    return sorted(path for path in source_root.rglob("*")
                  if path.is_file() and "__pycache__" not in path.parts)


def runtime_source_inventory(repository):
    inventory = []
    for owner, source_relative, package in PYTHON_RUNTIME_TREES:
        source_root = repository / source_relative
        inventory.extend((owner, source, package, "python", source.relative_to(source_root))
                         for source in _runtime_files(source_root))
    for owner, source_relative, package, installed_relative in SHARE_RUNTIME_TREES:
        source_root = repository / source_relative
        inventory.extend((owner, source, package, "share",
                          Path(installed_relative) / source.relative_to(source_root))
                         for source in _runtime_files(source_root))
    return inventory


def source_console_scripts(setup_path):
    tree = ast.parse(setup_path.read_text())
    return {node.value.strip() for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
            and " = " in node.value and ":" in node.value}


def generated_control_message_files(repository):
    message_root = repository / "src/interfaces/control_msgs/msg"
    for source in sorted(message_root.glob("*.msg")):
        expected = {}
        for field in parse_message_file("control_msgs", source).fields:
            field_type = str(field.type).replace("float64", "double").replace("bool", "boolean")
            if field_type.endswith("[]"):
                field_type = f"sequence<{field_type[:-2]}>"
            expected[field.name] = field_type
        observed = getattr(control_messages, source.stem).get_fields_and_field_types()
        if observed != expected:
            raise RuntimeError(f"stale generated control_msgs binding: {source.name}")
    prefix = Path(get_package_share_directory("control_msgs")).parents[1]
    if not Path(control_messages.__file__).absolute().is_relative_to(prefix.absolute()):
        raise RuntimeError("control_msgs Python binding resolves outside selected prefix")
    return _runtime_files(prefix)


def entrypoint_wrapper_coherent(path, distribution_name, entrypoint_name):
    tree = ast.parse(path.read_text())
    expected = (distribution_name, "console_scripts", entrypoint_name)
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and
                node.func.id == "load_entry_point" and len(node.args) >= 3):
            actual = tuple(arg.value if isinstance(arg, ast.Constant) else None
                           for arg in node.args[:3])
            normalized = ((actual[0].partition("==")[0] if isinstance(actual[0], str)
                           else actual[0]), *actual[1:])
            if normalized == expected:
                return True
    return False


def require_current_product_install(repository):
    matched = []
    packages = {item[2] for item in SHARE_RUNTIME_TREES}
    packages.update(item[2] for item in PYTHON_RUNTIME_TREES)
    packages.update(("f1tenth_racing_bringup", "f1tenth_racing_tools"))
    shares = {package: Path(get_package_share_directory(package)) for package in packages}

    for owner, source, package, area, installed_relative in runtime_source_inventory(repository):
        installed_root = (Path(importlib.import_module(package).__file__).parent
                          if area == "python" else shares[package])
        matched.append(require_matching_installed_file(
            repository, owner, source.relative_to(repository), installed_root / installed_relative,
        ))

    executables = (
        ("Bringup", "src/platform/f1tenth_racing_bringup/scripts/race_coordinator.py", "f1tenth_racing_bringup", "race_coordinator"),
        ("H2H tools", "src/platform/f1tenth_racing_tools/scripts/race_start.py", "f1tenth_racing_tools", "race_start"),
        *(("H2H tools", f"src/platform/f1tenth_racing_tools/scripts/{name}", "f1tenth_racing_tools", name)
          for name in ("f1sim", "f1fast", "f1safe", "f1go", "f1solo")),
        ("H2H tools", "src/verification/h2h_champion/resource_preflight.py",
         "f1tenth_racing_tools", "resource_preflight.py"),
    )
    for owner, source, package, executable in executables:
        prefix = shares[package].parents[1]
        matched.append(require_matching_installed_file(
            repository, owner, source, prefix / "lib" / package / executable,
        ))
    generated = {}
    for _, source_relative, package in PYTHON_RUNTIME_TREES:
        expected = source_console_scripts((repository / source_relative).parent / "setup.py")
        distribution = importlib.metadata.distribution(package.replace("_", "-"))
        entry_points = {f"{item.name} = {item.value}": item for item in distribution.entry_points
                        if item.group == "console_scripts"}
        if set(entry_points) != expected:
            raise RuntimeError(f"stale installed entry points for {package}")
        metadata = distribution.read_text("entry_points.txt")
        if expected and metadata is None:
            raise RuntimeError(f"missing installed entry-point metadata for {package}")
        if metadata is not None:
            generated[f"metadata:{distribution.metadata['Name']}:entry_points.txt"] = \
                hashlib.sha256(metadata.encode()).hexdigest()
        prefix = shares[package].parents[1]
        for declaration, entry_point in entry_points.items():
            executable = prefix / "lib" / package / entry_point.name
            distribution_name = distribution.metadata["Name"]
            if (not executable.is_file() or
                    not entrypoint_wrapper_coherent(
                        executable, distribution_name, entry_point.name) or
                    not callable(entry_point.load())):
                raise RuntimeError(f"invalid installed entry point: {declaration}")
            generated[str(executable)] = hashlib.sha256(executable.read_bytes()).hexdigest()
    for path in generated_control_message_files(repository):
        generated[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {
        "head": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repository, text=True,
        ).strip(),
        "tree": subprocess.check_output(
            ["git", "rev-parse", "HEAD^{tree}"], cwd=repository, text=True,
        ).strip(),
        "package_prefixes": sorted({str(share.parents[1]) for share in shares.values()}),
        "installed_files": {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in matched
        },
        "generated_artifacts": generated,
    }


def main():
    install_identity = require_current_product_install(Path(__file__).resolve().parents[3])
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--blue-cap", type=float, default=3.0)
    parser.add_argument("--interaction-cap", type=float)
    parser.add_argument("--laps", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--user-path", action="store_true")
    parser.add_argument("--evidence-dir", type=Path)
    parser.add_argument("--flight-recorder", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    try:
        resource_preflight = require_qualification_preflight(Path(__file__).resolve().parents[3])
    except RuntimeError as error:
        print("H2H qualification not started.")
        print(str(error))
        print("No unrelated processes were terminated.")
        raise SystemExit(2)
    if args.preflight_only:
        print(json.dumps(resource_preflight, sort_keys=True))
        return
    interaction_cap = args.blue_cap if args.interaction_cap is None else args.interaction_cap
    command = [
        "ros2", "launch", "f1tenth_racing_bringup", "race_foundation.launch.py",
        "use_rviz:=false", "solo:=false", "use_r_b0:=true",
        "execution_profile:=H2H_MPC_DYNAMIC_EXPERIMENTAL",
        "controller_kind:=mpc", "trajectory_candidate:=scp",
        "dynamic_red_guard_enabled:=true",
        "opponent_prediction_horizon_s:=9.0",
        f"speed_cap_mps:={args.blue_cap}",
        f"interaction_speed_cap_mps:={interaction_cap}",
        "use_opponent_perception:=true", "opponent_tracker:=cv",
        "use_opponent_prediction:=true", "opponent_predictor:=track_cv",
        "use_overtaking:=true", "publish_red_lidar:=false",
    ]
    if args.flight_recorder:
        if args.evidence_dir is None:
            parser.error("--flight-recorder requires --evidence-dir")
        command.extend([
            "use_flight_recorder:=true", "record_rosbag:=false",
            f"telemetry_run_root:={args.evidence_dir / 'flight_recorder'}",
            f"telemetry_run_id:=H2H-CAP-{args.blue_cap:.1f}-D{args.domain}",
        ])
    if args.user_path:
        # User-facing H2H runner must not silently launch the SOLO wrapper.
        # Preserve the requested qualification caps rather than using AUTO.
        command = ["ros2", "run", "f1tenth_racing_tools", "f1h2h",
                   "use_rviz:=false", f"speed_cap_mps:={args.blue_cap}",
                   f"interaction_speed_cap_mps:={interaction_cap}"]
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            owner_path = register_runtime_owner(
                process.pid, Path(__file__).resolve().parents[3], args.domain,
                f"H2H-CAP-{args.blue_cap:.1f}-D{args.domain}",
            )
        except Exception:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)
            raise
        rclpy.init()
        node = Node(f"h2h_truth_evaluator_{args.domain}")
        laps, walls0, walls1, valid, vehicle_collision = [None], [None], [None], [None], [False]
        plans, active, controls, states, predictions, odom0, odom1 = [], [], [], [], [], [], []
        lifecycle, source_transitions, completion_events, collision_time = [], [], [], [None]
        lap_times, blue_speeds, start_stamp = [], [], [None]
        late_process, late_output = [None], [""]
        atomic_lineage=AppliedLineageAudit(require_external_watchdog=True)
        node.create_subscription(String,'actuator_applied_receipt0',
                                 lambda m:atomic_lineage.observe(m.data),10)
        node.create_subscription(Int32, "lap_count0", lambda m: laps.__setitem__(0, m.data), 1)
        node.create_subscription(Int32, "wall_collision_count0", lambda m: walls0.__setitem__(0, m.data), 1)
        node.create_subscription(Int32, "wall_collision_count1", lambda m: walls1.__setitem__(0, m.data), 1)
        node.create_subscription(Bool, "race_valid", lambda m: valid.__setitem__(0, m.data), 1)
        def collision_received(message):
            vehicle_collision[0] = vehicle_collision[0] or message.data
            if message.data and collision_time[0] is None:
                collision_time[0] = node.get_clock().now().nanoseconds * 1e-9

        def plan_received(message):
            plans.append(message)
            completion_events.append(("PLAN", message.plan_revision, message.maneuver_state))
            event = (message.maneuver_state, message.status_reason, message.valid)
            if not lifecycle or lifecycle[-1][1:] != event:
                lifecycle.append((node.get_clock().now().nanoseconds * 1e-9, *event))

        def reference_received(message):
            active.append(message)
            completion_events.append(("REFERENCE", message.source_kind))
            event = (message.source_kind, message.status_reason)
            if not source_transitions or source_transitions[-1][1:] != event:
                source_transitions.append((node.get_clock().now().nanoseconds * 1e-9, *event))

        node.create_subscription(Bool, "vehicle_collision", collision_received, 1)
        node.create_subscription(Float64, "lap_time0", lambda m: lap_times.append(m.data), 10)
        node.create_subscription(Float64, "speed0", lambda m: blue_speeds.append(m.data), 10)
        node.create_subscription(Time, "race_start_time", lambda m: start_stamp.__setitem__(0, stamp_s(m)), 1)
        node.create_subscription(OvertakingPlan, "overtaking_plan", plan_received, 10)
        node.create_subscription(ActiveTargetTrajectory, "active_target_trajectory", reference_received, 10)
        node.create_subscription(ControllerStatus, "controller_status0", controls.append, 10)
        node.create_subscription(OpponentState, "opponent_state", states.append, 10)
        node.create_subscription(OpponentPrediction, "opponent_prediction", predictions.append, 10)
        node.create_subscription(Odometry, "odom0", lambda m: odom0.append((stamp_s(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m))), 10)
        node.create_subscription(Odometry, "odom1", lambda m: odom1.append((stamp_s(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y, yaw(m))), 10)
        client = node.create_client(Trigger, "race_start")
        sim_share = Path(get_package_share_directory("f1tenth_racing_sim"))
        environment = yaml.safe_load((sim_share / "config/competition_environment.yaml").read_text())
        half_length = 0.5 * environment["vehicle"]["description"]["chassis_length_m"]
        half_width = 0.5 * environment["vehicle"]["description"]["chassis_width_m"]
        geometry = ClosedTrackGeometry(
            load_centerline_csv(sim_share / "maps/iccas2025/iccas2025_centerline.csv")
        )
        started = time.monotonic()
        try:
            if not client.wait_for_service(timeout_sec=20.0):
                raise RuntimeError("race_start unavailable")
            if args.user_path:
                start_process = subprocess.Popen(
                    ["ros2", "run", "f1tenth_racing_tools", "f1go"],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                )
                start_deadline = time.monotonic() + 40.0
                while start_process.poll() is None:
                    if time.monotonic() >= start_deadline:
                        start_process.terminate()
                        start_output, _ = start_process.communicate(timeout=5.0)
                        raise RuntimeError(f"installed f1go timed out: {start_output[-2000:]}")
                    rclpy.spin_once(node, timeout_sec=0.1)
                start_output, _ = start_process.communicate()
                if start_process.returncode:
                    raise RuntimeError(f"installed f1go failed: {start_output[-2000:]}")
            else:
                future = client.call_async(Trigger.Request())
                start_deadline = time.monotonic() + 40.0
                while not future.done():
                    failure = startup_wait_failure(
                        process.poll(), time.monotonic(), start_deadline,
                    )
                    if failure:
                        raise RuntimeError(failure)
                    rclpy.spin_once(node, timeout_sec=0.1)
            deadline = time.monotonic() + args.timeout
            live_was_behind, live_pass_count = False, 0
            while not (runtime_complete(laps[0], args.laps, completion_events) and
                       live_pass_count > 0):
                if process.poll() is not None:
                    raise RuntimeError(f"launch exited early: {process.returncode}")
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"timeout laps={laps[0]}")
                if walls0[0] not in (None, 0) or vehicle_collision[0]:
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
                if odom0 and odom1 and abs(odom0[-1][0] - odom1[-1][0]) <= 0.04:
                    live_was_behind, passed = update_pass_state(
                        geometry, live_was_behind, odom0[-1], odom1[-1],
                    )
                    live_pass_count += int(passed)
                if late_process[0] is None and active and active[-1].source_kind == "OVERTAKE_OVERRIDE":
                    observer = Path(__file__).with_name("observe_active_reference.py")
                    late_process[0] = subprocess.Popen(
                        ["python3", str(observer)],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                    )
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
            pairs, minimum_separation, minimum_clearance, pass_count, first_pass_time, was_behind = 0, math.inf, math.inf, 0, None, False
            for blue in odom0:
                if not odom1:
                    break
                red = min(odom1, key=lambda item: abs(item[0] - blue[0]))
                if abs(red[0] - blue[0]) > 0.04:
                    continue
                pairs += 1
                minimum_separation = min(minimum_separation, math.hypot(blue[1] - red[1], blue[2] - red[2]))
                minimum_clearance = min(minimum_clearance,
                    footprint_clearance(
                        footprint(blue[1], blue[2], blue[3], half_length, half_width),
                        footprint(red[1], red[2], red[3], half_length, half_width),
                    ))
                was_behind, passed = update_pass_state(geometry, was_behind, blue, red)
                if passed:
                    pass_count += 1
                    if first_pass_time is None and start_stamp[0] is not None:
                        first_pass_time = blue[0] - start_stamp[0]
            control_times = [item.receipt_time_ns * 1e-9 for item in controls]
            override_index = next((index for index, item in enumerate(active)
                                   if item.source_kind == "OVERTAKE_OVERRIDE"), None)
            nominal_return = (override_index is not None and
                              any(item.source_kind == "NOMINAL" for item in active[override_index + 1:]))
            atomic_verdict=atomic_lineage.verdict()
            if not atomic_verdict['passed']:
                raise RuntimeError('H2H_ATOMIC_APPLIED_LINEAGE_NOT_QUALIFIED:'+str(atomic_verdict))
            result = {
                'atomic_applied_lineage':atomic_verdict,
                "experiment_id": f"H2H-CAP-{args.blue_cap:.1f}-D{args.domain}",
                "domain": args.domain, "blue_cap_mps": args.blue_cap,
                "interaction_cap_mps": interaction_cap, "laps": laps[0],
                "race_valid": valid[0], "walls0": walls0[0], "walls1": walls1[0],
                "vehicle_collision": vehicle_collision[0], "truth_pass_count": pass_count,
                "first_pass_time_s": first_pass_time, "lap_times_s": lap_times,
                "flying_lap_times_s": lap_times[1:],
                "race_start_time_s": start_stamp[0],
                "peak_blue_speed_mps": max(blue_speeds) if blue_speeds else None,
                "minimum_center_separation_m": minimum_separation, "truth_aligned_pairs": pairs,
                "minimum_footprint_clearance_m": minimum_clearance,
                "plan_count": len(plans), "valid_plan_count": sum(item.valid for item in plans),
                "plan_states": sorted(set(item.maneuver_state for item in plans)),
                "plan_reasons": sorted(set(item.status_reason for item in plans)),
                "passing_sides": sorted(set(item.passing_side for item in plans if item.valid)),
                "active_sources": [item.source_kind for item in active],
                "override_observed": any(item.source_kind == "OVERTAKE_OVERRIDE" for item in active),
                "nominal_return_observed": nominal_return,
                "late_join_override_observed": False,
                "valid_opponent_states": sum(item.valid for item in states),
                "valid_predictions": sum(item.valid for item in predictions),
                "r_e_compute_us": distribution([item.compute_time_us for item in plans]),
                "r_c_compute_us": distribution([item.compute_time_us for item in states]),
                "r_d_compute_us": distribution([item.compute_time_us for item in predictions if item.valid]),
                "r_b_compute_us": distribution([item.compute_time_us for item in controls]),
                "r_b_hz": ((len(control_times) - 1) / (control_times[-1] - control_times[0]) if len(control_times) > 1 else None),
                "abort_count": sum(item.maneuver_state == "ABORT" for item in plans),
                "collision_time_s": collision_time[0],
                "lifecycle_tail": lifecycle[-30:],
                "source_transition_tail": source_transitions[-20:],
                "last_blue_pose": odom0[-1] if odom0 else None,
                "last_red_pose": odom1[-1] if odom1 else None,
                "wall_elapsed_s": time.monotonic() - started,
                "install_identity": install_identity,
            }
        except Exception as error:
            if args.evidence_dir:
                args.evidence_dir.mkdir(parents=True, exist_ok=True)
                partial = {
                    "schema_version": 2,
                    "error": str(error),
                    "domain": args.domain,
                    "laps": laps[0],
                    "odom0": odom0,
                    "odom1": odom1,
                    "lap_times_s": lap_times,
                    "race_start_time_s": start_stamp[0],
                    "blue_speeds_mps": blue_speeds,
                    "lifecycle": lifecycle,
                    "source_transitions": source_transitions,
                    "completion_events": completion_events,
                    "controller_receipt_times_s": [
                        item.receipt_time_ns * 1e-9 for item in controls
                    ],
                    "plan_diagnostics": [[
                        stamp_s(item.header.stamp), item.maneuver_state, item.status_reason,
                        item.minimum_predicted_separation_m, item.minimum_corridor_clearance_m,
                    ] for item in plans],
                    "prediction_diagnostics": [[
                        stamp_s(item.source_observation_time), item.valid,
                        item.source_lifecycle, item.source_confidence,
                        item.source_hit_count, item.source_miss_count,
                        [[sample.dt_s, sample.s_m, sample.d_m,
                          sample.occupancy_half_length_m, sample.occupancy_half_width_m,
                          list(sample.covariance_xy)] for sample in item.samples],
                    ] for item in predictions],
                }
                (args.evidence_dir / "raw_runtime.json").write_text(
                    json.dumps(partial, sort_keys=True, separators=(",", ":")) + "\n"
                )
                log.seek(0)
                (args.evidence_dir / "launch.log").write_text(log.read())
            log.seek(0); print(log.read()[-8000:]); raise
        finally:
            if late_process[0] is not None:
                try:
                    late_output[0], _ = late_process[0].communicate(timeout=2.0)
                except subprocess.TimeoutExpired:
                    late_process[0].terminate()
                    late_output[0], _ = late_process[0].communicate(timeout=2.0)
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            if process.poll() is None: os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=5.0)
            unregister_runtime_owner(owner_path)
        result["launch_exit_code"] = process.returncode
        time.sleep(0.5)
        node_check = subprocess.run(
            ["ros2", "node", "list"], capture_output=True, text=True, timeout=5.0,
        )
        residual_nodes = sorted(line for line in node_check.stdout.splitlines() if line.strip())
        result["residual_nodes"] = residual_nodes
        result["residual_processes"] = int(process.poll() is None or bool(residual_nodes))
        result["late_join_override_observed"] = "OVERTAKE_OVERRIDE" in late_output[0]
        result["late_join_output"] = late_output[0].strip()[-200:]
        if args.evidence_dir:
            args.evidence_dir.mkdir(parents=True, exist_ok=True)
            raw = {
                "schema_version": 2,
                "result": result,
                "odom0": odom0,
                "odom1": odom1,
                "lap_times_s": lap_times,
                "race_start_time_s": start_stamp[0],
                "blue_speeds_mps": blue_speeds,
                "lifecycle": lifecycle,
                "source_transitions": source_transitions,
                "completion_events": completion_events,
                "controller_receipt_times_s": control_times,
                "plan_diagnostics": [[
                    stamp_s(item.header.stamp), item.maneuver_state, item.status_reason,
                    item.minimum_predicted_separation_m, item.minimum_corridor_clearance_m,
                ] for item in plans],
                "prediction_diagnostics": [[
                    stamp_s(item.source_observation_time), item.valid, item.source_lifecycle,
                    item.source_confidence, item.source_hit_count, item.source_miss_count,
                    [[sample.dt_s, sample.s_m, sample.d_m,
                      sample.occupancy_half_length_m, sample.occupancy_half_width_m,
                      list(sample.covariance_xy)] for sample in item.samples],
                ] for item in predictions],
            }
            (args.evidence_dir / "raw_runtime.json").write_text(
                json.dumps(raw, sort_keys=True, separators=(",", ":")) + "\n"
            )
            log.seek(0)
            (args.evidence_dir / "launch.log").write_text(log.read())
            (args.evidence_dir / "result.json").write_text(
                json.dumps(result, sort_keys=True, indent=2) + "\n"
            )
        print(json.dumps(result, sort_keys=True))
        failures = qualification_failures(result, args.laps, args.blue_cap)
        if failures:
            print("QUALIFICATION_FAILURES=" + ",".join(failures))
            raise SystemExit(1)


if __name__ == "__main__":
    main()
