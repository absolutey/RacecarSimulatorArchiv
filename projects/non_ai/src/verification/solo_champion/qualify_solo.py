#!/usr/bin/env python3
import argparse
import csv
import math
import yaml
from ament_index_python.packages import get_package_share_directory
from physical_lap_oracle import Observation,qualify_physical_laps
import json
import os
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'qualification_gates'))
from applied_lineage import AppliedLineageAudit
import signal
import statistics
import subprocess
import sys
import tempfile
import time

REPOSITORY = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPOSITORY / "src/verification/h2h_champion"))
from resource_preflight import (register_runtime_owner, require_qualification_preflight,
                                stop_owned_group, unregister_runtime_owner)

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from control_msgs.msg import ActiveTargetTrajectory, CarState, ControllerStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, Int32, String
from std_srvs.srv import Trigger


def distribution(values):
    ordered = sorted(values)
    percentile = lambda q: ordered[min(len(ordered) - 1, round(q * (len(ordered) - 1)))]
    return {
        "mean": statistics.fmean(values),
        "p95": percentile(0.95),
        "p99": percentile(0.99),
        "max": max(values),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--speed-cap", type=float, required=True)
    parser.add_argument("--environment-file", type=Path,
        help="Explicit installed map environment; required for Spielberg")
    parser.add_argument("--laps", type=int, default=3)
    parser.add_argument("--trajectory", choices=("baseline", "scp"), default="baseline")
    parser.add_argument("--controller", choices=("pursuit", "mpc"), default="pursuit")
    parser.add_argument("--candidate", choices=("B0", "B1", "B2"))
    parser.add_argument("--profile", default="default")
    parser.add_argument("--purpose", choices=("DEFAULT", "QUALIFICATION", "BLIND"),
                        default="DEFAULT")
    parser.add_argument("--review-phase", choices=("R-A", "R-B"), default="R-A")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--evidence-root")
    parser.add_argument("--run-id", default="")
    parser.add_argument("--invalidate-after-laps", type=int)
    parser.add_argument("--champion-user-path", action="store_true")
    parser.add_argument("--diagnostic-withhold-primary", action="store_true")
    args = parser.parse_args()
    if args.candidate:
        if args.candidate == "B2":
            raise ValueError("B2_NMPC_NOT_IMPLEMENTED: no corresponding native solver; cannot mislabel MPC")
        args.controller = {"B0": "pursuit", "B1": "mpc"}[args.candidate]
    candidate = args.candidate or ("B0" if args.controller == "pursuit" else "B1")
    if args.speed_cap <= 0.0:
        raise ValueError("speed cap must be positive")
    execution_profile = ("SOLO_BASELINE" if args.controller == "pursuit" and args.trajectory == "baseline"
                         else "SOLO_SCP_EXPERIMENTAL" if args.controller == "pursuit" and args.trajectory == "scp"
                         else "SOLO_MPC_EXPERIMENTAL" if args.controller == "mpc" and args.trajectory == "scp"
                         else None)
    if execution_profile is None and not (args.environment_file is not None and
            args.controller=='mpc' and args.trajectory=='baseline'):
        raise ValueError("SOLO_EXECUTION_PROFILE_MISMATCH: MPC requires SCP reference except provisional Spielberg")
    if args.evidence_root and not args.run_id:
        raise ValueError("--run-id is required with --evidence-root")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    resource_preflight = require_qualification_preflight(REPOSITORY)
    env_file=(args.environment_file if args.environment_file is not None else
              Path(get_package_share_directory('f1tenth_racing_sim')) /
              'config/competition_environment.yaml')
    environment=yaml.safe_load(env_file.read_text())
    if (environment['environment_identity']['active_track']=='spielberg' and
            args.controller=='mpc' and args.trajectory=='baseline'):
        execution_profile='SOLO_SPIELBERG_MPC_PRELIMINARY'
    if execution_profile is None:
        raise ValueError('UNSUPPORTED_MAP_OR_CONTROLLER_REFERENCE_COMBINATION')
    if (environment['environment_identity']['active_track']=='spielberg' and
            execution_profile!='SOLO_SPIELBERG_MPC_PRELIMINARY'):
        raise ValueError('SPIELBERG_SCP_REFERENCE_NOT_REGISTERED: use --controller mpc --trajectory baseline (preliminary)')
    sim_share=Path(get_package_share_directory('f1tenth_racing_sim'))
    centerline_file=sim_share / environment['map']['centerline_file']
    if not centerline_file.is_file():
        raise ValueError('PHYSICAL_ORACLE_CENTERLINE_MISSING')
    command = (["ros2", "run", "f1tenth_racing_tools", "f1solo"]
               if args.champion_user_path else [
                   "ros2", "launch", "f1tenth_racing_bringup", "race_foundation.launch.py",
               ])
    command.extend([
        "solo:=true", "use_rviz:=false", f"speed_cap_mps:={args.speed_cap}",
        f"execution_profile:={execution_profile}",
        f"environment_file:={env_file}",
        f"trajectory_candidate:={args.trajectory}",
        f"controller_kind:={args.controller}",
        f"controller_profile:={args.profile}", f"candidate_id:={candidate}",
    ])
    if args.evidence_root:
        command.extend([
            "use_flight_recorder:=true", "record_rosbag:=false",
            f"review_phase:={args.review_phase}",
            f"telemetry_run_root:={args.evidence_root}", f"telemetry_run_id:={args.run_id}",
        ])
    if args.diagnostic_withhold_primary:
        command.append("diagnostic_withhold_primary:=true")
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
        )
        owner_path = register_runtime_owner(
            process.pid, REPOSITORY, args.domain, args.run_id or f"{candidate}-{args.profile}"
        )
        rclpy.init()
        node = Node(f"solo_qualification_{args.domain}")
        laps, walls, valid = [None], [None], [None]
        lap_times, statuses, commands, states, state_times, command_times = [], [], [], [], [], []
        node.create_subscription(Int32, "lap_count0", lambda m: laps.__setitem__(0, m.data), 1)
        node.create_subscription(Int32, "wall_collision_count0", lambda m: walls.__setitem__(0, m.data), 1)
        node.create_subscription(Bool, "race_valid", lambda m: valid.__setitem__(0, m.data), 1)
        node.create_subscription(Float64, "lap_time0", lambda m: lap_times.append(m.data), 10)
        node.create_subscription(ControllerStatus, "controller_status0", statuses.append, 10)
        def receive_state(message):
            states.append(message)
            state_times.append(time.perf_counter())

        node.create_subscription(CarState, "state0", receive_state, 10)
        atomic_lineage=AppliedLineageAudit(require_external_watchdog=True)
        node.create_subscription(String,'actuator_applied_receipt0',
                                 lambda m:atomic_lineage.observe(m.data),10)
        invalid_pub = None
        fault_injected = False
        if args.invalidate_after_laps is not None:
            from f1tenth_racing_reference import invalid_active_reference
            from f1tenth_racing_reference.node import ACTIVE_REFERENCE_QOS
            from f1tenth_racing_reference.transport import to_message
            invalid_pub = node.create_publisher(
                ActiveTargetTrajectory, "active_target_trajectory", ACTIVE_REFERENCE_QOS,
            )
            invalid_message = to_message(invalid_active_reference("R_B4_FAULT_INJECTION"))

        def receive_command(message):
            commands.append(message)
            command_times.append(time.perf_counter())

        command_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(AckermannDriveStamped, "ackermann_cmd0", receive_command, command_qos)
        client = node.create_client(Trigger, "race_start")
        started = time.monotonic()
        try:
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError("race_start unavailable")
            deadline = time.monotonic() + 10.0
            blue_publishers = []
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                blue_publishers = node.get_publishers_info_by_topic("/ackermann_cmd0")
                if (len(blue_publishers) == 1 and
                        blue_publishers[0].node_name == "r_b0_racing_pursuit"):
                    break
            if len(blue_publishers) != 1 or blue_publishers[0].node_name != "r_b0_racing_pursuit":
                raise RuntimeError(f"unexpected Blue command owners: {blue_publishers}")
            if node.get_publishers_info_by_topic("/state1"):
                raise RuntimeError("solo profile unexpectedly has Red state publisher")
            state_deadline = time.monotonic() + 5.0
            while not states and time.monotonic() < state_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not states:
                raise RuntimeError("Blue state unavailable before race start")
            if args.evidence_root:
                manifest = Path(args.evidence_root) / args.run_id / "manifest.json"
                recorder_deadline = time.monotonic() + 30.0
                while time.monotonic() < recorder_deadline:
                    if manifest.exists() and json.loads(manifest.read_text()).get("lifecycle") == "PREARMED":
                        break
                    time.sleep(0.05)
                else:
                    raise RuntimeError("flight recorder did not reach PREARMED")
            prestart_deadline = time.monotonic() + 0.5
            while time.monotonic() < prestart_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            prestart_speed = max((abs(state.v) for state in states), default=float("inf"))
            if prestart_speed > 1e-9:
                raise RuntimeError(
                    f"Blue was not stationary before race start: {prestart_speed} m/s"
                )
            if args.champion_user_path:
                start_result = subprocess.run(
                    ["ros2", "run", "f1tenth_racing_tools", "f1go"],
                    check=False, capture_output=True, text=True, timeout=15.0,
                )
                if start_result.returncode != 0:
                    raise RuntimeError(f"f1go failed: {start_result.stdout}{start_result.stderr}")
            else:
                future = client.call_async(Trigger.Request())
                while not future.done():
                    rclpy.spin_once(node, timeout_sec=0.1)
            deadline = time.monotonic() + args.timeout
            target_lap_count = args.laps if args.invalidate_after_laps is not None else args.laps + 1
            while laps[0] is None or laps[0] < target_lap_count:
                if process.poll() is not None:
                    raise RuntimeError(f"launch exited early: {process.returncode}")
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"timeout laps={laps[0]}")
                rclpy.spin_once(node, timeout_sec=0.1)
                if (invalid_pub is not None and not fault_injected and
                        laps[0] is not None and laps[0] >= args.invalidate_after_laps):
                    invalid_pub.publish(invalid_message)
                    fault_injected = True
                if (fault_injected and statuses and statuses[-1].degradation_active and
                        abs(statuses[-1].actual_speed_mps) < 0.1):
                    break
                if valid[0] is False or walls[0] not in (None, 0):
                    for _ in range(5):
                        rclpy.spin_once(node, timeout_sec=0.05)
                    raise RuntimeError(f"invalid solo run valid={valid[0]} walls={walls[0]}")
            required_laps = args.invalidate_after_laps if fault_injected else args.laps
            if len(lap_times) < required_laps or not statuses or not commands:
                raise RuntimeError("incomplete runtime observations")
            with centerline_file.open(newline='') as f:
                centerline_rows=list(csv.DictReader(f))
            points=[(float(r['x_m']),float(r['y_m'])) for r in centerline_rows]
            halfwidth=min(min(float(r['w_tr_left_m']),float(r['w_tr_right_m']))
                          for r in centerline_rows)
            observed=[Observation(
                state.physics_stamp.sec+state.physics_stamp.nanosec*1e-9,
                state.px,state.py,False) for state in states]
            oracle=qualify_physical_laps(observed,points,
                corridor_half_width_m=halfwidth,
                max_vehicle_speed_mps=float(environment['dynamics']['limits']['speed_abs_max_m_s']))
            physical_lap_pass=(oracle.valid and oracle.qualified_laps>=required_laps)
            if not fault_injected and not physical_lap_pass:
                raise RuntimeError(f'PHYSICAL_LAP_ORACLE_REJECTED:{oracle.reason} '
                                   f'complete={oracle.qualified_laps}/{required_laps}')
            atomic_verdict=atomic_lineage.verdict()
            if not atomic_verdict['passed']:
                raise RuntimeError('SOLO_ATOMIC_APPLIED_LINEAGE_NOT_QUALIFIED:'+str(atomic_verdict))
            result = {
                'atomic_applied_lineage':atomic_verdict,
                'physical_oracle':{'valid':bool(oracle.valid),
                                   'qualified_laps':int(oracle.qualified_laps),
                                   'reason':oracle.reason,
                                   'forward_distance_m':oracle.forward_distance_m,
                                   'passed':bool(physical_lap_pass)},
                "domain": args.domain,
                "speed_cap_mps": args.speed_cap,
                "trajectory_candidate": args.trajectory,
                "controller_kind": args.controller,
                "candidate_id": candidate,
                "controller_profile": args.profile,
                "tournament_purpose": args.purpose,
                "fault_injected": fault_injected,
                "stationary_before_start": True,
                "prestart_peak_speed_mps": prestart_speed,
                "laps": args.laps,
                "evaluator_lap_count": laps[0],
                "lap_times_s": lap_times[:args.laps],
                "flying_lap_times_s": lap_times[:args.laps],
                "walls": walls[0],
                "valid": valid[0],
                "sole_blue_command_publisher": blue_publishers[0].node_name,
                "reference_id": statuses[-1].active_reference_id,
                "state_hz": (len(state_times) - 1) / (state_times[-1] - state_times[0]),
                "command_hz": (len(command_times) - 1) / (command_times[-1] - command_times[0]),
                "lateral_abs_m": distribution([abs(s.lateral_error_m) for s in statuses]),
                "heading_abs_rad": distribution([abs(s.heading_error_rad) for s in statuses]),
                "speed_error_abs_mps": distribution([abs(s.target_speed_mps - s.actual_speed_mps) for s in statuses]),
                "compute_us": distribution([s.compute_time_us for s in statuses]),
                "peak_actual_speed_mps": max(s.actual_speed_mps for s in statuses),
                "fallback_or_degraded_count": sum(s.degradation_active for s in statuses),
                "primary_fallback_count": sum(s.fallback_active for s in statuses),
                "primary_valid_count": sum(s.primary_status == "PRIMARY_VALID" for s in statuses),
                "primary_late_count": sum(s.primary_status == "PRIMARY_LATE" for s in statuses),
                "primary_infeasible_count": sum(s.primary_status == "PRIMARY_INFEASIBLE" for s in statuses),
                "primary_invalid_count": sum(s.primary_status == "PRIMARY_INVALID" for s in statuses),
                "primary_solve_us": distribution([s.primary_solve_time_us for s in statuses if s.primary_solve_time_us > 0.0]) if any(s.primary_solve_time_us > 0.0 for s in statuses) else None,
                "drive_speed_nonzero_count": sum(abs(c.drive.speed) > 1e-12 for c in commands),
                "steering_max_abs_rad": max(abs(c.drive.steering_angle) for c in commands),
                "acceleration_max_abs_mps2": max(abs(c.drive.acceleration) for c in commands),
                "wall_elapsed_s": time.monotonic() - started,
                "resource_preflight": resource_preflight,
            }
        except Exception:
            log.seek(0)
            print(log.read()[-5000:])
            raise
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            stop_owned_group(process.pid, timeout_s=15.0)
            if not process.poll():
                process.wait(timeout=1.0)
            unregister_runtime_owner(owner_path)
        result["launch_exit_code"] = process.returncode
        result["residual_processes"] = 0 if process.poll() is not None else 1
        print(json.dumps(result, sort_keys=True))
        if process.returncode != 0 or result["residual_processes"] != 0:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
