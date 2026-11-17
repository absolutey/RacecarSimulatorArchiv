#!/usr/bin/env bash
# F1TENTH isolated integrated candidate: READ ONLY full R-B and Guard state receipt capture (RCA2).
# READ/ROS only; does not modify repository, launch config or safety policy.
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$WS/install/setup.bash" ]]; then
    echo 'MISSING_ROS_OR_NEXT_SETUP' >&2; exit 2
fi
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-169}"  # caller may isolate further
OUT="$HOME/Downloads/F1TENTH_RAB_CONSOLIDATED_SOLO_$(date +%Y%m%dT%H%M%S)"
mkdir -p "$OUT/ros_logs" "$OUT/runs" "$OUT/actor_traces"
export F1TENTH_FORENSIC_TRACE_DIR="$OUT/actor_traces"
python3 -B "$WS/VERIFY_RELEASE.py"
python3 -B "$WS/TEST_UNIFIED_EXECUTION.py"
python3 -B "$WS/TEST_P0_ARCHITECTURE.py"
python3 -B "$WS/TEST_RUNTIME_CONTRACT_V2.py"
python3 -B "$WS/TEST_SCHED_INSTRUMENTATION.py"
python3 -B "$WS/TEST_P0_EXECUTION_PATH.py"
python3 -B "$WS/TEST_GUARD_TICK_BINDING.py"
python3 -B "$WS/TEST_OLD_VS_NEW_GUARD_CERT.py"
python3 -B "$WS/TEST_STOP_PROOF_EQUIVALENCE.py" > "$OUT/proof_equivalence.txt" 2>&1 || {
  cat "$OUT/proof_equivalence.txt" >&2
  exit 9
}
python3 -B "$WS/BENCHMARK_GUARD_REPLAY.py" > "$OUT/guard_rt_benchmark.json" 2>&1 || {
  cat "$OUT/guard_rt_benchmark.json" >&2
  exit 9
}
export ROS_LOG_DIR="$OUT/ros_logs"
RUN_ID="LATCH_FORENSIC_$(date +%Y%m%dT%H%M%S)"
{
  printf 'WORKSPACE=%s\n' "$WS"
  printf 'ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"
  printf 'RUN_ID=%s\n' "$RUN_ID"
  printf 'MODE=ICCAS2025 Solo with patched Guard+Pursuit (NOT MPC or racing qualification)\n'
  date -Is
} > "$OUT/run_context.txt"
for pkg in f1tenth_racing_sim f1tenth_racing_control f1tenth_racing_reference f1tenth_racing_bringup f1tenth_racing_telemetry; do
  PREFIX="$(ros2 pkg prefix "$pkg")"
  [[ "$PREFIX" == "$WS/install/"* ]] || { echo "WRONG_OVERLAY:$pkg:$PREFIX" >&2; exit 3; }
  printf '%s: ' "$pkg" >> "$OUT/package_prefixes.txt"
  ros2 pkg prefix "$pkg" >> "$OUT/package_prefixes.txt"
done

# Make the exact same-stage policy and all referenced CSV hashes succeed
# BEFORE launching ROS. This prevents the 2026-10-17 prestart regression.
python3 -B "$WS/PREFLIGHT_ARTIFACTS.py" > "$OUT/preflight_artifacts.txt" 2>&1 || {
  cat "$OUT/preflight_artifacts.txt" >&2
  # No ROS process exists in this branch: do not call stop_launch before its definition.
  cd "$HOME/Downloads"
  zip -qr "$(basename "$OUT").zip" "$(basename "$OUT")"
  sha256sum "$(basename "$OUT").zip"
  echo "EVIDENCE_ZIP=$OUT.zip"
  exit 7
}
# Guardian remains enabled. No ROS parameters or safety bounds relaxed.
# Two explicit execution modes: default is the existing Python authorities.
# Native full-stop rollout may be used only in an isolated Solo simulation,
# after local model-parity gates and an exact source/binary SHA receipt.
# Even successful native runs are NOT hardware / H2H qualifications.
NATIVE_ARGS=()
if [[ "${F1TENTH_EXPERIMENT_NATIVE_SOLO:-0}" == "1" ]]; then
  bash "$WS/BUILD_NATIVE_PROOF_EXPERIMENTAL.sh" > "$OUT/native_offline_parity.log" 2>&1 || {
    cat "$OUT/native_offline_parity.log" >&2; exit 12;
  }
  LIB="${TMPDIR:-/tmp}/f1tenth_native_stop_experiment_${USER:-user}.so"
  RECEIPT="$LIB.receipt.json"
  [[ -s "$LIB" && -s "$RECEIPT" ]] || { echo NATIVE_EXPERIMENT_PROVENANCE_MISSING >&2; exit 13; }
  NATIVE_ARGS=(
    experimental_native_guard:=true native_guard_library:="$LIB" native_guard_receipt:="$RECEIPT"
    experimental_native_guardahead:=true native_guardahead_library:="$LIB" native_guardahead_receipt:="$RECEIPT"
  )
  printf 'RUN_IS_EXPERIMENTAL_NATIVE_NOT_QUALIFIED\n' >> "$OUT/run_context.txt"
fi
timeout --signal=INT --kill-after=6s 75s \
  ros2 launch f1tenth_racing_bringup race_foundation.launch.py \
  solo:=true use_r_b0:=true \
  use_rviz:=false use_opponent_perception:=false \
  use_opponent_prediction:=false use_overtaking:=false \
  use_flight_recorder:=true use_spectator_hud:=false record_rosbag:=false \
  review_phase:=R-B execution_profile:=SOLO_BASELINE controller_kind:=pursuit trajectory_candidate:=baseline \
  telemetry_run_id:="$RUN_ID" telemetry_run_root:="$OUT/runs" \
  speed_cap_mps:=AUTO fallback_speed_cap_mps:=AUTO \
  "${NATIVE_ARGS[@]}" \
  > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
# Optional read-only /proc scheduler sampling. Disabled by default because
# measuring scheduler behavior itself can create scheduling pressure.
SCHED_PID=0
if [[ "${F1TENTH_SCHED_DIAG:-0}" == "1" ]]; then
  python3 -B "$WS/SCHED_DIAG_PROBE.py" \
    --out "$OUT/scheduler_trace.jsonl" --interval-ms "${F1TENTH_SCHED_SAMPLE_MS:-10}" --duration-s 75 \
    > "$OUT/scheduler_probe.log" 2>&1 &
  SCHED_PID=$!
fi
STOP_DONE=0
COLLECTOR_PID=0
QOS_PID=0
stop_scheduler() {
  if [[ "${SCHED_PID:-0}" -gt 0 ]]; then
    kill -INT "$SCHED_PID" 2>/dev/null || true
    wait "$SCHED_PID" 2>/dev/null || true
    SCHED_PID=0
  fi
}
stop_collector() {
  if [[ "$COLLECTOR_PID" -gt 0 ]]; then
    kill -INT "$COLLECTOR_PID" 2>/dev/null || true
    wait "$COLLECTOR_PID" 2>/dev/null || true
    COLLECTOR_PID=0
  fi
}
stop_qos() {
  if [[ "$QOS_PID" -gt 0 ]]; then
    kill -INT "$QOS_PID" 2>/dev/null || true
    wait "$QOS_PID" 2>/dev/null || true
    QOS_PID=0
  fi
}
stop_launch() {
  if [[ "$STOP_DONE" -eq 0 ]]; then
    STOP_DONE=1
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    set +e
    wait "$LAUNCH_PID"
    LAUNCH_RC=$?
    set -e
    printf 'LAUNCH_EXIT_CODE=%s\n' "$LAUNCH_RC" > "$OUT/exit_status.txt"
  fi
}
trap 'stop_scheduler; stop_qos; stop_collector; stop_launch' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
sleep 6

{
  echo '===== INITIAL ROS SERVICES ====='
  timeout 5s ros2 service list -t || true
  echo '===== INITIAL TOPICS ====='
  timeout 5s ros2 topic list -t || true
} > "$OUT/graph_initial.txt" 2>&1

# Capture full std_msgs/String data (not truncated ROS topic echo repr).
# Also observe state/intent arrival timestamps. NO publications or ROS services.
python3 -B "$(dirname "$0")/capture_full_topics.py" --out "$OUT/full_topic_trace" \
  > "$OUT/topic_collector.log" 2>&1 &
COLLECTOR_PID=$!
python3 -B "$WS/state_qos_probe.py" "$OUT/state_qos_probe.jsonl"   > "$OUT/state_qos_probe.log" 2>&1 &
QOS_PID=$!
# Enforce pre-start collector readiness; never run a blind race without evidence.
for i in {1..20}; do
  if grep -q 'FULL_TOPIC_CAPTURE_READY' "$OUT/topic_collector.log" 2>/dev/null; then break; fi
  if ! kill -0 "$COLLECTOR_PID" 2>/dev/null; then
    echo 'FULL_TOPIC_CAPTURE_START_FAILED' >&2
    cat "$OUT/topic_collector.log" >&2
    exit 4
  fi
  sleep 0.2
done
if ! grep -q 'FULL_TOPIC_CAPTURE_READY' "$OUT/topic_collector.log"; then
  echo 'FULL_TOPIC_CAPTURE_NOT_READY' >&2
  exit 4
fi

# Verify S0-R/R-B/Guard readiness BEFORE issuing a START service request.
# Never fabricate reference/Guard readiness or bypass the official start.
python3 -B "$WS/PREFLIGHT_LIVE.py" "$OUT/preflight_live.json" || {
  echo 'LIVE_PRESTART_GATE_FAILED; START_NOT_SENT' >&2
  cat "$OUT/preflight_live.json" >&2 || true
  START_SUCCESS=0
  printf 'START_SERVICE_ACCEPTED=0\n' | tee "$OUT/start_result.txt"
  stop_qos; stop_collector; stop_launch
  trap - EXIT INT TERM
  cd "$HOME/Downloads"
  zip -qr "$(basename "$OUT").zip" "$(basename "$OUT")"
  sha256sum "$(basename "$OUT").zip"
  echo "EVIDENCE_ZIP=$OUT.zip"
  exit 8
}
START_SUCCESS=0
for i in {1..3}; do
  echo "===== START_ATTEMPT=$i =====" | tee -a "$OUT/race_start_service.txt"
  set +e
  RESP="$(timeout -k 2s 6s ros2 service call /race_start std_srvs/srv/Trigger '{}' 2>&1)"
  CALL_RC=$?
  set -e
  printf 'RC=%s\n%s\n' "$CALL_RC" "$RESP" | tee -a "$OUT/race_start_service.txt"
  if grep -Eq 'success[=: ]+True' <<< "$RESP"; then
    START_SUCCESS=1
    break
  fi
  sleep 2
done
printf 'START_SERVICE_ACCEPTED=%s\n' "$START_SUCCESS" | tee "$OUT/start_result.txt"

{
  echo '===== GRAPH AFTER START ATTEMPTS ====='
  timeout 6s ros2 topic list -t || true
  for TOPIC in /race_start_time /race_readiness /state0 /active_target_trajectory /controller_status0 /actuator_intent0 /actuator_guard_status0 /ackermann_cmd0 /actuator_applied_receipt0; do
    echo "===== $TOPIC ====="
    timeout 6s ros2 topic info -v "$TOPIC" || true
  done
} > "$OUT/graph_after_start.txt" 2>&1

# Bounded topic probes, no topic publications or control commands.
for TOPIC in /race_readiness /actuator_guard_status0 /controller_status0 /state0 /actuator_applied_receipt0; do
  NAME="${TOPIC#/}"
  timeout -k 2s 5s ros2 topic echo --once "$TOPIC" \
    > "$OUT/sample_${NAME}.txt" 2>&1 || true
done
if [[ "$START_SUCCESS" -eq 1 ]]; then sleep 20; fi
stop_scheduler
stop_qos
stop_collector
stop_launch
trap - EXIT INT TERM

{
  echo '===== KEY LAUNCH LOGS ====='
  grep -Ein 'ERROR|Traceback|Exception|NOMINAL_LOAD_FAILED|START|SAFE_HOLD|GUARD_REJECT|STOP_CERTIFICATE|TIMEOUT|FINALIZED|TELEMETRY=|EVENTS=' "$OUT/launch.log" | tail -140 || true
  echo '===== LOG TAIL ====='
  tail -80 "$OUT/launch.log"
  echo '===== FUTURE TICK DEFER/REJECT COUNTS ====='
  grep -Ec 'F1TENTH_GUARD_FUTURE_TICK_DEFERRED' "$OUT/launch.log" || true
  grep -Ec 'FUTURE_STATE_.*|STOP_CERTIFICATE_NOT_VERIFIED|STATE_WATCHDOG_TIMEOUT' "$OUT/launch.log" || true
} > "$OUT/summary.txt"
python3 -B "$WS/ANALYZE_FORENSIC_TRACE.py" "$OUT" > "$OUT/trace_analysis_console.log" 2>&1 || true
if [[ -f "$OUT/scheduler_trace.jsonl" ]]; then
  python3 -B "$WS/ANALYZE_SCHED_CAUSAL.py" "$OUT" \
    > "$OUT/scheduler_attribution.json" 2>"$OUT/scheduler_attribution_error.log" || true
fi
cd "$HOME/Downloads"
zip -qr "$(basename "$OUT").zip" "$(basename "$OUT")"
sha256sum "$(basename "$OUT").zip"
echo "START_SERVICE_ACCEPTED=$START_SUCCESS"
echo "EVIDENCE_ZIP=$OUT.zip"
echo 'END. Start accepted does not mean a safe lap or high-speed MPC PASS.'
