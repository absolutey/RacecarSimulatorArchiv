#!/usr/bin/env bash
set -euo pipefail
WS="$HOME/Downloads/F1TENTH_RAB_UNIFIED_CONSOLIDATED_IMPLEMENTATION_20261018"
[[ "$(pwd -P)" == "$WS" && -d "$WS/src" ]] || { echo 'WRONG_ISOLATED_PATH'; exit 2; }
python3 -B VERIFY_RELEASE.py
python3 -B TEST_UNIFIED_EXECUTION.py
python3 -B TEST_RUNTIME_CONTRACT_V2.py
python3 -B TEST_SCHED_INSTRUMENTATION.py
python3 -B TEST_P0_EXECUTION_PATH.py
python3 -B TEST_GUARD_TICK_BINDING.py
python3 -B TEST_OLD_VS_NEW_GUARD_CERT.py
python3 -B REPLAY_P0_TICK_STOP_PROOF.py
python3 -B TEST_INTEGRAL_MAP.py
python3 -B TEST_STOP_PROOF_EQUIVALENCE.py
python3 -B TEST_ASYNC_TRACE.py
mkdir -p build
python3 -B BENCHMARK_GUARD_REPLAY.py > build/GUARD_RT_OFFLINE_BENCHMARK.json
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install
set +u
source "$WS/install/setup.bash"
set -u
for pkg in $(colcon list --names-only); do
 P="$(ros2 pkg prefix "$pkg")"
 [[ "$P" == "$WS/install/"* ]] || { echo "WRONG_PACKAGE_ORIGIN:$pkg:$P"; exit 3; }
done
printf '===== SAME-STAGE AUTHORITY REGRESSION =====\n'
python3 -m pytest -q tests/test_stage_authority_repair.py \
  src/platform/f1tenth_racing_reference/test/test_contract_guard_surgery.py
printf '===== FULL ROS PACKAGE REGRESSION =====\n'
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 -B VERIFY_RELEASE.py
printf 'P0_INTEGRATED_BUILD_AND_REGRESSION_FINISHED; NOT_RACING_PASS\n' 
