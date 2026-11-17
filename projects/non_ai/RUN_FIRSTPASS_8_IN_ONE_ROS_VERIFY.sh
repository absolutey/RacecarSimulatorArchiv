#!/usr/bin/env bash
# PASS ONE: all eight code workstreams, no simulation launch/physical qualification.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
EVIDENCE_ROOT="${EVIDENCE_ROOT:-$(dirname "$ROOT")/EVIDENCE_FIRSTPASS_8}"
export EVIDENCE_ROOT
case "$EVIDENCE_ROOT/" in "$ROOT/"*) echo 'BLOCKED=EVIDENCE_MUST_BE_OUTSIDE_RELEASE';exit 3;; esac
mkdir -p "$EVIDENCE_ROOT"
cd "$ROOT"
echo '===== PASS1 INTEGRITY + SOLO OFFLINE REGRESSION ====='
bash RUN_CAPTURE_PREFLIGHT_ONLY.sh 2>&1 | tee "$EVIDENCE_ROOT/OFFLINE_PREFLIGHT.log"
echo '===== 4 MAP/MODE PROFILES DRY-RUN, NO ROS START ====='
python3 -B FIRSTPASS_MULTIMAP_RUNNER.py > "$EVIDENCE_ROOT/MULTIMAP_PLAN.json"
echo '===== ROS UNDERLAY ====='
set +u
source /opt/ros/humble/setup.bash
set -u
: "${ROS_DISTRO:?BLOCKED=ROS_NOT_SOURCED}"
echo '===== SELECTIVE BUILD FIRST ====='
colcon build --symlink-install --packages-up-to f1tenth_racing_native_proof f1tenth_racing_control f1tenth_racing_overtaking f1tenth_racing_raceline f1tenth_racing_bringup --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE_ROOT/BUILD_SELECTED.log"
set +u
source install/setup.bash
set -u
NATIVE_PREFIX="$(ros2 pkg prefix f1tenth_racing_native_proof)"
test -x "$NATIVE_PREFIX/lib/f1tenth_racing_native_proof/native_actuator_guard" || { echo 'BLOCKED=NATIVE_GUARD_ROS_BINARY_NOT_INSTALLED';exit 4; }
echo '===== INSTALLED C++ PROOF AND TWO MAP PARITY ====='
F1_NATIVE_PROOF_LIBRARY="$NATIVE_PREFIX/lib/libf1tenth_native_stop.so" bash BUILD_NATIVE_PROOF_EXPERIMENTAL.sh 2>&1 | tee "$EVIDENCE_ROOT/NATIVE_DUAL_MAP_PARITY.log"
echo '===== SELECTED PACKAGE TESTS ====='
colcon test --packages-select f1tenth_racing_native_proof f1tenth_racing_control f1tenth_racing_overtaking f1tenth_racing_raceline f1tenth_racing_bringup --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE_ROOT/TEST_SELECTED.log"
colcon test-result --verbose 2>&1 | tee "$EVIDENCE_ROOT/RESULT_SELECTED.log"
python3 - "$EVIDENCE_ROOT/RESULT_SELECTED.log" <<'PY'
from pathlib import Path
import re,sys
text=Path(sys.argv[1]).read_text()
m=re.search(r'Summary:\s*(\d+) tests?,\s*(\d+) errors?,\s*(\d+) failures?,\s*(\d+) skipped',text)
if not m or any(int(i) for i in m.groups()[1:]):raise SystemExit('BLOCKED=SELECTED_ROS_REGRESSION')
PY
echo '===== ALL 17 PACKAGES BUILD + FULL REGRESSION ====='
colcon build --symlink-install --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE_ROOT/BUILD_FULL.log"
set +u
source install/setup.bash
set -u
colcon test --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE_ROOT/TEST_FULL.log"
colcon test-result --verbose 2>&1 | tee "$EVIDENCE_ROOT/RESULT_FULL.log"
python3 - "$EVIDENCE_ROOT/RESULT_FULL.log" <<'PY'
from pathlib import Path
import re,sys
text=Path(sys.argv[1]).read_text()
m=re.search(r'Summary:\s*(\d+) tests?,\s*(\d+) errors?,\s*(\d+) failures?,\s*(\d+) skipped',text)
if not m:raise SystemExit('BLOCKED=NO_FULL_COLCON_TEST_SUMMARY')
n,errors,failures,skipped=map(int,m.groups())
if n<685 or errors or failures or skipped:
 raise SystemExit(f'BLOCKED=FULL_ROS_TEST_REGRESSION {n=} {errors=} {failures=} {skipped=}')
print(f'FIRSTPASS_8_CODE_ROS_REGRESSION_PASS={n};RT_AND_H2H_PHYSICAL_NOT_QUALIFIED')
PY
echo '===== FINAL SIGNED-SOURCE STATIC INTEGRITY ====='
python3 -B VERIFY_RELEASE.py
printf '%s\n' 'FIRSTPASS_8_CODE_CANDIDATE_GATE_PASS;NO_PHYSICAL_VALIDATION_OR_G0G9_PROMOTION' "EVIDENCE_ROOT=$EVIDENCE_ROOT"
