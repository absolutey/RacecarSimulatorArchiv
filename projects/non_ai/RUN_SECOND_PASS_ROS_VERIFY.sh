#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"
EVIDENCE="${EVIDENCE_ROOT:-$PWD/../RAB_SECOND_PASS_ROS_EVIDENCE}"
mkdir -p "$EVIDENCE"
echo '== SECOND PASS SOURCE RELEASE INTEGRITY =='
python3 -B VERIFY_RELEASE.py 2>&1 | tee "$EVIDENCE/VERIFY_RELEASE.log"
echo '== ROS HUMBLE =='
set +u
source /opt/ros/humble/setup.bash
set -u
: "${ROS_DISTRO:?ROS_HUMBLE_NOT_AVAILABLE}"
echo '== COLCON PACKAGES =='
colcon list | tee "$EVIDENCE/PACKAGES.txt"
echo '== FULL ROS BUILD =='
colcon build --symlink-install --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/BUILD.log"
set +u
source install/setup.bash
set -u
echo '== WATCHDOG ROS INSTALL ENTRYPOINT =='
ros2 pkg executables f1tenth_racing_control | tee "$EVIDENCE/ROS_EXECUTABLES.txt"
grep -F 'independent_guard_watchdog' "$EVIDENCE/ROS_EXECUTABLES.txt" >/dev/null || {
 echo 'BLOCKED=WATCHDOG_ENTRYPOINT_NOT_INSTALLED';exit 1; }
echo '== INSTALLED NATIVE C++ LIBRARY IDENTITY =='
NATIVE_PREFIX="$(ros2 pkg prefix f1tenth_racing_native_proof)"
NATIVE_LIB="$NATIVE_PREFIX/lib/libf1tenth_native_stop.so"
NATIVE_CPP="$NATIVE_PREFIX/share/f1tenth_racing_native_proof/source/native_stop_proof.cpp"
test -s "$NATIVE_LIB" || { echo 'BLOCKED=NATIVE_INSTALLED_LIB_MISSING'; exit 1; }
test -s "$NATIVE_CPP" || { echo 'BLOCKED=NATIVE_INSTALLED_CPP_MISSING'; exit 1; }
cp -a "$NATIVE_LIB" "$EVIDENCE/installed_native_stop.so"
sha256sum "$NATIVE_LIB" "$NATIVE_CPP" | tee "$EVIDENCE/NATIVE_LIBRARY_HASHES.txt"
python3 -B - "$NATIVE_CPP" <<'PYNATIVE'
from pathlib import Path
import hashlib,sys
actual=Path(sys.argv[1]).read_bytes()
source=Path('src/environment/f1tenth_racing_sim/f1tenth_racing_sim/safety/native_stop_proof.cpp').read_bytes()
assert hashlib.sha256(actual).digest()==hashlib.sha256(source).digest(),'NATIVE_INSTALLED_SOURCE_DRIFT'
print('V7_INSTALLED_NATIVE_CXX_SOURCE_SHA_PASS')
PYNATIVE
# Run original independent proof comparisons against installed library, not
# the legacy /tmp artifact. These are NOT WCET or H2H authority.
python3 -B TEST_NATIVE_STOP_PROOF_EXPERIMENTAL.py "$NATIVE_LIB" 2>&1 | tee "$EVIDENCE/NATIVE_STOP_PARITY.log"
python3 -B TEST_NATIVE_GUARDAHEAD_CANDIDATES.py "$NATIVE_LIB" 2>&1 | tee "$EVIDENCE/NATIVE_GUARDAHEAD_PARITY.log"
echo '== G0-G9 V2 EVIDENCE REVIEW TESTS =='
python3 -B -m pytest -q tests_integration/test_g0g9_assessment.py 2>&1 | tee "$EVIDENCE/G0G9_V2_AUDIT.log"
echo '== COMPLETE ROS TEST SUITE =='
colcon test --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/TEST.log"
echo '== COLCON TEST VERDICT =='
colcon test-result --verbose 2>&1 | tee "$EVIDENCE/TEST_RESULTS.log"
python3 - "$EVIDENCE/TEST_RESULTS.log" <<'PYTEST'
from pathlib import Path
import re,sys
s=Path(sys.argv[1]).read_text()
m=re.search(r'Summary:\s*(\d+) tests?,\s*(\d+) errors?,\s*(\d+) failures?,\s*(\d+) skipped',s)
if not m:raise SystemExit('BLOCKED=NO_COLCON_SUMMARY')
count,errors,failures,skipped=map(int,m.groups())
if errors or failures or skipped or count < 508:
 raise SystemExit(f'BLOCKED=REGRESSION_OR_MISSING_NEW_TESTS count={count} errors={errors} failures={failures} skipped={skipped}')
print(f'RAB_SECOND_PASS_REGISTERED_TESTS_PASS={count} (FIRSTPASS Ubuntu 492 + 16 new expected)')
PYTEST
echo '== FIRSTPASS CODE CONTRACT CHECK =='
python3 -B - <<'PYFIRST'
from pathlib import Path
src=Path('src/driving/algorithms/f1tenth_racing_overtaking/f1tenth_racing_overtaking')
assert (src/'runtime_environment.py').is_file()
assert (src/'state_input_contract.py').is_file()
for x in ('node.py','successor_node.py'):
    s=(src/x).read_text()
    assert 'load_verified_runtime(' in s, x
    assert 'admit_ego_state(' in s,x
control=Path('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/node.py').read_text()
assert 'with self._state_receipt_lock:' in control
assert 'self.publish(' in control
print('FIRSTPASS_H2H_MAP_STATE_AND_RB_PUBLICATION_SOURCE_CHECK_PASS')
PYFIRST

echo '== SOURCE/WORKER INTEGRATION CHECK =='
python3 -B - <<'PYWORKER'
from pathlib import Path
p=Path('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/node.py').read_text()
for token in ('control_worker_enabled', 'LatestOnlyControlWorker(',
              'source_superseded', 'control_worker_metrics',
              'with self._control_lock:'):
    assert token in p, 'WORKER_WIRING_MARKER_MISSING:'+token
print('V6_WORKER_SOURCE_WIRING_MARKERS_PASS')
PYWORKER
echo '== STRICT APPLIED QUALIFICATION UNIT TESTS ==' 
python3 -B -m pytest -q tests_integration/test_external_watchdog_applied_v4.py   2>&1 | tee "$EVIDENCE/APPLIED_AUDIT_TEST.log"
printf '%s
' 'RAB_SECOND_PASS_ROS_UNIT_GATE_ONLY;H2H/Guard 15ms/GuardAhead 7ms REALTIME QUALIFICATION NOT ASSERTED.' "EVIDENCE_ROOT=$EVIDENCE"

 echo '== SECOND PASS EVIDENCE CONTRACT =='
 python3 -B - <<'PYSECPASS'
from pathlib import Path
base=Path('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control')
n=(base/'node.py').read_text()
assert 'primary_job_submitted = self._primary_worker.submit(' in n
assert 'self._primary_worker.take(' in n
assert 'self._primary_solver' in n
ref=Path('src/platform/f1tenth_racing_reference/f1tenth_racing_reference/node.py').read_text()
assert 'H2H_ABORT_COMPLETE_NOT_AUTHORIZED_FOR_NOMINAL' in ref
assert 'H2H_BLOCKING_HOLD_COMPLETE_REQUIRES_NEW_SAFETY_PLAN' in ref
assert Path('GENERATE_SPIELBERG_RA_CANDIDATE.py').is_file()
print('SECOND_PASS_SOURCE_WIRING_PASS_NO_REALTIME_OR_H2H_QUALIFICATION')
PYSECPASS
