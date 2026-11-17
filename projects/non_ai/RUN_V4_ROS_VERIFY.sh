#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"
EVIDENCE="${EVIDENCE_ROOT:-$PWD/../RAB_V4_ROS_EVIDENCE}"
mkdir -p "$EVIDENCE"
echo '== V4 RELEASE INTEGRITY =='
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
if errors or failures or skipped or count < 374:
 raise SystemExit(f'BLOCKED=REGRESSION_OR_MISSING_NEW_TESTS count={count} errors={errors} failures={failures} skipped={skipped}')
print(f'V4_ROS_REGISTERED_TESTS_PASS={count} (338 previous + 36 V4 expected)')
PYTEST
echo '== STRICT APPLIED QUALIFICATION UNIT TESTS =='
python3 -B -m pytest -q tests_integration/test_external_watchdog_applied_v4.py   2>&1 | tee "$EVIDENCE/APPLIED_AUDIT_TEST.log"
printf '%s
' 'H2H/Guard 15ms/GuardAhead 7ms REALTIME QUALIFICATION NOT ASSERTED.' "EVIDENCE_ROOT=$EVIDENCE"
