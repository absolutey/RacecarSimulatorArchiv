#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"
EVIDENCE="${EVIDENCE_ROOT:-$PWD/../RAB_V3_ROS_EVIDENCE}"
mkdir -p "$EVIDENCE"
echo '== V3 RELEASE CHECK =='
python3 -B VERIFY_RELEASE.py 2>&1 | tee "$EVIDENCE/VERIFY_RELEASE.log"
echo '== ROS2 HUMBLE =='
set +u
source /opt/ros/humble/setup.bash
set -u
: "${ROS_DISTRO:?ROS_ENVIRONMENT_NOT_LOADED}"
echo '== ROS FULL BUILD =='
colcon build --symlink-install --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/BUILD.log"
set +u
source install/setup.bash
set -u
echo '== FULL ROS TEST =='
colcon test --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/TEST.log"
echo '== TEST RESULT =='
colcon test-result --verbose 2>&1 | tee "$EVIDENCE/TEST_RESULTS.log"
python3 - "$EVIDENCE/TEST_RESULTS.log" <<'PYTEST'
from pathlib import Path
import re,sys
s=Path(sys.argv[1]).read_text()
m=re.search(r'Summary:\s*(\d+) tests?,\s*(\d+) errors?,\s*(\d+) failures?,\s*(\d+) skipped',s)
if not m:
    raise SystemExit('BLOCKED=NO_STRUCTURED_TEST_SUMMARY')
count, errors, failures, skipped=map(int,m.groups())
if errors or failures or skipped or count<338:
    raise SystemExit(f'BLOCKED=V3_REGRESSION_COUNT_OR_FAILURE {count=} {errors=} {failures=} {skipped=}')
print(f'V3_ROS_REGISTERED_TESTS_PASS={count} (baseline 328 + 10 new expected)')
PYTEST
printf '%s\n' 'H2H/Guard REALTIME QUALIFICATION NOT ASSERTED.' "EVIDENCE_ROOT=$EVIDENCE"
