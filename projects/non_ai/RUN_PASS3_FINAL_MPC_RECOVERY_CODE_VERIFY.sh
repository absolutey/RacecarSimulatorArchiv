#!/usr/bin/env bash
# Third-pass code closure only; never launches Solo/H2H. Existing 695 PASS preserved.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
EVIDENCE_ROOT="${EVIDENCE_ROOT:-$(dirname "$ROOT")/EVIDENCE_PASS3_MPC_RECOVERY}"
case "$EVIDENCE_ROOT/" in "$ROOT/"*) echo 'BLOCKED=EVIDENCE_IN_HASHED_RELEASE'; exit 3;; esac
mkdir -p "$EVIDENCE_ROOT"
cd "$ROOT"
python3 -B VERIFY_RELEASE.py 2>&1 | tee "$EVIDENCE_ROOT/VERIFY_PRE.log"
# These tests are Python-only and run before touching any ROS package.
PYTHONPATH="$ROOT/src/driving/algorithms/f1tenth_racing_control:$ROOT/src/platform/f1tenth_racing_geometry:$ROOT/src/environment/f1tenth_racing_sim:${PYTHONPATH:-}" \
  python3 -B -m pytest -q \
  src/driving/algorithms/f1tenth_racing_control/test/test_process_primary_worker_firstpass.py \
  src/driving/algorithms/f1tenth_racing_control/test/test_process_primary_recovery_thirdpass.py \
  2>&1 | tee "$EVIDENCE_ROOT/MPC_RECOVERY_OFFLINE_5.log"
set +u
source /opt/ros/humble/setup.bash
if [ -f "$ROOT/install/setup.bash" ]; then source "$ROOT/install/setup.bash"; fi
set -u
# Only the actually changed ROS package is rebuilt; other 16 stay installed.
colcon build --symlink-install --packages-select f1tenth_racing_control \
  --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE_ROOT/SELECTIVE_BUILD_CONTROL.log"
set +u
source "$ROOT/install/setup.bash"
set -u
colcon test --packages-select f1tenth_racing_control \
  --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE_ROOT/SELECTIVE_TEST_CONTROL.log"
# Registered python tests must really have been collected.
python3 - "$ROOT/build/f1tenth_racing_control/pytest.xml" <<'PY_GATE'
from pathlib import Path
import xml.etree.ElementTree as ET
import sys
p=Path(sys.argv[1]); assert p.is_file(),'BLOCKED=CONTROL_JUNIT_MISSING'
r=ET.parse(p).getroot()
count=sum(int(x.attrib.get('tests',0)) for x in (list(r.iter('testsuite')) or [r]))
invalid=sum(int(x.attrib.get(k,0)) for x in (list(r.iter('testsuite')) or [r]) for k in ('errors','failures','skipped'))
if count<218 or invalid: raise SystemExit(f'BLOCKED=CONTROL_TEST_CASES n={count} invalid={invalid}')
classes=[x.attrib.get('name','') for x in r.iter('testcase')]
for name in ('test_timeout_discards_entire_generation_before_next_primary',
             'test_process_crash_uses_fallback_until_new_generation_ready',
             'test_shutdown_denies_new_generation_and_no_primary_reactivation'):
    if name not in classes: raise SystemExit(f'BLOCKED=CONTROL_NEW_TEST_NOT_REGISTERED {name}')
print('PASS3_MPC_RECOVERY_SELECTED_TEST_PASS',count)
PY_GATE
# Full ROS regression: no new build, no skipped tests, all 17 packages.
colcon test --event-handlers console_direct+ \
  2>&1 | tee "$EVIDENCE_ROOT/ALL_REGISTERED_TESTS.log"
colcon test-result --verbose 2>&1 | tee "$EVIDENCE_ROOT/ALL_TEST_RESULTS.log"
python3 - "$EVIDENCE_ROOT/ALL_TEST_RESULTS.log" <<'PY_GATE'
from pathlib import Path
import re,sys
text=Path(sys.argv[1]).read_text()
m=re.search(r'Summary:\s*(\d+) tests?,\s*(\d+) errors?,\s*(\d+) failures?,\s*(\d+) skipped',text)
if not m: raise SystemExit('BLOCKED=NO_FULL_TEST_SUMMARY')
n,e,f,k=map(int,m.groups())
if n<698 or e or f or k:
    raise SystemExit(f'BLOCKED=FULL_TEST_NOT_PASS n={n} errors={e} failures={f} skipped={k}')
print('PASS3_FINAL_17PACKAGES_REGISTERED_CODE_REGRESSION_PASS',n,e,f,k)
PY_GATE
python3 -B VERIFY_RELEASE.py 2>&1 | tee "$EVIDENCE_ROOT/VERIFY_FINAL.log"
echo 'RESULT=PASS3_FINAL_MPC_RECOVERY_CODE_GATE_PASS;SOLO_H2H_RT_NOT_QUALIFIED'
echo "EVIDENCE_ROOT=$EVIDENCE_ROOT"
