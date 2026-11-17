#!/usr/bin/env bash
# Localized RAB second-pass S0-R failure replay; leaves other package tests alone.
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"
EVIDENCE="${EVIDENCE_ROOT:-$PWD/../RAB_REFERENCE_SELECT_EVIDENCE}"
mkdir -p "$EVIDENCE"
python3 -B VERIFY_RELEASE.py 2>&1 | tee "$EVIDENCE/VERIFY_RELEASE.log"
set +u
source /opt/ros/humble/setup.bash
set -u
# Build only the package and its missing dependencies; do not run other package tests.
colcon build --symlink-install --packages-up-to f1tenth_racing_reference \
  --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/REFERENCE_BUILD.log"
set +u
source install/setup.bash
set -u
colcon test --packages-select f1tenth_racing_reference \
  --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/REFERENCE_TEST.log"
colcon test-result --test-result-base build/f1tenth_racing_reference --verbose \
  2>&1 | tee "$EVIDENCE/REFERENCE_RESULTS.log"
python3 -B - <<'PYVERIFY'
from pathlib import Path
import xml.etree.ElementTree as ET
p=Path('build/f1tenth_racing_reference/pytest.xml')
if not p.exists():raise SystemExit('BLOCKED=REFERENCE_JUNIT_MISSING')
root=ET.parse(p).getroot()
case=list(root.iter('testcase'))
bad=sum(bool(list(c.iter('failure')) or list(c.iter('error')) or list(c.iter('skipped'))) for c in case)
if len(case)<50 or bad:
 raise SystemExit(f'BLOCKED=REFERENCE_TESTS_COUNT_OR_FAILURE tests={len(case)} failed={bad}')
print(f'REFERENCE_SELECT_PASS={len(case)} tests, 0 errors/failures/skips')
PYVERIFY
printf 'EVIDENCE=%s\n' "$EVIDENCE"
