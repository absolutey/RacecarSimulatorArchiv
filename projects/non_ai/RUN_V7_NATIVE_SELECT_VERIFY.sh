#!/usr/bin/env bash
# Isolated selective native proof build + real test. Does not re-run already qualified packages.
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"
echo '== RELEASE MANIFEST CHECK =='
python3 -B VERIFY_RELEASE.py
set +u
source /opt/ros/humble/setup.bash
set -u
EVIDENCE="${EVIDENCE_ROOT:-$PWD/../V7_NATIVE_SELECT_EVIDENCE}"
mkdir -p "$EVIDENCE"
echo '== SELECTED NATIVE PACKAGE BUILD =='
colcon build --symlink-install --packages-select f1tenth_racing_native_proof --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/NATIVE_BUILD.log"
set +u
source install/setup.bash
set -u
echo '== SELECTED NATIVE PACKAGE TEST =='
colcon test --packages-select f1tenth_racing_native_proof --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/NATIVE_TEST.log"
echo '== TEST RESULTS =='
colcon test-result --verbose 2>&1 | tee "$EVIDENCE/NATIVE_RESULTS.log"
python3 - <<'PY_NATIVE'
from pathlib import Path
import xml.etree.ElementTree as ET
files=list(Path('build/f1tenth_racing_native_proof/test_results').rglob('test_native_binary_contract.xunit.xml'))
assert len(files)==1,('NATIVE_RESULT_FILE_COUNT',files)
r=ET.parse(files[0]).getroot()
cases=list(r.iter('testcase'))
fails=sum(any(child.tag in ('failure','error','skipped') for child in case) for case in cases)
if len(cases)!=23 or fails:
    raise SystemExit(f'BLOCKED=NATIVE_VERIFICATION tests={len(cases)} failures_errors_skipped={fails}')
print('NATIVE_23_OF_23_PASS; NO_GUARD_RT_OR_H2H_QUALIFICATION')
PY_NATIVE
echo "EVIDENCE=$EVIDENCE"
