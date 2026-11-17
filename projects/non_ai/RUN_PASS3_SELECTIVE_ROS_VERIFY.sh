#!/usr/bin/env bash
# PASS3 FIX: CMake test-registration proof; code only; no Solo/H2H launch.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
EVIDENCE_ROOT="${EVIDENCE_ROOT:-$(dirname "$ROOT")/EVIDENCE_PASS3_CTEST_REGISTERED}"
case "$EVIDENCE_ROOT/" in "$ROOT/"*) echo 'BLOCKED=EVIDENCE_INSIDE_HASHED_RELEASE';exit 3;; esac
mkdir -p "$EVIDENCE_ROOT"
cd "$ROOT"
python3 -B VERIFY_RELEASE.py 2>&1 | tee "$EVIDENCE_ROOT/VERIFY_PRE.log"
python3 -B PASS3_OFFLINE_SOURCE_AUDIT.py "$EVIDENCE_ROOT/AUDIT_MATRIX.json" 2>&1 | tee "$EVIDENCE_ROOT/AUDIT_STATIC.log"
python3 -B -m pytest -q \
 src/platform/f1tenth_racing_native_proof/test/test_native_guard_identity_thirdpass.py \
 src/platform/f1tenth_racing_native_proof/test/test_native_guard_humble_api_secondpass.py 2>&1 | tee "$EVIDENCE_ROOT/NATIVE_OFFLINE_7.log"
python3 -B FIRSTPASS_MULTIMAP_RUNNER.py > "$EVIDENCE_ROOT/MULTIMAP_DRY_RUN.json"
# No source or message contracts are changed. Reconfigure ONLY the affected native package:
# the last run still had just 1 CTest (686 total), although CMakeLists specifies 3.
set +u
source /opt/ros/humble/setup.bash
if [ -f "$ROOT/install/setup.bash" ]; then source "$ROOT/install/setup.bash"; fi
set -u
colcon build --symlink-install --packages-select f1tenth_racing_native_proof \
 --cmake-force-configure --event-handlers console_direct+ \
 2>&1 | tee "$EVIDENCE_ROOT/BUILD_NATIVE_RECONFIGURED.log"
# Merely getting a successful CMake exit isn't sufficient. Prove CTest registry was updated.
ctest --test-dir "$ROOT/build/f1tenth_racing_native_proof" -N \
 2>&1 | tee "$EVIDENCE_ROOT/NATIVE_CTEST_REGISTRY.log"
python3 - "$EVIDENCE_ROOT/NATIVE_CTEST_REGISTRY.log" <<'PY_GATE'
from pathlib import Path
import re,sys
s=Path(sys.argv[1]).read_text()
registered=re.findall(r'Test\s*#\d+:\s*([^\s]+)',s)
required={'test_native_binary_contract','test_native_guard_humble_api_secondpass',
          'test_native_guard_identity_thirdpass'}
missing=required-set(registered)
if missing or len(registered)!=len(set(registered)):
    raise SystemExit(f'BLOCKED=CTEST_REGISTRATION_MISSING missing={sorted(missing)} found={registered}')
print('PASS3_NATIVE_CTEST_REGISTRATION_PASS',registered)
PY_GATE
set +u
source "$ROOT/install/setup.bash"
set -u
colcon test --packages-select f1tenth_racing_native_proof \
 --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE_ROOT/TEST_NATIVE_SELECTED.log"
python3 - "$ROOT/build/f1tenth_racing_native_proof/test_results/f1tenth_racing_native_proof" <<'PY_GATE'
from pathlib import Path
import sys,xml.etree.ElementTree as ET
root=Path(sys.argv[1]);required={
 'test_native_guard_humble_api_secondpass':4,
 'test_native_guard_identity_thirdpass':3,
 'test_native_binary_contract':24,
}
for name,min_cases in required.items():
    p=root/(name+'.xunit.xml')
    if not p.is_file():raise SystemExit(f'BLOCKED=NATIVE_JUNIT_MISSING {p}')
    tree=ET.parse(p).getroot()
    suites=[tree] if tree.tag=='testsuite' else list(tree.iter('testsuite'))
    counts={k:sum(int(x.attrib.get(k,0)) for x in suites) for k in ('tests','errors','failures','skipped')}
    if counts['tests']<min_cases or any(counts[k] for k in ('errors','failures','skipped')):
        raise SystemExit(f'BLOCKED=NATIVE_JUNIT_INCOMPLETE {name} {counts} expected>={min_cases}')
    print(f'PASS3_NATIVE_JUNIT_PASS {name} {counts}')
PY_GATE
colcon test-result --verbose 2>&1 | tee "$EVIDENCE_ROOT/RESULT_NATIVE_SELECTED.log"
python3 - "$EVIDENCE_ROOT/RESULT_NATIVE_SELECTED.log" <<'PY_GATE'
from pathlib import Path
import re,sys
s=Path(sys.argv[1]).read_text()
m=re.search(r'Summary:\s*(\d+) tests?,\s*(\d+) errors?,\s*(\d+) failures?,\s*(\d+) skipped',s)
if not m:raise SystemExit('BLOCKED=NO_SELECTED_TEST_SUMMARY')
n,e,f,k=map(int,m.groups())
if any((e,f,k)):raise SystemExit(f'BLOCKED=SELECTED_TEST_FAILED n={n} e={e} f={f} k={k}')
print('PASS3_NATIVE_REGISTERED_AND_SELECTED_PASS',n,e,f,k)
PY_GATE
# No recompilation of the other 16 passing packages; exercise ALL registered tests.
colcon test --event-handlers console_direct+ \
 2>&1 | tee "$EVIDENCE_ROOT/TEST_ALL_REGISTERED.log"
colcon test-result --verbose 2>&1 | tee "$EVIDENCE_ROOT/RESULT_ALL_REGISTERED.log"
python3 - "$EVIDENCE_ROOT/RESULT_ALL_REGISTERED.log" <<'PY_GATE'
from pathlib import Path
import re,sys
s=Path(sys.argv[1]).read_text()
m=re.search(r'Summary:\s*(\d+) tests?,\s*(\d+) errors?,\s*(\d+) failures?,\s*(\d+) skipped',s)
if not m:raise SystemExit('BLOCKED=PASS3_NO_FULL_TEST_SUMMARY')
n,e,f,k=map(int,m.groups())
# PASS2 686 genuine cases + 4 Humble API + 3 Guard identity cases.
if n<693 or any((e,f,k)):
    raise SystemExit(f'BLOCKED=PASS3_FULL_REGISTERED_TEST_FAIL n={n} e={e} f={f} k={k}')
print('PASS3_FULL_REGISTERED_ROS_REGRESSION_PASS',n,e,f,k)
PY_GATE
python3 -B VERIFY_RELEASE.py 2>&1 | tee "$EVIDENCE_ROOT/VERIFY_FINAL.log"
echo 'RESULT=PASS3_CODE_REGISTERED_ROS_REGRESSION_PASS;H2H_WCET_NOT_QUALIFIED'
echo "EVIDENCE_ROOT=$EVIDENCE_ROOT"
