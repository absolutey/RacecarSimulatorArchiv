#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
ROOT="$(pwd -P)"
EVIDENCE="$(realpath -m "${EVIDENCE_ROOT:-$ROOT/../EVIDENCE_STRUCTURAL_IMPLEMENTATION}")"
case "$EVIDENCE" in
  "$ROOT"|"$ROOT"/*)
    echo 'BLOCKED=EVIDENCE_ROOT_INSIDE_RELEASE_TREE' >&2
    exit 12
    ;;
esac
mkdir -p "$EVIDENCE"
python3 -B VERIFY_RELEASE.py | tee "$EVIDENCE/SOURCE_VERIFY.log"
python3 -B -m pytest -q src/platform/f1tenth_racing_reference/test/test_nested_stage_provenance_fix.py \
  | tee "$EVIDENCE/STAGE_PROVENANCE_OFFLINE.log"
set +u
source /opt/ros/humble/setup.bash
set -u
# Maintain isolated build from the entire candidate; no canonical workspace edits.
colcon build --symlink-install --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/BUILD.log"
set +u
source install/setup.bash
set -u
ros2 pkg prefix f1tenth_racing_native_proof | tee "$EVIDENCE/NATIVE_PREFIX.txt"
python3 -B TEST_NATIVE_STOP_PROOF_EXPERIMENTAL.py "$(ros2 pkg prefix f1tenth_racing_native_proof)/lib/libf1tenth_native_stop.so" \
  2>&1 | tee "$EVIDENCE/NATIVE_PARITY.log"
# One selected integration pass first; no source fallback/rollback on failure.
colcon test --packages-select f1tenth_racing_overtaking f1tenth_racing_reference f1tenth_racing_raceline f1tenth_racing_control f1tenth_racing_sim \
  --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/SELECTED_TEST.log"
colcon test-result --verbose 2>&1 | tee "$EVIDENCE/SELECTED_RESULT.log"
# The final full regression includes all 17 packages.
colcon test --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/FULL_TEST.log"
colcon test-result --verbose 2>&1 | tee "$EVIDENCE/FULL_RESULT.log"
python3 - "$EVIDENCE/FULL_RESULT.log" <<'PYVERIFY'
import re,sys
from pathlib import Path
s=Path(sys.argv[1]).read_text()
m=re.findall(r'Summary:\s*(\d+) tests?,\s*(\d+) errors?,\s*(\d+) failures?,\s*(\d+) skipped',s)
if not m:raise SystemExit('BLOCKED=NO_COLCON_TEST_SUMMARY')
count,errors,failed,skipped=map(int,m[-1])
if (count<535 or errors or failed or skipped):
 raise SystemExit(f'BLOCKED=STRUCTURAL_CODE_REGRESSION count={count} errors={errors} failures={failed} skipped={skipped}')
print(f'ROS_REGISTERED_TESTS_PASS={count} (508 baseline + 10 structural tests + 17 provenance tests)')
print('CODE_REGRESSION_ONLY_NO_WCET_OR_H2H_E2E_QUALIFICATION')
PYVERIFY
python3 -B -m pytest -q tests_integration/test_external_watchdog_applied_v4.py | tee "$EVIDENCE/STRICT_APPLIED_UNIT.log"
echo "EVIDENCE_ROOT=$EVIDENCE"
