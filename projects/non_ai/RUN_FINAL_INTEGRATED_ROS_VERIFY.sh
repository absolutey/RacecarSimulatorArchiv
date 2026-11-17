#!/usr/bin/env bash
# Validate only the isolated candidate. Never touch ~/f1tenth_racing.
set -eo pipefail
cd "$(dirname "$0")"
# Keep generated evidence OUTSIDE the signed source tree, otherwise the tee
# output races the release-hash inventory and creates an untracked file.
ROOT="$(pwd -P)"
EVIDENCE="$(realpath -m "${EVIDENCE_ROOT:-$ROOT/../EVIDENCE_FINAL_INTEGRATED}")"
case "$EVIDENCE" in
  "$ROOT"|"$ROOT"/*)
    echo 'BLOCKED=EVIDENCE_ROOT_INSIDE_RELEASE_TREE' >&2
    exit 12
    ;;
esac
mkdir -p "$EVIDENCE"
python3 -B VERIFY_RELEASE.py 2>&1 | tee "$EVIDENCE/RELEASE_HASHES.log"
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/BUILD.log"
set +u
source install/setup.bash
set -u
# First the modified packages only; preserve all other passing source.
colcon test --packages-select f1tenth_racing_sim f1tenth_racing_control f1tenth_racing_overtaking f1tenth_racing_reference f1tenth_racing_bringup --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/SELECTED.log"
colcon test-result --verbose 2>&1 | tee "$EVIDENCE/SELECTED_RESULT.log"
colcon test --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/FULL.log"
colcon test-result --verbose 2>&1 | tee "$EVIDENCE/FULL_RESULT.log"
python3 - "$EVIDENCE/FULL_RESULT.log" <<'PYVERIFY'
import re,sys
from pathlib import Path
result=Path(sys.argv[1]).read_text()
found=re.findall(r'Summary:\s*(\d+) tests?,\s*(\d+) errors?,\s*(\d+) failures?,\s*(\d+) skipped',result)
if not found:raise SystemExit('BLOCKED=NO_COLCON_VERDICT')
n,e,f,s=map(int,found[-1]); print(f'FINAL_INTEGRATED_ROS_TESTS={n} errors={e} failures={f} skipped={s}')
if n<668 or e or f or s:raise SystemExit('BLOCKED=FINAL_INTEGRATED_REGRESSION')
print('FINAL_CODE_REGRESSION_PASS; H2H_AND_REALTIME_E2E_NOT_CLAIMED')
PYVERIFY
python3 -B -m pytest -q tests_integration/test_external_watchdog_applied_v4.py 2>&1 | tee "$EVIDENCE/STRICT_APPLIED.log"
echo "EVIDENCE_ROOT=$EVIDENCE"
