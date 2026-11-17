#!/usr/bin/env bash
# Isolated: full build, selected tests, then all 17 packages; no ROS race start.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
EVIDENCE_ROOT="${EVIDENCE_ROOT:-$(dirname "$ROOT")/EVIDENCE_RB_STATE_QOS}"
export EVIDENCE_ROOT
cd "$ROOT"
# The integrated runner first builds all dependencies, then selects changed
# packages before rerunning the complete suite. It checks each return code.
bash RUN_FINAL_INTEGRATED_ROS_VERIFY.sh
bash RUN_CAPTURE_PREFLIGHT_ONLY.sh
printf 'R_B_QOS_FULL_CODE_GATE_PASS; SOLO_MOTION_AND_RT_NOT_YET_QUALIFIED\n'
