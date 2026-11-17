#!/usr/bin/env bash
# Fail the isolated workspace BEFORE colcon if Solo's offline prerequisites
# are out of sync with actual ROS source methods. Never touch ~/f1tenth_racing.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
EVIDENCE_ROOT="${EVIDENCE_ROOT:-$(dirname "$ROOT")/EVIDENCE_RB_QOS_PREFLIGHT_REPAIR}"
export EVIDENCE_ROOT
cd "$ROOT"
printf '===== STEP 0: EXACT SOURCE + ALL SOLO OFFLINE PREFLIGHT =====\n'
bash RUN_CAPTURE_PREFLIGHT_ONLY.sh
printf '===== STEP 1: ROS BUILD + SELECTED + FULL REGRESSION =====\n'
bash RUN_RB_STATE_QOS_CLOSURE_VERIFY.sh
printf 'RB_QOS_PREFLIGHT_REPAIR_ROS_REGRESSION_PASS; SOLO_TIMING_UNQUALIFIED\n'
