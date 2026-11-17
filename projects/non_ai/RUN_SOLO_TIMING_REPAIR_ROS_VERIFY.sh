#!/usr/bin/env bash
# Full regression first; no ROS process starts if a test, hash, or safety gate fails.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
EVIDENCE_ROOT="${EVIDENCE_ROOT:-$(dirname "$ROOT")/EVIDENCE_SOLO_TIMING_REPAIR}"
export EVIDENCE_ROOT
cd "$ROOT"
bash RUN_FINAL_INTEGRATED_ROS_VERIFY.sh
bash RUN_CAPTURE_PREFLIGHT_ONLY.sh
printf 'SOLO_TIMING_REPAIR_CODE_GATE_PASS_NO_REALTIME_OR_LAP_QUALIFICATION\n'
# Real Solo capture is a separate, explicit operator decision.
