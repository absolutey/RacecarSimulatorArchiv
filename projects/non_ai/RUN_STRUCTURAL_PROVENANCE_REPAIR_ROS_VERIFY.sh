#!/usr/bin/env bash
# Isolated workspace: full ROS build, selected packages, full 535+ suite.
set -eo pipefail
cd "$(dirname "$0")"
EVIDENCE_ROOT="${EVIDENCE_ROOT:-$PWD/../EVIDENCE_STRUCTURAL_PROVENANCE_REPAIR}" \
  bash RUN_STRUCTURAL_IMPLEMENTATION_ROS_VERIFY.sh
