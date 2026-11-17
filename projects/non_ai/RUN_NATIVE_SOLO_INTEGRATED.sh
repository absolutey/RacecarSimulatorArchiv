#!/usr/bin/env bash
# Explicit simulator-only end-to-end native R-B + independent Guard candidate.
# The existing launcher compiles the model, performs source/map parity, and
# fails before START if any receipt/hash or full-stop validation disagrees.
set -euo pipefail
cd "$(dirname "$0")"
python3 -B VERIFY_RELEASE.py
F1TENTH_EXPERIMENT_NATIVE_SOLO=1 exec bash RUN_INTEGRATED_SOLO_CAPTURE.sh
