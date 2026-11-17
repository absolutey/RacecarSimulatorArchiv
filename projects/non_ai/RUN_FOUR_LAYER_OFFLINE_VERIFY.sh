#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$ROOT"
python3 -B VERIFY_RELEASE.py
python3 -B FOUR_LAYER_CODE_CLOSURE_AUDIT.py
export PYTHONPATH="$ROOT/src/driving/algorithms/f1tenth_racing_control:$ROOT/src/driving/algorithms/f1tenth_racing_overtaking:$ROOT/src/driving/algorithms/f1tenth_racing_raceline:$ROOT/src/platform/f1tenth_racing_geometry:$ROOT/src/platform/f1tenth_racing_reference:$ROOT/src/environment/f1tenth_racing_sim:$ROOT/src/verification${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q \
 src/driving/algorithms/f1tenth_racing_overtaking/test/test_g8_real_retry_gate_firstpass.py \
 src/driving/algorithms/f1tenth_racing_overtaking/test/test_post_abort_red_epoch_firstpass.py \
 src/driving/algorithms/f1tenth_racing_raceline/test/test_spielberg_r_a3_firstpass.py \
 src/driving/algorithms/f1tenth_racing_control/test/test_process_primary_channel_fourlayer.py \
 src/platform/f1tenth_racing_native_proof/test/test_native_guard_bounds_fourlayer.py \
 src/platform/f1tenth_racing_native_proof/test/test_native_guard_humble_api_secondpass.py \
 src/platform/f1tenth_racing_native_proof/test/test_native_guard_identity_thirdpass.py
printf '%s\n' 'FOUR_LAYER_OFFLINE_SOURCE_GATE_PASS;ROS_FULL_REGRESSION_REQUIRED;NO_SIMULATION'
