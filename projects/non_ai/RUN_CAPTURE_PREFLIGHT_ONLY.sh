#!/usr/bin/env bash
# ROS-free preflight, including code-source SHA and all Solo startup gates.
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
cd "$WS"
python3 -B VERIFY_RELEASE.py
python3 -B -m pytest -q src/platform/f1tenth_racing_bringup/test/test_capture_offline_preflight.py
printf 'SOLO_OFFLINE_PREFLIGHT_PASS_NO_ROS_QUALIFICATION\n'
