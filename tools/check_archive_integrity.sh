#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OLD_HOME="/home"
OLD_USER="neo"
OLD_ROOT="${OLD_HOME}/${OLD_USER}/racecar_simulator"

echo "==== 0. root ===="
echo "$ROOT"

FAIL=0

echo
echo "==== 1. required packages ===="
for d in \
  "$ROOT/src/control_msgs" \
  "$ROOT/src/racecar_simulator" \
  "$ROOT/src/race_driver"
do
  echo
  echo "---- $d ----"
  if [ -d "$d" ]; then
    echo "PASS: exists"
  else
    echo "FAIL: missing"
    FAIL=1
  fi
done

echo
echo "==== 2. race_driver versions ===="
VERSIONS=(
  current_main
  v3_final_before_v4
  v4_5_6_base_20260502_031944
  v5_safe_fast_031_65
  best_boundary_mpc_92
  best_boundary_mpc_897
  best_boundary_mpc_aggressive_90
  verified_boundary_mpc_aggressive_90
  best_boundary_mpc_892_delta075
  candidate_steer_rate_010
  before_hysteresis_20260502_104926
  hysteresis_slow_20260502_105510
)

for v in "${VERSIONS[@]}"; do
  p="$ROOT/archive/race_driver_versions/$v"
  echo
  echo "---- $v ----"
  if [ -f "$p/package.xml" ] && [ -f "$p/CMakeLists.txt" ]; then
    echo "PASS: package archive exists"
  else
    echo "FAIL: incomplete or missing package archive"
    FAIL=1
  fi
done

echo
echo "==== 3. simulator immutability policy ===="
echo "PASS: src/racecar_simulator is treated as immutable competition simulator code"
echo "INFO: tools must not patch src/racecar_simulator"

echo
echo "==== 4. old absolute path scan outside simulator ===="
HITS=$(find "$ROOT" \
  -path "$ROOT/.git" -prune -o \
  -path "$ROOT/src/racecar_simulator" -prune -o \
  -type f \
  ! -name "SHA256SUMS.txt" \
  -print0 \
  | xargs -0 grep -In "${OLD_ROOT}" 2>/dev/null || true)

if [ -n "$HITS" ]; then
  echo "$HITS"
  echo "INFO: old path remains outside simulator, usually inside archived race_driver configs."
  echo "INFO: run tools/patch_clone_paths.sh after clone if active race_driver needs local path patching."
else
  echo "PASS: no old absolute project root path outside simulator"
fi

echo
echo "==== 5. executable tools ===="
for t in \
  "$ROOT/tools/list_race_driver_versions.sh" \
  "$ROOT/tools/activate_race_driver_version.sh" \
  "$ROOT/tools/check_archive_integrity.sh" \
  "$ROOT/tools/patch_clone_paths.sh"
do
  if [ -x "$t" ]; then
    echo "PASS: executable -> $t"
  else
    echo "FAIL: not executable -> $t"
    FAIL=1
  fi
done

echo
echo "==== 6. final ===="
if [ "$FAIL" -eq 0 ]; then
  echo "PASS: archive integrity basic check"
else
  echo "FAIL: archive integrity basic check"
  exit 1
fi
