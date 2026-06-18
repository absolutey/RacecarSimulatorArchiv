#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ACTIVE="$ROOT/src/race_driver"

OLD_HOME="/home"
OLD_USER="neo"
OLD_ROOT="${OLD_HOME}/${OLD_USER}/racecar_simulator"

echo "==== patch clone paths: race_driver only ===="
echo "ROOT=$ROOT"
echo "ACTIVE=$ACTIVE"
echo "NOTE: src/racecar_simulator is intentionally not patched."

if [ ! -d "$ACTIVE" ]; then
  echo "FAIL: missing active race_driver: $ACTIVE"
  exit 1
fi

grep -RIl "__RACECAR_ARCHIVE_ROOT__\|${OLD_ROOT}" "$ACTIVE" \
  --include="*.yaml" \
  --include="*.py" \
  --include="*.cpp" \
  --include="*.hpp" \
  --include="*.h" \
  --include="*.txt" \
  2>/dev/null | while read -r f; do
    echo "PATCH: $f"
    sed -i \
      -e "s|__RACECAR_ARCHIVE_ROOT__|$ROOT|g" \
      -e "s|${OLD_ROOT}|$ROOT|g" \
      "$f"
  done

echo "PASS: race_driver paths patched only"
