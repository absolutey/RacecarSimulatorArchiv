#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VERSION="${1:-}"

if [ -z "$VERSION" ]; then
  echo "Usage: $0 <version_name>"
  echo
  "$ROOT/tools/list_race_driver_versions.sh"
  exit 1
fi

SRC_VERSION="$ROOT/archive/race_driver_versions/$VERSION"
ACTIVE="$ROOT/src/race_driver"

OLD_HOME="/home"
OLD_USER="neo"
OLD_ROOT="${OLD_HOME}/${OLD_USER}/racecar_simulator"

echo "==== activate race_driver version ===="
echo "ROOT=$ROOT"
echo "VERSION=$VERSION"
echo "SRC_VERSION=$SRC_VERSION"
echo "ACTIVE=$ACTIVE"
echo "NOTE: only src/race_driver is replaced. src/racecar_simulator is never modified."

if [ ! -d "$SRC_VERSION" ]; then
  echo "FAIL: version not found: $VERSION"
  exit 1
fi

if [ ! -f "$SRC_VERSION/package.xml" ] || [ ! -f "$SRC_VERSION/CMakeLists.txt" ]; then
  echo "FAIL: selected version is not a complete race_driver package"
  exit 1
fi

TS="$(date +%Y%m%d_%H%M%S)"

echo
echo "==== backup current active race_driver ===="
if [ -d "$ACTIVE" ]; then
  BACKUP="$ROOT/src/race_driver_BACKUP_before_${VERSION}_${TS}"
  cp -a "$ACTIVE" "$BACKUP"
  echo "PASS: backup -> $BACKUP"
fi

echo
echo "==== copy selected version to src/race_driver ===="
rm -rf "$ACTIVE"
cp -a "$SRC_VERSION" "$ACTIVE"
find "$ACTIVE" -name COLCON_IGNORE -delete
echo "PASS: activated $VERSION"

echo
echo "==== patch active race_driver paths to this clone root ===="
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

echo "PASS: active race_driver path patch complete"

echo
echo "==== active version check ===="
ls -1 "$ACTIVE/package.xml" "$ACTIVE/config/race_driver.yaml" "$ACTIVE/CMakeLists.txt"
