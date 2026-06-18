#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OLD_HOME="/home"
OLD_USER="neo"
OLD_ROOT="${OLD_HOME}/${OLD_USER}/racecar_simulator"
VERSION="${1:-}"

if [ -z "$VERSION" ]; then
  echo "Usage: $0 <version_name>"
  echo
  "$ROOT/tools/list_race_driver_versions.sh"
  exit 1
fi

SRC_VERSION="$ROOT/archive/race_driver_versions/$VERSION"
ACTIVE="$ROOT/src/race_driver"
BACKUP="$ROOT/src/race_driver_BACKUP_before_${VERSION}_$(date +%Y%m%d_%H%M%S)"

echo "==== activate race_driver version ===="
echo "ROOT=$ROOT"
echo "VERSION=$VERSION"
echo "SRC_VERSION=$SRC_VERSION"
echo "ACTIVE=$ACTIVE"

if [ ! -d "$SRC_VERSION" ]; then
  echo "FAIL: version not found -> $SRC_VERSION"
  exit 1
fi

if [ ! -f "$SRC_VERSION/package.xml" ] || [ ! -f "$SRC_VERSION/CMakeLists.txt" ]; then
  echo "FAIL: selected version is not a race_driver ROS2 package"
  exit 1
fi

if [ -d "$ACTIVE" ]; then
  echo
  echo "==== backup current active race_driver ===="
  mv "$ACTIVE" "$BACKUP"
  echo "PASS: backup -> $BACKUP"
fi

echo
echo "==== copy selected version to src/race_driver ===="
mkdir -p "$(dirname "$ACTIVE")"
rsync -a "$SRC_VERSION/" "$ACTIVE/"
echo "PASS: activated $VERSION"

echo
echo "==== patch active race_driver paths to this clone root ===="
grep -RIl "__RACECAR_ARCHIVE_ROOT__\|${OLD_ROOT}" "$ACTIVE" \
  --exclude-dir=.git \
  --exclude="SHA256SUMS.txt" \
  2>/dev/null | while read -r f; do
    echo "PATCH: $f"
    sed -i \
      -e "s|__RACECAR_ARCHIVE_ROOT__|$ROOT|g" \
      -e "s|${OLD_ROOT}|$ROOT|g" \
      "$f"
  done

echo "PASS: path patch complete"

echo
echo "==== active version check ===="
find "$ACTIVE" -maxdepth 3 -type f \( -name "package.xml" -o -name "CMakeLists.txt" -o -name "race_driver.yaml" \) -print
