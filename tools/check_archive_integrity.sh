#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OLD_HOME="/home"
OLD_USER="neo"
OLD_ROOT="${OLD_HOME}/${OLD_USER}/racecar_simulator"
FAIL=0

echo "==== 0. root ===="
echo "$ROOT"

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
while IFS=$'\t' read -r version status kind source_meaning restore_target note; do
  [ "$version" = "version" ] && continue
  [ "$restore_target" != "src/race_driver" ] && continue

  d="$ROOT/archive/race_driver_versions/$version"
  echo
  echo "---- $version ----"
  if [ -d "$d" ] && [ -f "$d/package.xml" ] && [ -f "$d/CMakeLists.txt" ]; then
    echo "PASS: package archive exists"
  else
    echo "FAIL: missing package archive"
    FAIL=1
  fi
done < "$ROOT/VERSION_MANIFEST.tsv"

echo
echo "==== 3. old absolute path scan ===="
HITS=$(grep -RIn "$OLD_ROOT" "$ROOT" \
  --exclude-dir=.git \
  --exclude="SHA256SUMS.txt" \
  2>/dev/null || true)

if [ -z "$HITS" ]; then
  echo "PASS: no old absolute project root path"
else
  echo "$HITS" | head -200
  echo "FAIL: old absolute path remains"
  FAIL=1
fi

echo
echo "==== 4. placeholder count ===="
grep -RIn "__RACECAR_ARCHIVE_ROOT__" "$ROOT" \
  --exclude-dir=.git \
  --exclude="SHA256SUMS.txt" \
  2>/dev/null | wc -l

echo
echo "==== 5. large files over 50MB ===="
find "$ROOT" -type f -size +50M -printf "%s bytes  %p\n" 2>/dev/null | sort -nr || true

echo
echo "==== 6. executable tools ===="
for f in \
  "$ROOT/tools/list_race_driver_versions.sh" \
  "$ROOT/tools/activate_race_driver_version.sh" \
  "$ROOT/tools/check_archive_integrity.sh" \
  "$ROOT/tools/patch_clone_paths.sh"
do
  if [ -x "$f" ]; then
    echo "PASS: executable -> $f"
  else
    echo "FAIL: not executable -> $f"
    FAIL=1
  fi
done

echo
echo "==== 7. final ===="
if [ "$FAIL" -eq 0 ]; then
  echo "PASS: archive integrity basic check"
else
  echo "FAIL: archive integrity has problems"
fi

exit "$FAIL"
