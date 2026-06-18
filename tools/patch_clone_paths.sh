#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OLD_HOME="/home"
OLD_USER="neo"
OLD_ROOT="${OLD_HOME}/${OLD_USER}/racecar_simulator"

echo "==== patch clone paths ===="
echo "ROOT=$ROOT"

grep -RIl "__RACECAR_ARCHIVE_ROOT__\|${OLD_ROOT}" "$ROOT" \
  --exclude-dir=.git \
  --exclude="SHA256SUMS.txt" \
  2>/dev/null | while read -r f; do
    echo "PATCH: $f"
    sed -i \
      -e "s|__RACECAR_ARCHIVE_ROOT__|$ROOT|g" \
      -e "s|${OLD_ROOT}|$ROOT|g" \
      "$f"
  done

echo "PASS: clone paths patched to $ROOT"
