#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MANIFEST="$ROOT/VERSION_MANIFEST.tsv"

echo "==== Race Driver version manifest ===="
column -t -s $'\t' "$MANIFEST" 2>/dev/null || cat "$MANIFEST"

echo
echo "==== Available full package versions ===="
find "$ROOT/archive/race_driver_versions" -maxdepth 1 -mindepth 1 -type d -printf "%f\n" 2>/dev/null | sort
