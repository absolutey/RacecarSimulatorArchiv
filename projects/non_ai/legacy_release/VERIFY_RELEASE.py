#!/usr/bin/env python3
"""Immutable P0 source and build-input validation; not a racing certificate."""
import hashlib
import json
from pathlib import Path

root=Path(__file__).resolve().parent
raw=json.loads((root/'RELEASE_HASHES.json').read_text())
for relative, sha in raw.items():
    p=root/relative
    if p.is_symlink() or not p.is_file():
        raise SystemExit('MISSING_OR_SYMLINK:'+relative)
    if hashlib.sha256(p.read_bytes()).hexdigest()!=sha:
        raise SystemExit('HASH_MISMATCH:'+relative)
source=json.loads((root/'SOURCE_MANIFEST.json').read_text())
if len(source['files'])!=253:raise SystemExit('SOURCE_COUNT_NOT_253')
for entry in source['files']:
    p=root/entry['path']
    if not p.is_file() or p.is_symlink():raise SystemExit('SOURCE_MISSING:'+entry['path'])
    if hashlib.sha256(p.read_bytes()).hexdigest()!=entry['candidate_sha256']:
        raise SystemExit('SOURCE_HASH_MISMATCH:'+entry['path'])
modified=[e['path'] for e in source['files'] if e['candidate_sha256']!=e['original_sha256']]
if sorted(modified)!=sorted(source['modified_files']):
    raise SystemExit('SOURCE_MODIFICATION_MANIFEST_MISMATCH')
if len(set(e['path'] for e in source['files']))!=253:
    raise SystemExit('DUPLICATE_SOURCE_MANIFEST')
# No silent extra source files disguised by old manifest. Python/test tools outside src are
# separately checked by release hashes; src tree has no unmanaged modifications.
known={e['path'] for e in source['files']}
for p in (root/'src').rglob('*'):
    if (p.is_file() and not p.is_symlink() and
            not any(segment in ('.pytest_cache','__pycache__') for segment in p.parts) and
            p.suffix!='.pyc'):
        if p.relative_to(root).as_posix() not in raw:
            raise SystemExit('UNHASHED_SRC_FILE:'+str(p.relative_to(root)))
control=root/'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control'
g=(control/'actuator_guard_node.py').read_text()
ga=(control/'guard_ahead.py').read_text()
if 'H2H_STOP_ENVELOPE_DYNAMIC_OCCUPANCY_NOT_QUALIFIED' not in g:
    raise SystemExit('H2H_INTERLOCK_REMOVED')
if 'if elapsed_ns>15000000:' not in g:
    raise SystemExit('GUARD_DEADLINE_CHANGED')
if 'MAX_EXTRA_NS = 7_000_000' not in ga:
    raise SystemExit('GUARDAHEAD_BUDGET_CHANGED')
print('P0_RELEASE_HASH_PASS',len(raw),'files; managed',len(known),'modified',len(modified))
print('GUARD_15MS_AND_GUARDAHEAD_7MS_PRESERVED; H2H_INTERLOCK_PRESERVED')
print('ROS_EXECUTION_AND_PHYSICAL_LAP_NOT_QUALIFIED')
