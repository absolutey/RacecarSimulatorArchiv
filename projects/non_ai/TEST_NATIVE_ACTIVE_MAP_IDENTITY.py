from pathlib import Path
import json,sys
here=Path(__file__).resolve().parent
sys.path.insert(0,str(here/'src/environment/f1tenth_racing_sim'))
from f1tenth_racing_sim.safety.native_stop_proof import check_experimental_parity_receipt
lib=Path(sys.argv[1]).resolve(strict=True)
receipt=Path(str(lib)+'.receipt.json')
rec=json.loads(receipt.read_text())
map_entry=next(e for e in rec['source_hashes'] if e['path'].endswith('iccas2025_map.pgm'))
assert check_experimental_parity_receipt(lib,receipt,expected_map_sha256=map_entry['sha256'],active_map_file=map_entry['path'])
try:
 check_experimental_parity_receipt(lib,receipt,expected_map_sha256='0'*64,active_map_file=map_entry['path'])
except ValueError as exc:
 assert 'NATIVE_ACTIVE_MAP_HASH_MISMATCH' in str(exc)
else:raise AssertionError('BAD_ACTIVE_MAP_HASH_ACCEPTED')
other_map=here/'src/environment/f1tenth_racing_sim/maps/Spielberg/Spielberg_map.pgm'
if other_map.exists():
 import hashlib
 try:
  check_experimental_parity_receipt(lib,receipt,expected_map_sha256=hashlib.sha256(other_map.read_bytes()).hexdigest(),active_map_file=other_map)
 except ValueError as exc:
  assert 'NATIVE_PARITY_NOT_RUN_FOR_ACTIVE_MAP' in str(exc)
 else:raise AssertionError('UNQUALIFIED_MAP_REUSED_PARITY_RECEIPT')
print('NATIVE_ACTIVE_MAP_IDENTITY_GUARD_PASS')
