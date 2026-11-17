#!/usr/bin/env python3
"""Actual source-provenance check; detects stale or swapped native image."""
import json,sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent/'src/environment/f1tenth_racing_sim'))
from f1tenth_racing_sim.safety.native_stop_proof import check_experimental_parity_receipt
lib=Path(sys.argv[1]).resolve(strict=True)
receipt=Path(str(lib)+'.receipt.json')
assert check_experimental_parity_receipt(str(lib),str(receipt))
data=json.loads(receipt.read_text())
with tempfile.TemporaryDirectory() as tmp:
  p=Path(tmp)/'invalid.json';bad=dict(data,library_sha256='0'*64)
  p.write_text(json.dumps(bad))
  try:check_experimental_parity_receipt(lib,p)
  except ValueError as e:assert 'NATIVE_PARITY_LIBRARY_HASH_MISMATCH' in str(e)
  else:raise AssertionError('ALTERED_LIBRARY_SHA_ACCEPTED')
  bad=dict(data,source_hashes=[dict(data['source_hashes'][0],sha256='0'*64)]+data['source_hashes'][1:])
  p.write_text(json.dumps(bad))
  try:check_experimental_parity_receipt(lib,p)
  except ValueError as e:assert 'NATIVE_PARITY_SOURCE_HASH_MISMATCH' in str(e)
  else:raise AssertionError('ALTERED_SOURCE_SHA_ACCEPTED')
  bad=dict(data,stop_cases=0)
  p.write_text(json.dumps(bad))
  try:check_experimental_parity_receipt(lib,p)
  except ValueError as e:assert 'NATIVE_PARITY_RECEIPT_NOT_QUALIFIED' in str(e)
  else:raise AssertionError('NO_TEST_RECEIPT_ACCEPTED')
print('NATIVE_PARITY_RECEIPT_VALIDATION_AND_FAULT_INJECTION_PASS; NO_REALTIME_CERTIFICATE')
