#!/usr/bin/env python3
"""Integrity and static integration only. Does NOT qualify ROS/H2H safety."""
from pathlib import Path
import hashlib,json,ast
ROOT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
raw=json.loads((ROOT/'RELEASE_HASHES.json').read_text())
actual={p.relative_to(ROOT).as_posix():p for p in ROOT.rglob('*') if p.is_file() and
    not p.is_symlink() and p.relative_to(ROOT).parts[0] not in ('build','install','log') and
    not any(x in ('__pycache__','.pytest_cache') for x in p.relative_to(ROOT).parts) and p.suffix!='.pyc'}
actual.pop('RELEASE_HASHES.json',None)
if set(actual)!=set(raw):
    raise SystemExit('UNEXPECTED_OR_MISSING_FILES:'+str(sorted(set(actual)^set(raw))[:25]))
for name,expected in sorted(raw.items()):
    if sha(actual[name])!=expected:
        raise SystemExit('RELEASE_HASH_MISMATCH:'+name)
source=json.loads((ROOT/'SOURCE_MANIFEST.json').read_text())
src={k:v for k,v in actual.items() if k.startswith('src/')}
if source['count']!=len(source['files']) or set(e['path'] for e in source['files'])!=set(src):
    raise SystemExit('SOURCE_MANIFEST_MISMATCH')
for entry in source['files']:
    if sha(src[entry['path']])!=entry['candidate_sha256']:
        raise SystemExit('SOURCE_HASH_MISMATCH:'+entry['path'])
original_mod=sorted(e['path'] for e in source['files'] if e['original_sha256']!=e['candidate_sha256'])
if original_mod!=sorted(source['modified_files']):
    raise SystemExit('MODIFIED_PATHS_MISMATCH')
for name,pattern in (
 ('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/actuator_guard_node.py','witness.collision_free(z,z1,0.0,0.025)'),
 ('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/actuator_guard_node.py','if elapsed_ns>15000000:'),
 ('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/guard_ahead.py','MAX_EXTRA_NS = 7_000_000'),
 ('src/platform/f1tenth_racing_bringup/launch/race_foundation.launch.py','H2H_GUARD_RED_PROOF_REQUIRED'),
 ('src/platform/f1tenth_racing_bringup/launch/race_foundation.launch.py','independent_guard_watchdog'),
 ('src/environment/f1tenth_racing_sim/f1tenth_racing_sim/simulator.py','EGO_EXTERNAL_WATCHDOG_MANDATORY'),
 ('src/verification/qualification_gates/applied_lineage.py','INDEPENDENT_WATCHDOG_CERTIFIED_APPLIED_UNPROVEN'),
):
    if pattern not in src[name].read_text():raise SystemExit('STATIC_INTEGRATION_MARKER_MISSING:'+name)
for name,p in actual.items():
    if name.endswith('.py'):
        ast.parse(p.read_text(),filename=name)
print('INTEGRATION_SOURCE_HASH_PASS',len(src),'source files')
print('INTEGRATION_RELEASE_HASH_PASS',len(raw),'files')
print('STATIC_GUARD_RED_AND_BUDGET_MARKERS_PRESENT_NO_RUNTIME_TIMING_CLAIM')
for name,pattern in (
 ('src/platform/f1tenth_racing_reference/f1tenth_racing_reference/abort_recovery_gate.py','PARKED_NOMINAL_RED_CLEAR_PRECONDITION_ONLY'),
 ('src/platform/f1tenth_racing_reference/f1tenth_racing_reference/node.py','H2H_ABORT_RECOVERY_STATIONARY_REJOIN'),
 ('src/driving/algorithms/f1tenth_racing_overtaking/f1tenth_racing_overtaking/successor_core.py','def acknowledge_abort_recovery'),
 ('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/actuator_guard_node.py','H2H_ABORT_REFERENCE_WITHDRAWN'),
):
    if pattern not in src[name].read_text():raise SystemExit('V5_RECOVERY_CLOSURE_MARKER_MISSING:'+name)
print('V5_H2H_REALTIME_NOT_QUALIFIED;V5_UBUNTU_BASELINE_405_PASS_FROM_USER_LOG')

for name,pattern in (
 ('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/node.py','LatestOnlyControlWorker('),
 ('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/node.py','source_superseded'),
 ('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/guard_ahead.py','vars(initial).values()'),
 ('src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/control_worker.py','class LatestOnlyControlWorker'),
):
 if pattern not in src[name].read_text():
  raise SystemExit('V6_CONTROL_WORKER_STATIC_MARKER_MISSING:'+name)
print('V6_RB_WORKER_AND_NONFINITE_GUARDAHEAD_STATIC_PASS')
print('V5_UBUNTU_405_PASS;V6_UBUNTU_433_PASS_FROM_USER_LOG;NO_REALTIME_OR_H2H_QUALIFICATION')

# V7 source identity and provisional evidence policy, NO safety authorization.
import hashlib as _sha
_native='src/platform/f1tenth_racing_native_proof/src/native_stop_proof.cpp'
_auth='src/environment/f1tenth_racing_sim/f1tenth_racing_sim/safety/native_stop_proof.cpp'
if _sha.sha256(src[_native].read_bytes()).digest()!=_sha.sha256(src[_auth].read_bytes()).digest():
 raise SystemExit('V7_NATIVE_SOURCE_DRIFT_DENIED')
cmake=src['src/platform/f1tenth_racing_native_proof/CMakeLists.txt'].read_text()
for key in ('NATIVE_PROOF_CPP_SOURCE_DRIFT_BLOCKED','add_library(f1tenth_native_stop SHARED','install(TARGETS f1tenth_native_stop'):
 if key not in cmake:raise SystemExit('V7_NATIVE_INSTALL_WIRING_MISSING:'+key)
gate=src['src/verification/qualification_gates/gate_model.py'].read_text()
for key in ('RAB_GATE_EVIDENCE_V2','RAB_G0G9_PROVISIONAL_V2','consumed_paths','_validate_gate_evidence'):
 if key not in gate:raise SystemExit('V7_EVIDENCE_POLICY_WIRING_MISSING:'+key)
print('V7_NATIVE_CXX_INSTALL_AND_GATE_BINDING_STATIC_PASS')
print('FIRSTPASS_UBUNTU_492_PASS_FROM_USER_LOG;V7_ORIGINAL_NATIVE_ENV_REPAIRED;H2H_AND_RT_UNQUALIFIED')


# FIRSTPASS: no 35-gap closure claim.
from pathlib import Path as _P
_re='src/driving/algorithms/f1tenth_racing_overtaking/f1tenth_racing_overtaking/'
for _name in ('runtime_environment.py','state_input_contract.py'):
 if _re+_name not in src:raise SystemExit('FIRSTPASS_INPUT_CONTRACT_MISSING:'+_name)
for _name in ('node.py','successor_node.py'):
 if 'load_verified_runtime(' not in src[_re+_name].read_text():
  raise SystemExit('FIRSTPASS_DUAL_MAP_NOT_BOUND:'+_name)
 if 'admit_ego_state(' not in src[_re+_name].read_text():
  raise SystemExit('FIRSTPASS_EGO_ADMISSION_NOT_BOUND:'+_name)
print('FIRSTPASS_STATIC_DUAL_MAP_EGO_GATE_PASS_NO_RUNTIME_AUTHORITY_CLAIM')

# SECOND PASS: explicit check is source only, never performance/authority.
_required_secondpass = (
 'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/primary_worker.py',
 'src/driving/algorithms/f1tenth_racing_control/test/test_primary_worker_secondpass.py',
 'src/driving/algorithms/f1tenth_racing_control/test/test_red_confidence_secondpass.py',
 'src/driving/algorithms/f1tenth_racing_raceline/test/test_spielberg_source_secondpass.py',
 'src/platform/f1tenth_racing_reference/f1tenth_racing_reference/node.py',
)
for _file in _required_secondpass:
 if _file not in src:raise SystemExit('SECOND_PASS_MISSING:'+_file)
assert 'H2H_ABORT_COMPLETE_NOT_AUTHORIZED_FOR_NOMINAL' in src[_required_secondpass[-1]].read_text()
print('SECOND_PASS_SOURCE_FILE_CONTRACT_PASS_NO_PRODUCT_COMPLETION_CLAIM')
