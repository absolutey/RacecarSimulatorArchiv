"""Fail-closed evidence review for provisional RAB G0-G9 gate definitions.

This is deliberately NOT the project's signed official promotion authority.
Every source/map gate requires matching concrete evidence; existing build
success never turns physical or latency gates into PASS.
"""
import hashlib
import json
import math
from pathlib import Path

GATE_SCOPES={
    'G0':('GLOBAL',),                 # deterministic source and release
    'G1':('GLOBAL',),                 # ROS package build
    'G2':('GLOBAL',),                 # full existing/new regression
    'G3':('GLOBAL',),                 # Guard rejection/proof contract
    'G4':('GLOBAL',),                 # 15ms Guard end-to-end + deadline
    'G5':('GLOBAL',),                 # 7ms GuardAhead end-to-end
    'G6':('iccas2025','spielberg'),   # map-specific physical solo laps
    'G7':('iccas2025','spielberg'),   # R-A/S0-R -> MPC -> Applied physical trace
    'G8':('iccas2025','spielberg'),   # H2H lifecycle and Red collision
    'G9':('GLOBAL',),                 # fail-closed process-fault exercises
}


def expected_gate_keys():
    return tuple((gate,scope) for gate,scopes in GATE_SCOPES.items() for scope in scopes)


def _integer(values, name, *, minimum=0):
    value = values[name]
    if type(value) is not int or value < minimum:
        raise ValueError('GATE_METRIC_INTEGER_REQUIRED:'+name)
    return value


def _elapsed_ms(values, name, *, max_ms):
    value=values[name]
    if type(value) not in (int,float) or not math.isfinite(value) or not (0 <= value <= max_ms):
        raise ValueError('GATE_METRIC_BOUND_INVALID:'+name)
    return value


def _true(values, name):
    return values[name] is True


def _metric_ok(gate, values):
    """Typed and nonnegative; boolean is never accepted as an integer count."""
    if type(values) is not dict:
        return False
    try:
        if gate=='G0':
            return _true(values,'source_hash_verified') and _true(values,'release_hash_verified')
        if gate=='G1':
            return _integer(values,'packages_finished',minimum=16)>=16 and _integer(values,'build_exit')==0
        if gate=='G2':
            return (_integer(values,'tests',minimum=327)>=327 and
                    all(_integer(values,k)==0 for k in ('errors','failures','skipped')))
        if gate=='G3':
            return _integer(values,'unsafe_intents_rejected',minimum=1)>0 and _integer(values,'unsafe_intents_applied')==0
        if gate=='G4':
            return (_integer(values,'samples',minimum=1)>0 and
                    _integer(values,'deadline_violations')==0 and
                    _elapsed_ms(values,'end_to_end_max_ms',max_ms=15.0)>=0)
        if gate=='G5':
            return (_integer(values,'samples',minimum=1)>0 and
                    _integer(values,'deadline_violations')==0 and
                    _elapsed_ms(values,'guardahead_max_ms',max_ms=7.0)>=0)
        if gate=='G6':
            return (_integer(values,'physical_laps',minimum=1)>0 and
                    _integer(values,'wall_collisions')==0 and
                    _integer(values,'vehicle_collisions')==0)
        if gate=='G7':
            return (_integer(values,'applied_intent_matched',minimum=1)>0 and
                    _integer(values,'unmatched_applied')==0 and
                    _integer(values,'reference_mismatch')==0 and
                    _true(values,'mpc_primary_verified'))
        if gate=='G8':
            # A clean single pass is NOT the user's H2H success contract.
            # Every map must exercise unexpected Red behavior, fail-closed
            # ABORT/REJOIN, a post-ABORT fresh prediction and a different
            # plan-identity retry. Signed external evidence remains required.
            return (_integer(values,'overtakes_completed',minimum=1)>0 and
                    _true(values,'abort_or_rejoin_verified') and
                    _integer(values,'abort_events',minimum=1)>0 and
                    _integer(values,'successful_retries',minimum=1)>0 and
                    _true(values,'fresh_red_after_abort_verified') and
                    _true(values,'retry_used_new_plan_identity') and
                    _integer(values,'wall_collisions')==0 and
                    _integer(values,'vehicle_collisions')==0)
        if gate=='G9':
            return _integer(values,'faults_injected',minimum=1)>0 and _integer(values,'unsafe_commands_applied')==0
        return False
    except (KeyError,TypeError,ValueError,OverflowError):
        return False


def _validate_gate_evidence(candidate, *, gate, scope, run_id, source_tree_sha256, metrics):
    """Cross-bind the evidence file to the exact gate, map scope and run.

    This provides tamper and cross-scope resistance only, not independent
    physical truth or formal authorization. Files must remain read-only JSON.
    """
    try:
        payload=json.loads(candidate.read_text())
        return (type(payload) is dict and
                payload.get('schema')=='RAB_GATE_EVIDENCE_V2' and
                payload.get('gate')==gate and payload.get('scope')==scope and
                payload.get('run_id')==run_id and
                payload.get('source_tree_sha256')==source_tree_sha256 and
                payload.get('metrics')==metrics and
                _metric_ok(gate,payload.get('metrics')))
    except (OSError,UnicodeError,ValueError,TypeError,OverflowError):
        return False


def inspect_receipts(receipts, evidence_root, *, source_tree_sha256, run_id):
    """Read-only, deterministic assessment; never signs or promotes authority."""
    root=Path(evidence_root).resolve(strict=True)
    expected=set(expected_gate_keys())
    records={}
    errors=[]
    for record in receipts:
        try:
            key=(record['gate'],record['scope'])
            if key not in expected or key in records:
                errors.append(f'INVALID_OR_DUPLICATE_GATE:{key}')
                continue
            records[key]=record
        except (TypeError,KeyError):
            errors.append('MALFORMED_GATE_RECORD')
    decisions={}
    consumed_paths=set()
    for key in expected_gate_keys():
        receipt=records.get(key)
        if receipt is None:
            decisions['/'.join(key)]='MISSING'
            continue
        reason='NO_VERIFIABLE_EVIDENCE'
        if (receipt.get('schema')=='RAB_G0G9_PROVISIONAL_V2' and
                receipt.get('status')=='PASS' and
                receipt.get('source_tree_sha256')==source_tree_sha256 and
                receipt.get('run_id')==run_id and
                isinstance(receipt.get('evidence'),list) and
                len(receipt['evidence'])>=1 and _metric_ok(key[0],receipt.get('metrics',{}))):
            verified=True
            own_paths=set()
            for item in receipt['evidence']:
                try:
                    if not isinstance(item,dict):
                        verified=False;break
                    rel=Path(item['path'])
                    if rel.is_absolute() or '..' in rel.parts or not rel.parts:
                        verified=False;break
                    if any((root.joinpath(*rel.parts[:n])).is_symlink()
                           for n in range(1,len(rel.parts)+1)):
                        verified=False;break
                    candidate=(root/rel).resolve(strict=True)
                    if (not candidate.is_relative_to(root) or not candidate.is_file() or
                            type(item['sha256']) is not str or
                            len(item['sha256'])!=64 or
                            not all(c in '0123456789abcdef' for c in item['sha256']) or
                            candidate in own_paths or candidate in consumed_paths):
                        verified=False;break
                    if hashlib.sha256(candidate.read_bytes()).hexdigest()!=item['sha256']:
                        verified=False;break
                    if not _validate_gate_evidence(candidate,gate=key[0],scope=key[1],
                            run_id=run_id,source_tree_sha256=source_tree_sha256,
                            metrics=receipt['metrics']):
                        verified=False;break
                    own_paths.add(candidate)
                except (KeyError,TypeError,ValueError,OSError,OverflowError):
                    verified=False;break
            if verified:
                consumed_paths.update(own_paths)
            reason='PROVISIONAL_EVIDENCE_VERIFIED' if verified else 'EVIDENCE_HASH_SCOPE_OR_METRICS_INVALID'
        decisions['/'.join(key)]=reason
    complete=(not errors and all(x=='PROVISIONAL_EVIDENCE_VERIFIED' for x in decisions.values()))
    return {'schema':'RAB_G0G9_ASSESSMENT_V2',
            'policy':'DRAFT_NOT_PRODUCT_PROMOTION',
            'source_tree_sha256':source_tree_sha256,
            'run_id':run_id,
            'decisions':decisions,'errors':errors,
            'evidence_complete':complete,
            'product_promoted':False,
            'verdict':'READY_FOR_AUTHORIZED_POLICY_REVIEW' if complete else 'DENIED_MISSING_OR_INVALID_EVIDENCE'}
