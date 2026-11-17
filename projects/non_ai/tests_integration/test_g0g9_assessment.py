import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/verification/qualification_gates'))
from gate_model import expected_gate_keys, inspect_receipts

SHA='f'*64

def record(gate,scope,hashv,metrics):
    return {'gate':gate,'scope':scope,'schema':'RAB_G0G9_PROVISIONAL_V2',
            'status':'PASS','run_id':'run77','source_tree_sha256':SHA,
            'evidence':[{'path':'proof.txt','sha256':hashv}], 'metrics':metrics}

def all_receipts(tmp_path):
    metrics={
        'G0':{'source_hash_verified':True,'release_hash_verified':True},
        'G1':{'packages_finished':16,'build_exit':0},
        'G2':{'tests':327,'errors':0,'failures':0,'skipped':0},
        'G3':{'unsafe_intents_rejected':1,'unsafe_intents_applied':0},
        'G4':{'samples':5,'deadline_violations':0,'end_to_end_max_ms':14.8},
        'G5':{'samples':5,'deadline_violations':0,'guardahead_max_ms':6.7},
        'G6':{'physical_laps':2,'wall_collisions':0,'vehicle_collisions':0},
        'G7':{'applied_intent_matched':3,'unmatched_applied':0,'reference_mismatch':0,'mpc_primary_verified':True},
        'G8':{'overtakes_completed':1,'abort_or_rejoin_verified':True,'wall_collisions':0,'vehicle_collisions':0},
        'G9':{'faults_injected':1,'unsafe_commands_applied':0},
    }
    records=[]
    for gate,scope in expected_gate_keys():
        payload={'schema':'RAB_GATE_EVIDENCE_V2','gate':gate,'scope':scope,
                 'run_id':'run77','source_tree_sha256':SHA,'metrics':metrics[gate]}
        name=f'{gate}_{scope}.json'
        artifact=tmp_path/name
        artifact.write_text(json.dumps(payload,sort_keys=True))
        entry=record(gate,scope,hashlib.sha256(artifact.read_bytes()).hexdigest(),metrics[gate])
        entry['evidence'][0]['path']=name
        records.append(entry)
    return records

def check(records,tmp):
    return inspect_receipts(records,tmp,source_tree_sha256=SHA,run_id='run77')

def test_all_evidence_requires_review_not_automatic_promotion(tmp_path):
    decision=check(all_receipts(tmp_path),tmp_path)
    assert decision['evidence_complete'] and not decision['product_promoted']

def test_missing_evidence_never_promotes(tmp_path):
    assert check([],tmp_path)['verdict'].startswith('DENIED')

def test_wrong_source_denied(tmp_path):
    records=all_receipts(tmp_path)
    records[0]['source_tree_sha256']='0'*64
    assert not check(records,tmp_path)['evidence_complete']

def test_deadline_failure_denied(tmp_path):
    records=all_receipts(tmp_path)
    next(r for r in records if r['gate']=='G4')['metrics']['end_to_end_max_ms']=15.3
    assert check(records,tmp_path)['decisions']['G4/GLOBAL']!='PROVISIONAL_EVIDENCE_VERIFIED'

def test_hash_tamper_denied(tmp_path):
    records=all_receipts(tmp_path)
    (tmp_path/'G0_GLOBAL.json').write_text('modified')
    assert not check(records,tmp_path)['evidence_complete']

def test_duplicate_cannot_overwrite_gate(tmp_path):
    records=all_receipts(tmp_path)
    assert check(records+[records[0]],tmp_path)['errors']

def test_missing_second_map_denied(tmp_path):
    records=[r for r in all_receipts(tmp_path)
             if (r['gate'],r['scope'])!=('G8','spielberg')]
    assert check(records,tmp_path)['decisions']['G8/spielberg']=='MISSING'

def test_incomplete_metric_denied(tmp_path):
    records=all_receipts(tmp_path)
    next(r for r in records if r['gate']=='G2')['metrics']['failures']=1
    assert not check(records,tmp_path)['evidence_complete']

def test_path_traversal_denied(tmp_path):
    records=all_receipts(tmp_path)
    records[0]['evidence'][0]['path']='../proof.txt'
    assert not check(records,tmp_path)['evidence_complete']


def test_gate_evidence_cross_map_swap_denied(tmp_path):
    records=all_receipts(tmp_path)
    a=next(r for r in records if (r['gate'],r['scope'])==('G8','iccas2025'))
    b=next(r for r in records if (r['gate'],r['scope'])==('G8','spielberg'))
    a['evidence'],b['evidence']=b['evidence'],a['evidence']
    assert check(records,tmp_path)['decisions']['G8/iccas2025']!='PROVISIONAL_EVIDENCE_VERIFIED'


def test_gate_metrics_tampered_without_file_denied(tmp_path):
    records=all_receipts(tmp_path)
    next(r for r in records if r['gate']=='G4')['metrics']['end_to_end_max_ms']=13.1
    assert not check(records,tmp_path)['evidence_complete']


def test_reused_gate_file_denied(tmp_path):
    records=all_receipts(tmp_path)
    records[1]['evidence']=records[0]['evidence']
    assert not check(records,tmp_path)['evidence_complete']


def test_legacy_unbound_receipt_denied(tmp_path):
    records=all_receipts(tmp_path)
    records[0]['schema']='RAB_G0G9_PROVISIONAL_V1'
    assert not check(records,tmp_path)['evidence_complete']


def test_negative_test_count_denied(tmp_path):
    records=all_receipts(tmp_path)
    next(r for r in records if r['gate']=='G2')['metrics']['errors']=-1
    assert not check(records,tmp_path)['evidence_complete']


def test_boolean_test_count_denied(tmp_path):
    records=all_receipts(tmp_path)
    next(r for r in records if r['gate']=='G2')['metrics']['skipped']=False
    assert not check(records,tmp_path)['evidence_complete']


def test_invalid_gate_evidence_json_denied(tmp_path):
    records=all_receipts(tmp_path)
    p=tmp_path/'G2_GLOBAL.json';p.write_text('broken json')
    records[2]['evidence'][0]['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
    assert not check(records,tmp_path)['evidence_complete']


def test_evidence_scope_in_file_mismatch_denied(tmp_path):
    records=all_receipts(tmp_path)
    p=tmp_path/'G8_spielberg.json'
    doc=json.loads(p.read_text());doc['scope']='iccas2025'
    p.write_text(json.dumps(doc))
    rec=next(r for r in records if (r['gate'],r['scope'])==('G8','spielberg'))
    rec['evidence'][0]['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
    assert not check(records,tmp_path)['evidence_complete']


def test_nan_deadline_denied(tmp_path):
    records=all_receipts(tmp_path)
    next(r for r in records if r['gate']=='G5')['metrics']['guardahead_max_ms']=float('nan')
    assert not check(records,tmp_path)['evidence_complete']


def test_symlink_artifact_denied_even_with_matching_hash(tmp_path):
    records=all_receipts(tmp_path)
    target=tmp_path/'G0_GLOBAL.json'
    alias=tmp_path/'alias.json'
    alias.symlink_to(target)
    records[0]['evidence'][0]['path']='alias.json'
    assert not check(records,tmp_path)['evidence_complete']
