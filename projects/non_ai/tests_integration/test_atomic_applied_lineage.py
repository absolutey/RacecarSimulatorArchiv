import sys
import json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/verification/qualification_gates'))
from applied_lineage import AppliedLineageAudit

def receipt(*,sequence=4,mode='CERTIFIED',deadman=False,epoch='testepoch'):
    return json.dumps({'deadman':deadman,'command_stamp_ns':1000,
        'guard_intent_identity':{'run_epoch':epoch,'intent_sequence':sequence if mode=='CERTIFIED' else None,
        'command_stamp_ns':1000,'source_state_sequence':10,'source_id':'R_B',
        'authority_mode':mode,'proof_verified_by_guard':mode=='CERTIFIED'}})

def test_repeated_physics_ticks_of_one_certified_intent():
    a=AppliedLineageAudit()
    for _ in range(3):a.observe(receipt())
    assert a.verdict()['passed']
    assert a.verdict()['certified_ticks']==3

def test_missing_identity_fails_without_optimistic_join():
    a=AppliedLineageAudit()
    a.observe(json.dumps({'deadman':False,'command_stamp_ns':1000,'guard_intent_identity':None}))
    assert not a.verdict()['passed']
    assert a.unmatched==1

def test_brake_is_not_miscounted_as_certified_motion():
    a=AppliedLineageAudit();a.observe(receipt(mode='MINIMUM_RISK_BRAKE'))
    assert a.minimum_risk_brake==1
    assert not a.verdict()['passed']

def test_regressing_intent_sequence_fails():
    a=AppliedLineageAudit();a.observe(receipt(sequence=8));a.observe(receipt(sequence=6))
    assert a.unmatched==1

def test_epoch_change_fails():
    a=AppliedLineageAudit();a.observe(receipt());a.observe(receipt(epoch='other'))
    assert not a.verdict()['passed']

def test_stale_deadman_receipt_not_false_positive():
    a=AppliedLineageAudit();a.observe(json.dumps({'deadman':True}))
    assert a.unmatched==0 and not a.verdict()['passed']
