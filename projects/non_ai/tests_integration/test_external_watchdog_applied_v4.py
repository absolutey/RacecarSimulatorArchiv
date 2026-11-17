"""V4 strict Applied lineage: independent veto must be live for every certified tick."""
import sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src/verification/qualification_gates'))
from applied_lineage import AppliedLineageAudit


def good(*,live=True,reason='INDEPENDENT_WATCHDOG_LIVE',mode='CERTIFIED',deadman=False):
    return {'deadman':deadman,'command_stamp_ns':1000,
        'independent_watchdog_live':live,'independent_watchdog_reason':reason,
        'guard_intent_identity':{'run_epoch':'epoch','intent_sequence':4 if mode=='CERTIFIED' else None,
        'command_stamp_ns':1000,'source_state_sequence':10,'source_id':'R_B',
        'authority_mode':mode,'proof_verified_by_guard':mode=='CERTIFIED'}}


def test_qualified_guard_and_external_watchdog_receipt():
    a=AppliedLineageAudit(require_external_watchdog=True)
    a.observe(good());assert a.verdict()['passed']
    assert a.verdict()['external_watchdog_checked_ticks']==1


def test_external_watchdog_missing_rejects_live_qualification():
    a=AppliedLineageAudit(require_external_watchdog=True)
    m=good();m.pop('independent_watchdog_live');a.observe(m)
    assert not a.verdict()['passed'] and a.verdict()['unmatched_applied']==1


def test_external_watchdog_false_rejects_live_qualification():
    a=AppliedLineageAudit(require_external_watchdog=True)
    a.observe(good(live=False))
    assert not a.verdict()['passed']


def test_external_watchdog_wrong_reason_rejects_live_qualification():
    a=AppliedLineageAudit(require_external_watchdog=True)
    a.observe(good(reason='STALE'))
    assert not a.verdict()['passed']


def test_minimum_risk_brake_still_permitted_without_heartbeat():
    a=AppliedLineageAudit(require_external_watchdog=True)
    a.observe(good(live=False,mode='MINIMUM_RISK_BRAKE'))
    assert a.verdict()['minimum_risk_brake_ticks']==1
    assert not a.verdict()['passed']  # no certified motion


def test_deadman_does_not_count_as_certified():
    a=AppliedLineageAudit(require_external_watchdog=True)
    a.observe({'deadman':True})
    assert not a.verdict()['passed'] and a.verdict()['certified_ticks']==0


def test_real_qualifiers_require_external_watchdog():
    for rel in ('src/verification/solo_champion/qualify_solo.py',
                'src/verification/h2h_champion/qualify_h2h.py'):
        text=(root/rel).read_text()
        assert 'AppliedLineageAudit(require_external_watchdog=True)' in text
