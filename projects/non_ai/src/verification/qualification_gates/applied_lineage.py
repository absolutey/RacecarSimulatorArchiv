"""Read-only applied-actuation lineage audit; never grants authority."""
import json

class AppliedLineageAudit:
    def __init__(self, *, require_external_watchdog=False):
        # Historical offline fixture tests need not contain a V4 field, but
        # every LIVE qualification must opt into strict independent-veto proof.
        self.require_external_watchdog=bool(require_external_watchdog)
        self.watchdog_checked_certified_ticks=0
        self.applied=0
        self.certified=0
        self.minimum_risk_brake=0
        self.unmatched=0
        self.reasons={}
        self.highest_intent_sequence=0
        self.epoch=None

    def observe(self, payload):
        try:
            value=json.loads(payload) if isinstance(payload,str) else payload
            if value['deadman'] is True:
                return
            self.applied+=1
            identity=value['guard_intent_identity']
            mode=identity['authority_mode']
            epoch=identity['run_epoch']
            if not epoch or (self.epoch is not None and epoch!=self.epoch):
                raise ValueError('EPOCH_MISMATCH')
            self.epoch=epoch
            if int(identity['command_stamp_ns'])!=int(value['command_stamp_ns']):
                raise ValueError('STAMP_MISMATCH')
            if not identity['source_id'] or identity['source_state_sequence']<0:
                raise ValueError('SOURCE_MISSING')
            if mode=='CERTIFIED':
                if self.require_external_watchdog:
                    if value.get('independent_watchdog_live') is not True or \
                            value.get('independent_watchdog_reason')!='INDEPENDENT_WATCHDOG_LIVE':
                        raise ValueError('INDEPENDENT_WATCHDOG_CERTIFIED_APPLIED_UNPROVEN')
                    self.watchdog_checked_certified_ticks+=1
                if identity['proof_verified_by_guard'] is not True or int(identity['intent_sequence'])<=0:
                    raise ValueError('CERTIFICATE_MISSING')
                # One Intent may be held across multiple physics ticks. Reject
                # regression but do not count repeated ticks as a new intent.
                if int(identity['intent_sequence'])<self.highest_intent_sequence:
                    raise ValueError('INTENT_REGRESSION')
                self.highest_intent_sequence=max(self.highest_intent_sequence,int(identity['intent_sequence']))
                self.certified+=1
            elif mode=='MINIMUM_RISK_BRAKE':
                if identity['proof_verified_by_guard'] or identity['intent_sequence'] is not None:
                    raise ValueError('BRAKE_MASQUERADING_AS_CERTIFICATE')
                self.minimum_risk_brake+=1
            else:
                raise ValueError('UNKNOWN_AUTHORITY_MODE')
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            self.unmatched+=1
            self.reasons[type(exc).__name__+':'+str(exc)[:60]]=self.reasons.get(type(exc).__name__+':'+str(exc)[:60],0)+1

    def verdict(self):
        return {'applied_ticks':self.applied,'certified_ticks':self.certified,
                'minimum_risk_brake_ticks':self.minimum_risk_brake,
                'unmatched_applied':self.unmatched,
                'last_accepted_intent_sequence':self.highest_intent_sequence,
                'run_epoch_observed':self.epoch,
                'external_watchdog_required':self.require_external_watchdog,
                'external_watchdog_checked_ticks':self.watchdog_checked_certified_ticks,
                'reasons':self.reasons,
                'passed':self.certified>0 and self.unmatched==0}
