"""A-stage safety regressions; do not start ROS or alter Guard authority."""
from dataclasses import dataclass
from types import SimpleNamespace
import time
import pytest

from f1tenth_racing_control.final_preflight import admit_final_command

@dataclass(frozen=True)
class Proposal:
    steering_rad: float = -0.075
    acceleration_mps2: float = 8.7356
    mode: str = 'PRIMARY_VALID'
    reason: str = 'MPC_VALID'
    degradation_active: bool = False
    target_speed_mps: float = 5.0

class Brake:
    def degraded(self, steer, reason, speed, dt):
        return Proposal(steering_rad=steer, acceleration_mps2=-2.0,
                        mode='BRAKE',reason=reason,degradation_active=True,
                        target_speed_mps=0.0)

class Cert:
    def __init__(self,ok,accel=3.739):self.ok=ok;self.accel=accel;self.calls=0
    def choose(self,msg,steer,accel,t):
        self.calls+=1
        return SimpleNamespace(proved=self.ok,steering_rad=steer,
              acceleration_mps2=self.accel,reason='CURRENT_NEXT_PROOF' if self.ok else 'SWEPT_COLLISION')

def go(mode='PRIMARY_VALID',cert=None,**opts):
    c=cert or Cert(True)
    proposal=Proposal(mode=mode)
    kwargs=dict(message=object(),state_finite=True,actual_speed_mps=.434,
                reference_valid=True,cycle_started_ns=time.perf_counter_ns(),
                reserve_ns=8_000_000,previous_steering=-.07,dt_s=.025,
                guard_ahead=c,controller=Brake())
    kwargs.update(opts)
    return admit_final_command(proposal,**kwargs),c

@pytest.mark.parametrize('mode',['PRIMARY_VALID','FALLBACK_STARTUP_ACCEL_RELEASE',
  'DYNAMIC_ADMISSION','STABILITY_RECOVERY','NORMAL_TRACKING'])
def test_every_positive_mode_denial_brakes(mode):
    outcome,cert=go(mode,Cert(False))
    assert cert.calls==1 and outcome.rejected
    assert outcome.decision.acceleration_mps2<=0

def test_primary_moderation_is_not_still_primary():
    outcome,_=go()
    assert outcome.changed and not outcome.rejected
    assert outcome.decision.mode!='PRIMARY_VALID'
    assert outcome.decision.acceleration_mps2==3.739

def test_budget_failure_does_not_issue_positive():
    outcome,cert=go(cert=Cert(True),cycle_started_ns=time.perf_counter_ns()-30_000_000)
    assert outcome.rejected and cert.calls==0
    assert outcome.decision.acceleration_mps2<=0

def test_invalid_context_not_authorized():
    outcome,cert=go(cert=Cert(True),reference_valid=False)
    assert outcome.rejected and cert.calls==0

def test_negative_command_must_prove_stop_corridor():
    # Regression of V4's swept-wall collision while STABILIZE was braking.
    # A signed nonpositive command is NOT its own stop certificate.
    cert=Cert(False, accel=-2.0)
    outcome=admit_final_command(Proposal(acceleration_mps2=-2),message=object(),
       state_finite=True,actual_speed_mps=.434,reference_valid=True,
       cycle_started_ns=time.perf_counter_ns(),reserve_ns=8_000_000,
       previous_steering=-.07,dt_s=.025,guard_ahead=cert,controller=Brake())
    assert cert.calls==1
    assert outcome.rejected and outcome.changed
    assert outcome.reason=='SWEPT_COLLISION'
    assert outcome.decision.acceleration_mps2<=0


def test_negative_command_valid_stop_certificate_no_mutation():
    cert=Cert(True,accel=-2.0)
    outcome=admit_final_command(Proposal(acceleration_mps2=-2),message=object(),
       state_finite=True,actual_speed_mps=.434,reference_valid=True,
       cycle_started_ns=time.perf_counter_ns(),reserve_ns=8_000_000,
       previous_steering=-.07,dt_s=.025,guard_ahead=cert,controller=Brake())
    assert cert.calls==1 and not outcome.rejected and not outcome.changed
    assert outcome.decision.acceleration_mps2==-2.0


def test_negative_command_cannot_become_positive_even_with_claimed_proof():
    # A wrongly shaped or malicious proof result cannot turn braking into gas.
    cert=Cert(True,accel=3.739)
    outcome=admit_final_command(Proposal(acceleration_mps2=-2),message=object(),
       state_finite=True,actual_speed_mps=.434,reference_valid=True,
       cycle_started_ns=time.perf_counter_ns(),reserve_ns=8_000_000,
       previous_steering=-.07,dt_s=.025,guard_ahead=cert,controller=Brake())
    assert cert.calls==1 and outcome.rejected
    assert outcome.reason=='FINAL_PREFLIGHT_UNSAFE_CERT_OUTPUT'
    assert outcome.decision.acceleration_mps2<=0
