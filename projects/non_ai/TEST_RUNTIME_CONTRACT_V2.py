#!/usr/bin/env python3
"""Source-executed regression of real Guard publish method and R-B mailbox."""
import ast
import json
import sys
import time
import types
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parent
CTRL=ROOT/'src/driving/algorithms/f1tenth_racing_control'
SIM=ROOT/'src/environment/f1tenth_racing_sim'
sys.path[:0]=[str(CTRL),str(SIM)]
# Load the dependency-free contract directly. Importing the package __init__
# would transitively require the ROS geometry overlay, unavailable here.
import importlib.util
p_contract=CTRL/'f1tenth_racing_control/actuator_guard_contract.py'
spec=importlib.util.spec_from_file_location('rtv2_guard_contract',p_contract)
contract_module=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=contract_module
spec.loader.exec_module(contract_module)
ActuatorGuardContract=contract_module.ActuatorGuardContract
ActuatorIntent=contract_module.ActuatorIntent

def ns(m):return m.sec*1000000000+m.nanosec

def stamp(n):return types.SimpleNamespace(sec=n//1000000000,nanosec=n%1000000000)

class Clock:
    def __init__(self, n, advance_on_call=None):
        self.n=n;self.calls=0;self.advance_on_call=advance_on_call or {}
    def now(self):
        self.calls+=1
        self.n+=self.advance_on_call.get(self.calls,0)
        n=self.n
        return types.SimpleNamespace(nanoseconds=n,to_msg=lambda:stamp(n))

class DummyMessage:
    def __init__(self, created, expiry, source, sequence=1):
        self.source_id='R_B';self.run_epoch='epoch';self.sequence=sequence
        self.created_ros_stamp=stamp(created)
        self.expires_ros_stamp=stamp(expiry)
        self.source_state_stamp=stamp(source)
        self.source_state_sequence=10
        self.active_reference_id='ref';self.active_revision=1
        self.track_id='iccas2025';self.map_sha256='hash'
        self.steering_rad=0.0;self.acceleration_mps2=1.0

class Drive:
    def __init__(self):
        self.header=types.SimpleNamespace(stamp=None)
        self.drive=types.SimpleNamespace(steering_angle=None,acceleration=None)

def get_guard_method():
    path=CTRL/'f1tenth_racing_control/actuator_guard_node.py'
    cl=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='IndependentActuatorGuard')
    fn=next(n for n in cl.body if isinstance(n,ast.FunctionDef) and n.name=='_validate_and_publish_intent')
    space={'time':time,'ns':ns,'IntentContract':ActuatorIntent,'AckermannDriveStamped':Drive,
           'GuardedActuatorCommand':types.SimpleNamespace,'String':types.SimpleNamespace,'json':json}
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),space)
    return space['_validate_and_publish_intent']

METHOD=get_guard_method()
class GuardStub:
    def __init__(self, now=1_000_000_000, advances=None, proof_advance=0, state_steady_age_ns=1_000_000):
        self.clock=Clock(now,advances)
        self.identity=('epoch','R_B','iccas2025','hash')
        self.reference_id='ref';self.reference_revision=1
        self.emergency_unrecoverable=False;self.collision=False
        self.contract=ActuatorGuardContract()
        self.contract.arm(run_epoch='epoch',source_id='R_B',reference_id='ref',track_id='iccas2025',map_sha256='hash',ros_time_ns=now-50_000_000,steady_time_ns=time.monotonic_ns()-50_000_000)
        self.state_received_steady_ns=time.monotonic_ns()-state_steady_age_ns
        self._stop_probe_last_detail=None;self._stop_probe_emitted=True
        self.last_reason=None;self.last_certificate=None
        self.stop_reasons=[];self.commands=[];self.atomic_commands=[];self.receipts=[]
        self.final_pub=types.SimpleNamespace(publish=self.commands.append)
        self.atomic_pub=types.SimpleNamespace(publish=self.atomic_commands.append)
        self.intent_receipt_pub=types.SimpleNamespace(publish=self.receipts.append)
        self.proof_advance=proof_advance
        self.last_accepted_sequence=0
        self.last_accepted_acceleration_mps2=0
        self.last_intent_steady_ns=None
    def get_clock(self):return self.clock
    def _arm_if_ready(self):return None
    def _brake(self,reason):self.stop_reasons.append(reason)
    def _certificate(self,msg):
        self.clock.n+=self.proof_advance
        return True
    _validate_and_publish_intent=METHOD

class Tests(unittest.TestCase):
    def msg(self,now=1_000_000_000, exp=75_000_000):
        return DummyMessage(now-10_000_000,now+exp,now-20_000_000)
    def test_normal_granted_only_after_real_contract_and_proof(self):
        g=GuardStub();g._validate_and_publish_intent(self.msg())
        self.assertEqual(len(g.commands),1)
        self.assertEqual(len(g.atomic_commands),1)
        self.assertEqual(g.atomic_commands[0].authority_mode,'CERTIFIED')
        self.assertTrue(g.atomic_commands[0].stop_certificate_verified)
        self.assertEqual(g.atomic_commands[0].intent_sequence,1)
        self.assertEqual(len(g.receipts),1)
        self.assertEqual(g.last_accepted_sequence,1)
        self.assertEqual(g.contract.state,'RUNNING')
    def test_expiry_during_proof_rejected_by_real_contract(self):
        g=GuardStub(proof_advance=10_000_000)
        g._validate_and_publish_intent(self.msg(exp=5_000_000))
        self.assertEqual(g.commands,[])
        self.assertIn('GUARD_REJECT:STALE_OR_FUTURE_INTENT',g.stop_reasons)
    def test_expiry_between_validation_and_publish_rejected(self):
        # Guard time calls: entry, post-proof validation, response stamp,
        # post-response publish validation. Only the last call advances.
        g=GuardStub(advances={4:10_000_000})
        g._validate_and_publish_intent(self.msg(exp=5_000_000))
        self.assertEqual(g.commands,[])
        self.assertEqual(g.stop_reasons,['POST_CERTIFICATE_LEASE_EXPIRED'])
    def test_steady_state_lease_expired_cannot_publish(self):
        g=GuardStub(state_steady_age_ns=150_000_000)
        g._validate_and_publish_intent(self.msg())
        self.assertEqual(g.commands,[])
        self.assertIn('GUARD_REJECT:STALE_OR_FUTURE_STATE',g.stop_reasons)
    def test_future_intent_rejected(self):
        g=GuardStub();m=self.msg();m.created_ros_stamp=stamp(g.clock.n+1)
        g._validate_and_publish_intent(m)
        self.assertEqual(g.commands,[])
    def test_direct_clock_regression_fails_closed(self):
        g=GuardStub(advances={4:-10_000_000})
        g._validate_and_publish_intent(self.msg())
        self.assertEqual(g.commands,[])
        self.assertEqual(g.stop_reasons,['POST_CERTIFICATE_LEASE_EXPIRED'])
    def test_r_b_mailbox_5ms_drain_no_duplicate_or_made_up_state(self):
        src=(CTRL/'f1tenth_racing_control/node.py').read_text()
        self.assertIn('self.create_timer(0.005, self.run_latest_state_cycle)',src)
        self.assertNotIn('self.create_timer(0.025, self.run_latest_state_cycle)',src)
        module={}
        # Load exact source-based test helpers without launching ROS.
        t=ROOT/'TEST_UNIFIED_EXECUTION.py'
        space={'__name__':'source_test_helper','__file__':str(t)}
        exec(compile(t.read_text(),str(t),'exec'),space)
        s=space['Stub'](now=1_000_000_000)
        message=space['state']
        for seq in range(1,41):
            # Mock physical states at 25ms intervals and a coalesced 5ms poll.
            source=900_000_000+seq*25_000_000
            s.clock.n=source+1_000_000
            s.receive_state_snapshot(message(seq,source))
            for phase in range(5):
                s.clock.n=source+(phase+1)*1_000_000
                s.run_latest_state_cycle()
        self.assertEqual(s.received,list(range(1,41)))
    def test_hard_safety_limits_and_single_publisher_preserved(self):
        ga=(CTRL/'f1tenth_racing_control/guard_ahead.py').read_text()
        guard=(CTRL/'f1tenth_racing_control/actuator_guard_node.py').read_text()
        self.assertIn('MAX_EXTRA_NS = 7_000_000',ga)
        self.assertIn('if elapsed_ns>15000000:',guard)
        from TEST_H2H_DYNAMIC_GUARD_SOURCE import assert_h2h_dynamic_guard_source
        assert_h2h_dynamic_guard_source(guard)
        self.assertIn("'ackermann_cmd0'",guard)

if __name__=='__main__':unittest.main(verbosity=2)
