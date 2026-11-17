#!/usr/bin/env python3
"""Run real Guard._certificate AST with ROS-independent test fixtures.

This is *not* an ROS or physical lap qualification.  The test intentionally
executes the actual release Guard method, not a second hand-written algorithm.
"""
import ast
import copy
import json
import math
import time
import unittest
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent
GUARD = ROOT/'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/actuator_guard_node.py'
SNAPSHOT = ROOT/'FIRST_P0_TICK_FAILURE_SNAPSHOT.json'


def load_method():
    source=ast.parse(GUARD.read_text())
    classes=[x for x in source.body if isinstance(x,ast.ClassDef) and x.name=='IndependentActuatorGuard']
    assert len(classes)==1
    methods=[x for x in classes[0].body if isinstance(x,ast.FunctionDef) and x.name=='_certificate']
    assert len(methods)==1
    namespace={'math':math,'time':time,'ns':lambda t: int(t.sec)*1_000_000_000+int(t.nanosec)}
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods,type_ignores=[])),str(GUARD),'exec'),namespace)
    return namespace['_certificate']


def tmsg(n):
    return SimpleNamespace(sec=n//10**9,nanosec=n%10**9)


@dataclass
class DummyPhysics:
    x: float=0.0


class ProbeEnvelope:
    def __init__(self):
        self.step_calls=0
        self.evaluate_calls=0
        self.dynamics=SimpleNamespace(step=self.step)
    def step(self,*args):
        self.step_calls+=1
    def evaluate(self,*args):
        self.evaluate_calls+=1
        return SimpleNamespace(qualified=True)


def make_case():
    data=json.loads(SNAPSHOT.read_text())
    p,l,i=data['previous_state'],data['last_state'],data['intent']
    state=lambda o:SimpleNamespace(**{**o,'physics_stamp':tmsg(o['physics_stamp_ns'])})
    env=ProbeEnvelope()
    guard=SimpleNamespace(
        last_state=state(l),previous_state=state(p),last_state_sequence=int(data['last_state_sequence']),
        environment={'dynamics':{'limits':{'steering_angle_abs_max_rad':4.0,
            'acceleration_max_m_s2':40.0,'deceleration_max_m_s2':40.0}},
            'simulation':{'physics_timestep_s':0.025,'physics_rate_hz':40.0}},
        get_parameter=lambda name:SimpleNamespace(value=20.0),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=data['now_ros_ns'])),
        identity=('epoch','R_B','iccas2025','map'),reference_revision=1,
        contract=SimpleNamespace(state_age_limit_ns=100_000_000),collision=False,
        solo=True,native_dual_proof=None,
        _state=lambda:DummyPhysics(),envelope=env,
        stop_proof_max_duration_us=0.0,stop_proof_slow_count=0,
        _stop_probe_last_detail=None,last_reason='',
    )
    intent=SimpleNamespace(source_state_sequence=int(i['source_state_sequence']),
          source_state_stamp=tmsg(i['source_state_stamp_ns']),steering_rad=i['steering_rad'],
          acceleration_mps2=i['acceleration_mps2'],active_revision=1)
    return guard,intent,env,data


class GuardTickTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.certify=staticmethod(load_method())
    def test_original_20261018_failure_crosscheck(self):
        g,m,e,d=make_case()
        prev,last=d['previous_state'],d['last_state']
        gap=(last['physics_stamp_ns']-prev['physics_stamp_ns'])*1e-9
        xy=math.hypot(last['px']-prev['px'],last['py']-prev['py'])
        speed=max(abs(prev['v']),abs(last['v']))
        self.assertGreater(xy,speed*gap+0.02)  # old broken guard predicate
        self.assertLessEqual(xy,speed*0.025+0.02)  # true fixed physics step
        self.assertTrue(self.certify(g,m))
        self.assertEqual(e.evaluate_calls,1)  # full stop proof STILL mandatory
    def test_wall_stamp_not_used_as_physics_timestep(self):
        g,m,e,d=make_case()
        self.assertAlmostEqual((g.last_state.physics_stamp_ns - g.previous_state.physics_stamp_ns)/1e9,0.015242463,places=9)
        self.assertTrue(self.certify(g,m))
    def test_teleport_is_rejected_before_proof(self):
        g,m,e,d=make_case()
        g.last_state.py+=1.0
        self.assertFalse(self.certify(g,m))
        self.assertEqual(e.evaluate_calls,0)
        self.assertEqual(g._stop_probe_last_detail['branch'],'PHYSICS_TICK_DISPLACEMENT_UNREACHABLE')
    def test_one_tick_velocity_discontinuity_rejected(self):
        g,m,e,d=make_case()
        g.last_state.v-=0.2
        self.assertFalse(self.certify(g,m))
        self.assertEqual(e.evaluate_calls,0)
    def test_one_tick_yaw_jump_rejected(self):
        g,m,e,d=make_case()
        g.last_state.yaw+=0.2
        self.assertFalse(self.certify(g,m))
        self.assertEqual(e.evaluate_calls,0)
    def test_out_of_order_state_sequence_rejected(self):
        g,m,e,d=make_case()
        g.last_state_sequence+=1
        self.assertFalse(self.certify(g,m))
        self.assertEqual(e.evaluate_calls,0)
    def test_stale_source_stamp_rejected(self):
        g,m,e,d=make_case()
        # preserve stamp match between intent and previous state; invalidate freshness
        g.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=d['now_ros_ns']+110_000_000))
        self.assertFalse(self.certify(g,m))
        self.assertEqual(e.evaluate_calls,0)
    def test_physics_timestamp_regression_rejected(self):
        g,m,e,d=make_case()
        g.last_state.physics_stamp=tmsg(d['previous_state']['physics_stamp_ns']-1)
        self.assertFalse(self.certify(g,m))
        self.assertEqual(e.evaluate_calls,0)
    def test_inconsistent_step_rate_rejected(self):
        g,m,e,d=make_case()
        g.environment['simulation']['physics_timestep_s']=0.05
        self.assertFalse(self.certify(g,m))
        self.assertEqual(e.evaluate_calls,0)
        self.assertEqual(g._stop_probe_last_detail['branch'],'PHYSICS_TICK_CONTRACT_INVALID')
    def test_nonfinite_physics_rejected(self):
        g,m,e,d=make_case()
        g.last_state.px=float('nan')
        self.assertFalse(self.certify(g,m))
        self.assertEqual(e.evaluate_calls,0)
    def test_nonfinite_command_rejected(self):
        g,m,e,d=make_case()
        m.steering_rad=float('nan')
        self.assertFalse(self.certify(g,m))
        self.assertEqual(e.evaluate_calls,0)
    def test_exact_sequence_no_lag(self):
        g,m,e,d=make_case()
        m.source_state_sequence=g.last_state_sequence
        m.source_state_stamp=g.last_state.physics_stamp
        self.assertTrue(self.certify(g,m))
        self.assertEqual(e.evaluate_calls,1)
    def test_failing_stop_envelope_remains_rejection(self):
        g,m,e,d=make_case()
        e.evaluate=lambda *args:SimpleNamespace(qualified=False,reason='COLLISION',
                   stopping_time_s=0.1,stopping_distance_m=0.2,
                   max_abs_slip_rad=0.0,max_lateral_accel_mps2=0.0,steps=1)
        self.assertFalse(self.certify(g,m))
        self.assertEqual(g._stop_probe_last_detail['branch'],'STOPPING_ENVELOPE_REJECT')
    def test_guard_and_guardahead_deadlines_unmodified(self):
        g=GUARD.read_text()
        assert 'if elapsed_ns>15000000:' in g
        from TEST_H2H_DYNAMIC_GUARD_SOURCE import assert_h2h_dynamic_guard_source
        assert_h2h_dynamic_guard_source(g)
        ga=(GUARD.parent/'guard_ahead.py').read_text()
        assert 'MAX_EXTRA_NS = 7_000_000' in ga

if __name__=='__main__':
    unittest.main(verbosity=2)
