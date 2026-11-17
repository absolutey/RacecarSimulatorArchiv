#!/usr/bin/env python3
"""Offline fail-closed deadline, hard brake publish order and live #508 proof.

No ROS process or vehicle execution. Never promotes a candidate to qualified.
"""
import ast
import json
import math
import sys
import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / 'src/environment/f1tenth_racing_sim'))
from f1tenth_racing_sim.physics.vehicle import State
from f1tenth_racing_sim.safety import stopping_envelope as stop_module
from f1tenth_racing_sim.safety.stopping_envelope import StoppingEnvelope
from f1tenth_racing_sim.world.map import OccupancyMap
import yaml

SOURCE = HERE / 'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/actuator_guard_node.py'
INTEGRITY = HERE / 'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/guard_state_integrity.py'


def live_fixture():
    sensor = json.loads((HERE / 'FIRST_STOP_SNAPSHOT_508.json').read_text())
    cfg = yaml.safe_load((HERE / 'src/environment/f1tenth_racing_sim/config/competition_environment.yaml').read_text())
    root = HERE / 'src/environment/f1tenth_racing_sim'
    import hashlib
    image = root / cfg['map']['image_file']
    assert hashlib.sha256(image.read_bytes()).hexdigest() == sensor['identity']['map_sha256']
    occ = OccupancyMap(image, cfg['map'])
    e = StoppingEnvelope(cfg['dynamics'], occ,
                         cfg['collision']['footprint_relative_to_base_link'],
                         reaction_s=.25, uncertainty_padding_m=.03)
    r = sensor['last_state']
    i = sensor['intent']
    z = State(x=r['px'], y=r['py'], yaw=r['yaw'], speed=r['v'], vx=r['vx'], vy=r['vy'],
              yaw_rate=r['omega'], acceleration=r['a'], ax=r['ax'], ay=r['ay'],
              slip=r['slip_angle'], requested_acceleration=r['accel'], jerk=r['jerk'],
              steering=r['steer'], steering_rate=r['steer_vel'])
    z1 = replace(z)
    e.dynamics.step(z1, i['steering_rad'], i['acceleration_mps2'], .025)
    return e, z1, i


def isolated_brake_method():
    module=ast.parse(SOURCE.read_text())
    guard=next(x for x in module.body if isinstance(x,ast.ClassDef) and x.name=='IndependentActuatorGuard')
    brake=next(x for x in guard.body if isinstance(x,ast.FunctionDef) and x.name=='_brake')
    namespace={}
    class FakeAckermann:
        def __init__(self):
            self.header=SimpleNamespace(stamp=None)
            self.drive=SimpleNamespace(steering_angle=None,acceleration=None)
    # Compile the actual release's numeric state checker (without ROS).
    safety_env={}
    exec(compile(INTEGRITY.read_text(),str(INTEGRITY),'exec'),safety_env)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[brake],type_ignores=[])),str(SOURCE),'exec'),
         {'time':time, 'math':math, 'json':json, 'AckermannDriveStamped':FakeAckermann,
          'finite_physics_state':safety_env['finite_physics_state']},namespace)
    return namespace['_brake']


def fake_guard(*,fresh=True):
    operations=[]
    rx_ns=time.monotonic_ns() if fresh else time.monotonic_ns()-200_000_000
    state=SimpleNamespace(px=0.,py=0.,yaw=0.,v=2.,vx=2.,vy=0.,
        omega=0.,a=0.,ax=0.,ay=0.,slip_angle=0.,accel=0.,
        jerk=0.,steer=0.,steer_vel=0.)
    def pub(m):
        assert g.contract.state=='SAFE_HOLD' and g.emergency_unrecoverable
        operations.append(('applied',m.drive.acceleration))
    g=SimpleNamespace(
        pending_future_intent='old',pending_future_deadline_steady_ns=123,
        pending_future_received_steady_ns=123,
        last_state=state, state_received_steady_ns=rx_ns,
        last_state_sequence=829,last_accepted_sequence=507,
        last_accepted_acceleration_mps2=2.,
        emergency_unrecoverable=False,collision=False,
        startup_waiting_for_fresh_state=False,last_certificate=True,last_reason='',
        contract=SimpleNamespace(state='RUNNING'),
        final_pub=SimpleNamespace(publish=pub),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(
            nanoseconds=100, to_msg=lambda:100)),
        get_logger=lambda:SimpleNamespace(error=lambda s:operations.append(('log',s))),
    )
    return g,operations


class ExecutionPathTests(unittest.TestCase):
    def test_508_unbounded_full_certificate_unchanged(self):
        e,z,i=live_fixture()
        r=e.evaluate(z,i['steering_rad'],i['acceleration_mps2'])
        self.assertTrue(r.qualified)
        self.assertEqual(r.steps,61)
        self.assertAlmostEqual(r.stopping_time_s,1.525,places=9)
        self.assertAlmostEqual(r.stopping_distance_m,2.2881603603908602,places=9)

    def test_508_bounded_generous_deadline_same_proof(self):
        e,z,i=live_fixture()
        r=e.evaluate_bounded(z,i['steering_rad'],i['acceleration_mps2'],time.perf_counter_ns()+200_000_000)
        self.assertTrue(r.qualified)
        self.assertEqual(r.steps,61)

    def test_expired_deadline_never_approves(self):
        e,z,i=live_fixture()
        r=e.evaluate_bounded(z,i['steering_rad'],i['acceleration_mps2'],time.perf_counter_ns()-1)
        self.assertFalse(r.qualified)
        self.assertEqual(r.reason,'PROOF_BUDGET_UNPROVEN')
        self.assertEqual(r.steps,0)

    def test_midrollout_deadline_does_not_approve(self):
        e,z,i=live_fixture()
        origin=stop_module.time
        calls=[0]
        class FakeTime:
            @staticmethod
            def perf_counter_ns():
                calls[0]+=1
                return calls[0]
        stop_module.time=FakeTime
        try:
            r=e.evaluate_bounded(z,i['steering_rad'],i['acceleration_mps2'],3)
        finally:
            stop_module.time=origin
        self.assertFalse(r.qualified)
        self.assertEqual(r.reason,'PROOF_BUDGET_UNPROVEN')
        self.assertTrue(0<r.steps<61)

    def test_brake_publishes_before_synchronous_log(self):
        brake=isolated_brake_method()
        guard,events=fake_guard()
        brake(guard,'GUARD_REJECT:STOP_CERTIFICATE_NOT_VERIFIED')
        self.assertEqual(events[0],('applied',-2.0))
        self.assertEqual(events[1][0],'log')
        self.assertTrue(guard.emergency_unrecoverable)
        self.assertEqual(guard.contract.state,'SAFE_HOLD')
        self.assertEqual(guard.first_brake_record['state_sequence'],829)
        brake(guard,'STATE_WATCHDOG_TIMEOUT')
        self.assertEqual([k for k,_ in events].count('log'),1)
        self.assertEqual(guard.first_brake_record['reason'],'GUARD_REJECT:STOP_CERTIFICATE_NOT_VERIFIED')

    def test_stale_state_keeps_deadman_and_records_fault(self):
        brake=isolated_brake_method()
        guard,events=fake_guard(fresh=False)
        brake(guard,'STATE_WATCHDOG_TIMEOUT')
        self.assertFalse(any(k=='applied' for k,_ in events))
        self.assertEqual(events[0][0],'log')
        self.assertTrue(guard.emergency_unrecoverable)

    def test_watchdog_first_report_is_after_brake(self):
        module=ast.parse(SOURCE.read_text())
        guard=next(x for x in module.body if isinstance(x,ast.ClassDef) and x.name=='IndependentActuatorGuard')
        watchdog=next(x for x in guard.body if isinstance(x,ast.FunctionDef) and x.name=='watchdog')
        order=[]
        for node in ast.walk(watchdog):
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute):
                if node.func.attr=='_brake' and node.args and isinstance(node.args[0],ast.Constant) and node.args[0].value=='STATE_WATCHDOG_TIMEOUT':
                    order.append(('brake',node.lineno))
                if node.func.attr=='error' and node.args and isinstance(node.args[0],ast.BinOp) and 'FIRST_STATE_WATCHDOG' in ast.unparse(node.args[0]):
                    order.append(('log',node.lineno))
        self.assertEqual([t for t,_ in sorted(order,key=lambda x:x[1])],['brake','log'])

    def test_safety_limits_preserved(self):
        g=SOURCE.read_text()
        a=(HERE / 'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/guard_ahead.py').read_text()
        self.assertIn('if elapsed_ns>15000000:',g)
        self.assertIn('MAX_EXTRA_NS = 7_000_000',a)
        from TEST_H2H_DYNAMIC_GUARD_SOURCE import assert_h2h_dynamic_guard_source
        assert_h2h_dynamic_guard_source(g)
        self.assertIn('self.final_pub.publish(cmd)',g)


if __name__=='__main__':
    unittest.main(verbosity=2)
