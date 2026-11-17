#!/usr/bin/env python3
"""Run with Python standard library; no ROS installation required."""
import ast
import hashlib
import json
from pathlib import Path
from unittest import TestCase, main

ROOT=Path(__file__).resolve().parent
LAUNCH=ROOT/'src/platform/f1tenth_racing_bringup/launch/race_foundation.launch.py'
CONTROL=ROOT/'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control'
SIM=ROOT/'src/environment/f1tenth_racing_sim/f1tenth_racing_sim'

def profile_function():
    tree=ast.parse(LAUNCH.read_text())
    funcs=[x for x in tree.body if isinstance(x,ast.FunctionDef) and
           x.name=='validate_execution_profile']
    assert len(funcs)==1
    ns={}
    exec(compile(ast.Module(body=funcs,type_ignores=[]),str(LAUNCH),'exec'),ns)
    return ns['validate_execution_profile']

class P0Checks(TestCase):
    def test_all_authorized_solo_profiles(self):
        f=profile_function()
        for profile,kind,candidate in (
            ('SOLO_BASELINE','pursuit','baseline'),
            ('SOLO_SCP_EXPERIMENTAL','pursuit','scp'),
            ('SOLO_MPC_EXPERIMENTAL','mpc','scp')):
            self.assertEqual(f(profile=profile,solo=True,use_r_b0=True,
                controller_kind=kind,trajectory_candidate=candidate,
                use_opponent_perception=False,use_opponent_prediction=False,
                use_overtaking=False),profile)
    def test_h2h_and_unsafe_mixes_rejected(self):
        f=profile_function()
        valid=dict(profile='SOLO_BASELINE',solo=True,use_r_b0=True,
                   controller_kind='pursuit',trajectory_candidate='baseline',
                   use_opponent_perception=False,use_opponent_prediction=False,
                   use_overtaking=False)
        for modified in (
            dict(solo=False),dict(use_r_b0=False),
            dict(use_opponent_perception=True),dict(use_opponent_prediction=True),
            dict(use_overtaking=True),dict(controller_kind='mpc'),
            dict(trajectory_candidate='scp'),dict(profile='INVALID')):
            with self.subTest(modified=modified),self.assertRaises(RuntimeError):
                f(**(valid | modified))
    def test_safe_launch_defaults(self):
        s=LAUNCH.read_text()
        self.assertIn('DeclareLaunchArgument("solo", default_value="true")',s)
        # The integrated launch now defaults to Solo MPC+SCP (not old P0 pursuit).
        self.assertIn('DeclareLaunchArgument("execution_profile", default_value="SOLO_MPC_EXPERIMENTAL")',s)
        self.assertIn('DeclareLaunchArgument("controller_kind", default_value="mpc")',s)
        self.assertIn('DeclareLaunchArgument("trajectory_candidate", default_value="scp")',s)
        self.assertEqual(profile_function()(profile='SOLO_MPC_EXPERIMENTAL',solo=True,
            use_r_b0=True,controller_kind='mpc',trajectory_candidate='scp',
            use_opponent_perception=False,use_opponent_prediction=False,
            use_overtaking=False),'SOLO_MPC_EXPERIMENTAL')
        self.assertIn('H2H_GUARD_RED_PROOF_REQUIRED',s)
        self.assertIn('"max_speed_cap_mps": speed_cap',s)
    def test_single_source_stop_proof_pinned(self):
        mod=CONTROL/'fast_stop_proof.py'
        source=mod.read_text()
        self.assertIn('class CertifiedFreeStoppingEnvelope(StoppingEnvelope)',source)
        self.assertNotIn('def evaluate(',source)
        env=SIM/'safety/stopping_envelope.py'
        sweep=SIM/'world/swept_collision.py'
        text=source.replace(' ', '')
        for sha in (hashlib.sha256(env.read_bytes()).hexdigest(),
                    hashlib.sha256(sweep.read_bytes()).hexdigest()):
            self.assertIn(sha,text)
    def test_guard_immutable_safety_gates(self):
        g=(CONTROL/'actuator_guard_node.py').read_text()
        ga=(CONTROL/'guard_ahead.py').read_text()
        from TEST_H2H_DYNAMIC_GUARD_SOURCE import assert_h2h_dynamic_guard_source
        assert_h2h_dynamic_guard_source(g)
        self.assertIn('if elapsed_ns>15000000:',g)
        self.assertIn('MAX_EXTRA_NS = 7_000_000',ga)
    def test_scheduling_probe_is_readonly(self):
        src=(ROOT/'SCHED_DIAG_PROBE.py').read_text()
        tree=ast.parse(src)
        assert tree
        for bad in ('sched_setscheduler','sched_setaffinity','rclpy',
                    'create_publisher','actuator_intent0','ackermann_cmd0'):
            self.assertNotIn(bad,src)
    def test_no_mutable_cache_manifest(self):
        manifest=json.loads((ROOT/'RELEASE_HASHES.json').read_text())
        self.assertTrue(all(not (x.startswith('.pytest_cache/') or
                            x.startswith('build/') or x.startswith('install/') or
                            x.startswith('log/')) for x in manifest))

if __name__=='__main__': main(verbosity=2)
