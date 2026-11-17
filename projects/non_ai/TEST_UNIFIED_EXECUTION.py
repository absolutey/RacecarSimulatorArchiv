#!/usr/bin/env python3
"""Tests extracted from the actual implementation without requiring ROS2."""
import ast, math, time, json, collections, types, unittest, threading
from pathlib import Path
BASE=Path(__file__).resolve().parent
NODE=BASE/'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/node.py'
GA=BASE/'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/guard_ahead.py'
GUARD=BASE/'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/actuator_guard_node.py'
SPEED=BASE/'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/speed_contract_audit.py'

def method(name):
    cl=next(x for x in ast.parse(NODE.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='RacingPursuitController')
    node=next(x for x in cl.body if isinstance(x,ast.FunctionDef) and x.name==name)
    ns={'time':time,'json':json,'String':lambda:types.SimpleNamespace(data='')}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(NODE),'exec'),ns)
    return ns[name]
class Clock:
    def __init__(self,n):self.n=n
    def now(self):return types.SimpleNamespace(nanoseconds=self.n)
class Stub:
    def __init__(self,now=1_000_000_000):
        self.clock=Clock(now);self._pending_state=None;self._processed_state_sequence=-1
        # Match the live callback/worker synchronization contract. This stub
        # deliberately runs the non-worker path; no ROS node is instantiated.
        self._state_receipt_lock=threading.Lock()
        self._control_lock=threading.RLock()
        self.control_worker_enabled=False
        self._control_worker=None
        self._pending_state_steady_ns=None
        self._received_state_sequence=-1;self._last_received_physics_stamp_ns=-1
        # Actual R-B state callback records source-to-receipt age under
        # _state_receipt_lock. Match its diagnostic-only state in this stub;
        # never mock the timestamp checks or the real 75ms execution gate.
        self._state_rx_recent_ages_ns=collections.deque(maxlen=8)
        self._state_rx_receipt_count=0
        self._state_rx_last_receipt_steady_ns=-1
        self.state_freshness_ns=100_000_000;self.received=[];self._stage_telemetry_queue=collections.deque(maxlen=64)
        self._stage_diagnostic_dropped=0;self.last_state_ns=None
        self.stage_timing_pub=types.SimpleNamespace(publish=lambda x:self.received.append(x))
    def get_clock(self):return self.clock
    def receive_state(self,msg):self.received.append(msg.physics_sequence)
    receive_state_snapshot=method('receive_state_snapshot')
    run_latest_state_cycle=method('run_latest_state_cycle')
    flush_stage_diagnostics=method('flush_stage_diagnostics')
def state(seq,stamp):return types.SimpleNamespace(physics_sequence=seq,physics_stamp=types.SimpleNamespace(sec=stamp//10**9,nanosec=stamp%10**9))
class Checks(unittest.TestCase):
    def test_latest_only(self):
        s=Stub();s.receive_state_snapshot(state(1,970_000_000));s.receive_state_snapshot(state(2,975_000_000))
        self.assertEqual(s._pending_state.physics_sequence,2);s.run_latest_state_cycle();self.assertEqual(s.received,[2])
        s.run_latest_state_cycle();self.assertEqual(s.received,[2])
    def test_regression_fails_to_mutate_snapshot(self):
        s=Stub();s.receive_state_snapshot(state(4,980_000_000));s.receive_state_snapshot(state(3,985_000_000))
        self.assertEqual(s._pending_state.physics_sequence,4)
        self.assertEqual(s._last_dropped_compute_reason,'RECEIPT_NON_MONOTONIC_UNPROVEN')
    def test_stamp_regression(self):
        s=Stub();s.receive_state_snapshot(state(1,980_000_000));s.receive_state_snapshot(state(2,970_000_000))
        self.assertEqual(s._pending_state.physics_sequence,1)
    def test_stale_before_compute_no_emit(self):
        s=Stub();s.receive_state_snapshot(state(1,800_000_000));s.run_latest_state_cycle()
        self.assertFalse(s.received);self.assertEqual(s._last_dropped_compute_reason,'PRECOMPUTE_STATE_AGE_UNPROVEN')
    def test_future_stamp_no_emit(self):
        s=Stub();s.receive_state_snapshot(state(1,1_000_000_001));s.run_latest_state_cycle()
        self.assertFalse(s.received)
    def test_explicit_proof_identity_no_extrapolated_stamp(self):
        src=NODE.read_text()
        self.assertIn('self.last_source_state_stamp = message.physics_stamp',src)
        self.assertIn('POSTCOMPUTE_EXPIRED_UNPROVEN',src)
        self.assertIn('source_stamp_ns',src)
        self.assertIn('time.perf_counter_ns()-cycle_started_ns > 25_000_000',src)
    def test_qos_receipt_observability_bounded_without_granting_freshness(self):
        s=Stub()
        for seq in range(1,13):
            s.clock.n=1_000_000_000+seq*25_000_000
            s.receive_state_snapshot(state(seq,s.clock.n-2_000_000))
        self.assertEqual(s._state_rx_receipt_count,12)
        self.assertEqual(list(s._state_rx_recent_ages_ns),[2_000_000]*8)
        self.assertGreater(s._state_rx_last_receipt_steady_ns,0)
        # Regression (out-of-order physics) must NOT impersonate a new sample.
        s.receive_state_snapshot(state(11,s.clock.n+5_000_000))
        self.assertEqual(s._state_rx_receipt_count,12)
        self.assertEqual(len(s._state_rx_recent_ages_ns),8)
        self.assertEqual(s._last_dropped_compute_reason,'RECEIPT_NON_MONOTONIC_UNPROVEN')
    def test_qos_receipt_stale_input_diagnostic_does_not_bypass_gate(self):
        s=Stub(now=1_000_000_000)
        s.receive_state_snapshot(state(1,800_000_000))
        self.assertEqual(list(s._state_rx_recent_ages_ns),[200_000_000])
        s.run_latest_state_cycle()
        self.assertEqual(s.received,[])
        self.assertEqual(s._last_dropped_compute_reason,'PRECOMPUTE_STATE_AGE_UNPROVEN')
    def test_diagnostics_bounded(self):
        s=Stub();[s._stage_telemetry_queue.append({'sequence':i}) for i in range(5)]
        s.flush_stage_diagnostics();self.assertEqual(len(s.received),2)
        self.assertEqual(len(s._stage_telemetry_queue),3)
    def test_gate_preserved(self):
        self.assertIn('if elapsed_ns>15000000:',GUARD.read_text())
        self.assertIn('GUARD_15MS_RESPONSE_DEADLINE',GUARD.read_text())
        self.assertIn('MAX_EXTRA_NS = 7_000_000',GA.read_text())
        self.assertIn('GUARD_AHEAD_RESPONSE_DEADLINE_UNPROVEN',GA.read_text())
        # The former unconditional 'H2H unqualified' marker was replaced by
        # independent, per-candidate Red motion occupancy verification.
        # Assert the effective fail-closed path, not the deleted diagnostic.
        guard=GUARD.read_text()
        self.assertIn("if not self.solo and not self.red_guard_enabled:",guard)
        self.assertIn("H2H_REQUIRES_INDEPENDENT_DYNAMIC_RED_GUARD",guard)
        self.assertIn("RedDynamicWitness(self.red_prediction",guard)
        self.assertIn("witness.collision_free(z,z1,0.0,0.025)",guard)
        self.assertIn("return witness.collision_free(before,after,",guard)
        self.assertIn("H2H_RED_PREDICTION_MISSING_OR_STALE",guard)
        self.assertIn("H2H_RED_WITNESS_UNPROVEN",guard)
        self.assertIn("if dynamic_checker is not None:",guard)
        self.assertIn("return False  # no legacy unbounded/non-dynamic fallback in H2H",guard)
    def test_speed_check_never_qualifies(self):
        ns={};exec(compile(SPEED.read_text(),str(SPEED),'exec'),ns)
        f=ns['audit_nominal_stop_horizon']
        self.assertFalse(f(19.65).within_nominal_horizon)
        self.assertTrue(f(4.0).within_nominal_horizon)
        self.assertFalse(hasattr(f(4.0),'qualified'))
        with self.assertRaises(ValueError):f(float('nan'))
if __name__=='__main__':unittest.main(verbosity=2)
