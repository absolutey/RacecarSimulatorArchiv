#!/usr/bin/env python3
"""Actual AST and source-contract tests for the observational instrumentation."""
import ast,math,subprocess,sys,time,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent
CONTROL=ROOT/'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control'
class Test(unittest.TestCase):
 def test_guard_safety_bounds(self):
  s=(CONTROL/'actuator_guard_node.py').read_text()
  self.assertIn('if elapsed_ns>15000000:',s)
  from TEST_H2H_DYNAMIC_GUARD_SOURCE import assert_h2h_dynamic_guard_source
  assert_h2h_dynamic_guard_source(s)
  self.assertIn("'voluntary_context_switches':int(_voluntary)",s)
  self.assertIn("'involuntary_context_switches':int(_involuntary)",s)
 def test_ga_safety_bounds(self):
  s=(CONTROL/'guard_ahead.py').read_text()
  self.assertIn('MAX_EXTRA_NS = 7_000_000',s)
  self.assertIn('next_ok=self._certificate',s)
  self.assertIn('current_ok=self._certificate',s)
  self.assertIn('involuntary_context_switches',s)
 def test_ros_source_syntax(self):
  for f in (CONTROL/'actuator_guard_node.py',CONTROL/'guard_ahead.py',CONTROL/'final_preflight.py'):
   ast.parse(f.read_text())
 def test_synthetic_scheduler_bracketing(self):
  import tempfile,json
  from ANALYZE_SCHED_CAUSAL import analyze
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);(p/'actor_traces').mkdir()
   (p/'actor_traces'/'GUARD_7.jsonl').write_text(json.dumps({'event':'GUARD_STOP_FAILURE_SNAPSHOT','monotonic_ns':16_000_000,'pid':7,'snapshot':{'intent_sequence':184}})+'\n')
   with (p/'scheduler_trace.jsonl').open('w') as f:
    for t,run,wait in [(10_000_000,10,10),(20_000_000,1_000_010,5_000_010)]:
     f.write(json.dumps({'steady_ns':t,'sample_overruns':0,'processes':[{'node':'actuator_guard','pid':7,'threads':[{'tid':7,'run_ns':run,'runqueue_wait_ns':wait,'slices':1}]}]})+'\n')
   r=analyze(p)
   self.assertEqual(r['status'],'READ_ONLY_RUNQUEUE_BRACKETING')
   self.assertEqual(r['first_failure']['snapshot']['intent_sequence'],184)
   self.assertAlmostEqual(r['near_failure'][0]['runqueue_wait_delta_ms'],5.)
 def test_no_privileged_or_affinity_changes(self):
  src=(ROOT/'SCHED_DIAG_PROBE.py').read_text()
  for forbidden in ('sched_setaffinity','sched_setscheduler','chrt','sudo','taskset'):
   self.assertNotIn(forbidden,src)
if __name__=='__main__':unittest.main(verbosity=2)
