#!/usr/bin/env python3
"""Diagnostics-only nonblocking writer test in separate process."""
from pathlib import Path
import os,sys,subprocess,json,tempfile
root=Path(__file__).resolve().parent
with tempfile.TemporaryDirectory() as d:
 env=os.environ.copy();env['F1TENTH_FORENSIC_TRACE_DIR']=d
 env['PYTHONPATH']=':'.join(str(x.parent) for x in (root/'src').rglob('setup.py'))
 source="from f1tenth_racing_control.forensic_trace import trace_guard; [trace_guard('TEST_TICK', physics_sequence=i) for i in range(4000)]"
 r=subprocess.run([sys.executable,'-c',source],env=env,capture_output=True,text=True,timeout=12)
 assert r.returncode==0,(r.stdout,r.stderr)
 files=list(Path(d).glob('GUARD_*.jsonl'))
 assert len(files)==1,files
 events=[json.loads(x) for x in files[0].read_text().splitlines()]
 assert len(events)==4000,len(events)
 assert [e['physics_sequence'] for e in events]==list(range(4000))
 print('ASYNC_TRACE_4000_EVENTS_PASS')
