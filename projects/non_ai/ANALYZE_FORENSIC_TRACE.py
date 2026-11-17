#!/usr/bin/env python3
"""Analyze real callback entry/exit stamps, not inferred DDS delivery timing."""
import json, pathlib, sys, collections, statistics
from glob import glob
if len(sys.argv)!=2:
 print('USAGE: python3 -B ANALYZE_FORENSIC_TRACE.py /path/to/F1TENTH_RAB_LATCH_FORENSIC_...');sys.exit(2)
root=pathlib.Path(sys.argv[1]);p=root/'actor_traces'
assert p.is_dir(),p
E=[]
for f in p.glob('*.jsonl'):
 for line in f.read_text().splitlines():
  try:E.append(json.loads(line))
  except Exception: pass
E.sort(key=lambda e:e.get('wall_ns',0))
assert E,'EMPTY_ACTOR_TRACES'
by=collections.defaultdict(list)
for e in E:by[e['event']].append(e)
stat={'events':{k:len(v) for k,v in by.items()},'first_guard_brakes':by['GUARD_BRAKE'][:12],
 'first_rb_intents':by['RB_INTENT_EMIT'][:15]}
def q(vals):
 if not vals:return None
 vals=sorted(vals)
 return {'count':len(vals),'min':vals[0],'median':statistics.median(vals),'p95':vals[min(len(vals)-1,int(.95*(len(vals)-1)))],'max':vals[-1]}
stat['rb_state_callback_physics_age_ms']=q([(e['wall_ns']-e['physics_stamp_ns'])/1e6 for e in by['RB_STATE_ENTER']])
stat['rb_state_callback_duration_ms']=q([e['callback_elapsed_ns']/1e6 for e in by['RB_STATE_EXIT']])
stat['rb_reference_callback_duration_ms']=q([e['callback_elapsed_ns']/1e6 for e in by['RB_REFERENCE_EXIT']])
stat['rb_callback_interarrival_ms']=q([(b['monotonic_ns']-a['monotonic_ns'])/1e6 for a,b in zip(by['RB_STATE_ENTER'],by['RB_STATE_ENTER'][1:])])
stat['rb_watchdog_interarrival_ms']=q([(b['monotonic_ns']-a['monotonic_ns'])/1e6 for a,b in zip(by['RB_WATCHDOG_ENTER'],by['RB_WATCHDOG_ENTER'][1:])])
stat['guard_callback_physics_age_ms']=q([(e['wall_ns']-e['physics_stamp_ns'])/1e6 for e in by['GUARD_STATE_ENTER']])
stat['first_1000_events']=str(root/'first_1000_events.jsonl')
(root/'first_1000_events.jsonl').write_text(''.join(json.dumps(x,sort_keys=True)+'\n' for x in E[:1000]))
(root/'FORENSIC_CALLBACK_SUMMARY.json').write_text(json.dumps(stat,indent=2,ensure_ascii=False,sort_keys=True)+'\n')
print(json.dumps(stat,indent=2,ensure_ascii=False,sort_keys=True))
