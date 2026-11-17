#!/usr/bin/env python3
"""Associate first Guard failure with /proc thread runqueue wait deltas.
Read-only, no scheduling causality is implied by time-coincident intervals.
"""
import json,sys
from pathlib import Path

def rows(path):
    with open(path,encoding='utf-8') as f:
        for line in f:
            if line.strip():yield json.loads(line)

def get_first_failure(directory):
    paths=sorted((directory/'actor_traces').glob('GUARD_*.jsonl'))
    for path in paths:
        for e in rows(path):
            if e.get('event')=='GUARD_STOP_FAILURE_SNAPSHOT':
                return {'monotonic_ns':e.get('monotonic_ns'), 'pid':e.get('pid'),
                        'snapshot':e.get('snapshot',{})}
    return None

def analyze(directory):
    failure=get_first_failure(directory)
    trace=directory/'scheduler_trace.jsonl'
    if not trace.exists():return {'status':'NO_SCHED_SAMPLES','first_failure':failure}
    previous={}; stats={}; near=[]; count=0; overruns=0
    fail_t=failure.get('monotonic_ns') if failure else None
    for rec in rows(trace):
        count+=1;overruns=max(overruns,int(rec.get('sample_overruns',0)))
        t=rec['steady_ns']
        for proc in rec['processes']:
            label=proc['node']
            for th in proc['threads']:
                k=(proc['pid'],th['tid']); cur=(th['run_ns'],th['runqueue_wait_ns'],th['slices'],t)
                if k in previous:
                    prior=previous[k]
                    elapsed=t-prior[3]
                    if elapsed<=0:continue
                    dw=max(0,cur[1]-prior[1]); dc=max(0,cur[0]-prior[0])
                    s=stats.setdefault(label,{'samples':0,'max_wait_delta_ms':0.,'max_cpu_delta_ms':0.,'max_sampling_gap_ms':0.})
                    s['samples']+=1
                    s['max_wait_delta_ms']=max(s['max_wait_delta_ms'],dw/1e6)
                    s['max_cpu_delta_ms']=max(s['max_cpu_delta_ms'],dc/1e6)
                    s['max_sampling_gap_ms']=max(s['max_sampling_gap_ms'],elapsed/1e6)
                    if fail_t is not None and (prior[3]-25_000_000<=fail_t<=t+25_000_000):
                        near.append({'node':label,'pid':k[0],'tid':k[1],
                          'interval_start_ns':prior[3],'interval_end_ns':t,
                          'interval_ms':elapsed/1e6,'runqueue_wait_delta_ms':dw/1e6,
                          'cpu_run_delta_ms':dc/1e6,
                          'covers_first_failure':prior[3]<=fail_t<=t})
                previous[k]=cur
    near.sort(key=lambda r:(not r['covers_first_failure'],abs(r['interval_end_ns']-(fail_t or 0))))
    return {'status':'READ_ONLY_RUNQUEUE_BRACKETING','source':'proc_schedstat_not_sched_switch',
      'samples':count,'sample_overruns':overruns,'first_failure':failure,
      'per_node':stats,'near_failure':near[:100],
      'interpretation':'Exact runnable/sleep/preemption split needs kernel sched_switch/wakeup; do NOT automatically change safety limits, priorities, or affinity.'}
if __name__=='__main__':
    if len(sys.argv)!=2:raise SystemExit('usage: ANALYZE_SCHED_CAUSAL.py RUN_DIR')
    print(json.dumps(analyze(Path(sys.argv[1])),ensure_ascii=False,indent=2))
