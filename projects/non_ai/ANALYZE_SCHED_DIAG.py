#!/usr/bin/env python3
"""Aggregates observed /proc scheduling wait changes, not kernel traces."""
import json
import sys
from collections import defaultdict
from pathlib import Path

def summarize(path):
    prev = {}
    statistics = defaultdict(lambda: {'samples': 0, 'max_delta_wait_ms': 0.0,
                                       'max_delta_cpu_ms': 0.0,
                                       'max_slice_delta': 0})
    for line in Path(path).read_text().splitlines():
        row = json.loads(line)
        for p in row.get('processes', []):
            for t in p.get('threads', []):
                k=(p['pid'], t['tid'])
                this=(t['run_ns'], t['wait_ns'], t['slices'])
                if k in prev:
                    a=prev[k]
                    d=[max(0,v-v0) for v,v0 in zip(this,a)]
                    st=statistics[p['node']]
                    st['samples']+=1
                    st['max_delta_wait_ms']=max(st['max_delta_wait_ms'],d[1]/1e6)
                    st['max_delta_cpu_ms']=max(st['max_delta_cpu_ms'],d[0]/1e6)
                    st['max_slice_delta']=max(st['max_slice_delta'],d[2])
                prev[k]=this
    return dict(statistics)

if __name__=='__main__':
    if len(sys.argv)!=2:raise SystemExit('usage: ANALYZE_SCHED_DIAG.py trace.jsonl')
    print(json.dumps({'scope':'diagnostic_only_not_causal_proof',
                      'statistics':summarize(sys.argv[1])},indent=2,sort_keys=True))
