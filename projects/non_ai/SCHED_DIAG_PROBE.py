#!/usr/bin/env python3
"""Read-only thread scheduling sampler; no affinity/priority changes.
10ms samples bracket kernel schedstat deltas; they are NOT event-level traces.
"""
import argparse, json, os, signal, time
from pathlib import Path
STOP=False
TARGETS=('actuator_guard','r_b0_controller','competition_simulator','simulator','flight_recorder')
def handle_signal(*_):
    global STOP
    STOP=True

def discover():
    targets={}
    for p in Path('/proc').iterdir():
        if not p.name.isdecimal() or int(p.name)==os.getpid():continue
        try:
            words=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace').split()
            label=next((target for target in TARGETS if any(Path(w).name==target or Path(w).name.startswith(target+'.') for w in words)),None)
            if label:targets[int(p.name)]=label
        except (PermissionError, FileNotFoundError, ProcessLookupError):continue
    return targets

def metrics(pid, label):
    tasks=[]
    for path in (Path('/proc')/str(pid)/'task').iterdir():
        try:
            tid=int(path.name)
            vals=[int(v) for v in (path/'schedstat').read_text().split()[:3]]
            if len(vals)!=3:continue
            tasks.append({'tid':tid,'run_ns':vals[0],'runqueue_wait_ns':vals[1],'slices':vals[2]})
        except (PermissionError,FileNotFoundError,ProcessLookupError,ValueError):continue
    return {'pid':pid,'node':label,'threads':tasks}

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--out',required=True)
    ap.add_argument('--interval-ms',type=int,default=10)
    ap.add_argument('--duration-s',type=float,default=75)
    a=ap.parse_args()
    if not (5<=a.interval_ms<=1000 and 1<=a.duration_s<=120):
        raise SystemExit('SAMPLE_ARGUMENT_OUT_OF_RANGE')
    signal.signal(signal.SIGTERM,handle_signal)
    signal.signal(signal.SIGINT,handle_signal)
    end=time.monotonic()+a.duration_s
    nxt=time.monotonic()
    found={}; last_scan=0.; overruns=0
    with open(a.out,'w',encoding='utf-8',buffering=1048576) as out:
        while not STOP and time.monotonic()<end:
            started=time.monotonic_ns()
            now=time.monotonic()
            if now-last_scan>.5:
                found=discover();last_scan=now
            procs=[]
            for pid,label in found.items():
                try:procs.append(metrics(pid,label))
                except (FileNotFoundError, PermissionError, ProcessLookupError):continue
            out.write(json.dumps({'steady_ns':started,'wall_ns':time.time_ns(),
                'sample_overruns':overruns,'processes':procs},separators=(',',':'))+'\n')
            nxt+=a.interval_ms/1000.
            delay=nxt-time.monotonic()
            if delay>0:time.sleep(delay)
            else:
                overruns+=1
                nxt=time.monotonic()
    print('SCHED_SAMPLER_STOPPED')
if __name__=='__main__':main()
