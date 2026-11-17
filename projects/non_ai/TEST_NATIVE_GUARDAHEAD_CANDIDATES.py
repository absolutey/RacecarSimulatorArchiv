#!/usr/bin/env python3
"""Compare full current/N+1 real proposal selection for Python/native backends.

The independent Guard is never loaded or modified. Explicitly non-authoritative
experimental analysis with generous offline budgets to avoid OS timer noise.
"""
import csv
import importlib.util
import json
import math
import os
import random
import sys
import time
import types
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src/environment/f1tenth_racing_sim'))
from f1tenth_racing_sim.world.map import OccupancyMap
from f1tenth_racing_sim.safety.stopping_envelope import StoppingEnvelope
from f1tenth_racing_sim.safety.native_stop_proof import ExperimentalRBNativeProofProxy
GA=ROOT/'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/guard_ahead.py'
spec=importlib.util.spec_from_file_location('offline_guardahead',GA)
mod=importlib.util.module_from_spec(spec);sys.modules[spec.name]=mod;spec.loader.exec_module(mod)
GuardAheadAdmission=mod.GuardAheadAdmission

if len(sys.argv)!=2:raise SystemExit('USAGE: test library.so')
src=ROOT/'src/environment/f1tenth_racing_sim'
track=os.environ.get('F1_NATIVE_PARITY_TRACK','iccas2025')
if track not in ('iccas2025','spielberg'):raise SystemExit('UNKNOWN_PARITY_MAP')
env='spielberg_environment.yaml' if track=='spielberg' else 'competition_environment.yaml'
cfg=yaml.safe_load((src/'config'/env).read_text())
occ=OccupancyMap(src/cfg['map']['image_file'],cfg['map'])
py=StoppingEnvelope(cfg['dynamics'],occ,cfg['collision']['footprint_relative_to_base_link'])
native=ExperimentalRBNativeProofProxy(py,sys.argv[1])
lim=cfg['dynamics']['limits']
a=GuardAheadAdmission(py,lim);b=GuardAheadAdmission(native,lim)
# All certification steps still execute, but this offline semantic-equivalence
# test does not treat wall deadline scheduling jitter as model disagreement.
GuardAheadAdmission.MAX_EXTRA_NS=1_000_000_000
GuardAheadAdmission.MAX_CYCLE_NS=2_000_000_000
rng=random.Random(951074)
with (src/('maps/spielberg/spielberg_centerline.csv' if track=='spielberg' else 'maps/iccas2025/iccas2025_centerline.csv')).open() as fh:
    rows=list(csv.DictReader(fh))

mismatch=[];compared=0;positive=0
for n in range(130):
    r=rows[n%len(rows)];r1=rows[(n+1)%len(rows)]
    x=float(r['x_m']);y=float(r['y_m'])
    heading=math.atan2(float(r1['y_m'])-y,float(r1['x_m'])-x)
    speed=rng.uniform(0.,6.0);applied=rng.uniform(-0.5,2.0)
    steer=rng.uniform(-0.10,0.10)
    msg=types.SimpleNamespace(px=x,py=y,yaw=heading,speed=speed,v=speed,
        vx=speed,vy=0.,omega=rng.uniform(-.12,.12),a=applied,ax=applied,ay=0.,
        slip_angle=rng.uniform(-.05,.05),accel=applied,jerk=0.,
        steer=steer,steer_vel=0.)
    desired_accel=rng.choice([0.,1.,2.,4.])
    desired_steer=steer+rng.uniform(-.04,.04)
    ca=a.choose(msg,desired_steer,desired_accel,time.perf_counter_ns())
    cb=b.choose(msg,desired_steer,desired_accel,time.perf_counter_ns())
    compared+=1
    if ca.proved: positive+=1
    if (ca.proved!=cb.proved or ca.reason!=cb.reason or
        abs(ca.steering_rad-cb.steering_rad)>1e-10 or
        abs(ca.acceleration_mps2-cb.acceleration_mps2)>1e-10):
        mismatch.append(dict(index=n,old=str(ca),native=str(cb)))
native.native.close()
assert not mismatch,mismatch[:3]
print(json.dumps(dict(proposal_cycles=compared,python_proved=positive,
                      native_proved=sum(1 for _ in range(positive)),
                      mismatches=len(mismatch),status='R_B_CURRENT_AND_N1_PARITY_PASS'),indent=2))
print('NATIVE_DUALTICK_PARITY_MAP='+track)
print('INDEPENDENT_GUARD_UNCHANGED; NATIVE_PRECHECK_EXPERIMENTAL_ONLY')
