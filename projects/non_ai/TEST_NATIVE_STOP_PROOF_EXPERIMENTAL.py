#!/usr/bin/env python3
"""Offline native-vs-original stop proof parity; no ROS actuation or enabling."""
import json
import hashlib
import math
import random
import statistics
import sys
import time
from dataclasses import replace
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src/environment/f1tenth_racing_sim'))
from f1tenth_racing_sim.physics.vehicle import State
from f1tenth_racing_sim.safety.stopping_envelope import StoppingEnvelope
from f1tenth_racing_sim.safety.native_stop_proof import ExperimentalNativeStopProof
from f1tenth_racing_sim.world.map import OccupancyMap

if len(sys.argv)!=2:raise SystemExit('USAGE: script /path/to/experimental.so')
art=json.loads((ROOT/'GUARD_FAILED_CERTIFICATE_INPUT.json').read_text())
cfg=yaml.safe_load((ROOT/'src/environment/f1tenth_racing_sim/config/competition_environment.yaml').read_text())
map_path=ROOT/'src/environment/f1tenth_racing_sim'/cfg['map']['image_file']
assert hashlib.sha256(map_path.read_bytes()).hexdigest()==art['identity']['map_sha256']
occ=OccupancyMap(map_path,cfg['map'])
foot=cfg['collision']['footprint_relative_to_base_link']
py=StoppingEnvelope(cfg['dynamics'],occ,foot,reaction_s=.25,uncertainty_padding_m=.03)
native=ExperimentalNativeStopProof(py,sys.argv[1]);r=art['last_state'];m=art['intent']
def state(**override):
    vals=dict(x=r['px'],y=r['py'],yaw=r['yaw'],speed=r['v'],vx=r['vx'],vy=r['vy'],
              yaw_rate=r['omega'],acceleration=r['a'],ax=r['ax'],ay=r['ay'],
              slip=r['slip_angle'],requested_acceleration=r['accel'],jerk=r['jerk'],
              steering=r['steer'],steering_rate=r['steer_vel'])
    vals.update(override)
    return State(**vals)

cases=[]
z=state();z1=replace(z);py.dynamics.step(z1,m['steering_rad'],m['acceleration_mps2'],.025)
cases.append(('original_live',z1,m['steering_rad'],m['acceleration_mps2']))
rng=random.Random(418170)
for i in range(175):
    dx=rng.uniform(-6,6);dy=rng.uniform(-6,6);speed=rng.uniform(-5,19.9)
    z=state(x=r['px']+dx,y=r['py']+dy,speed=speed,vx=speed,
        yaw=rng.uniform(-3.14,3.14),slip=rng.uniform(-.3,.3),
        steering=rng.uniform(-.2,.2),requested_acceleration=rng.uniform(-3,3),
        yaw_rate=rng.uniform(-2,2))
    steer=rng.uniform(-.25,.25);acc=rng.uniform(-2,5)
    z1=replace(z)
    try:py.dynamics.step(z1,steer,acc,.025)
    except (ArithmeticError,ValueError,OverflowError):continue
    cases.append((f'golden{i:03}',z1,steer,acc))

# Track-specific physically plausible poses ensure full 60+ step stop rollout,
# not only immediate off-map or slip rejection (which would mislead timing).
import csv
center=ROOT/'src/environment/f1tenth_racing_sim/maps/iccas2025/iccas2025_centerline.csv'
with center.open() as fh:
    rows=list(csv.DictReader(fh))
for i in range(550):
    point=rows[i%len(rows)]
    next_point=rows[(i+1)%len(rows)]
    x=float(point['x_m']);y=float(point['y_m'])
    yaw=math.atan2(float(next_point['y_m'])-y,float(next_point['x_m'])-x)
    speed=rng.uniform(0.0,8.0)
    offset=rng.uniform(-.4,.4)
    yaw+=rng.uniform(-.12,.12)
    st=state(x=x+offset*math.cos(yaw+math.pi/2),
             y=y+offset*math.sin(yaw+math.pi/2),
             yaw=yaw,speed=speed,vx=speed,vy=0.0,
             slip=rng.uniform(-.05,.05),yaw_rate=rng.uniform(-.2,.2),
             steering=rng.uniform(-.08,.08),
             requested_acceleration=rng.uniform(-1.0,3.0))
    cases.append((f'track{i:03}',st,rng.uniform(-.12,.12),rng.uniform(-2.0,4.0)))

# Additional near-boundary and high velocity cases to stress state transitions.
for i in range(450):
    speed=rng.uniform(-19.9,19.9)
    st=state(x=r['px']+rng.uniform(-4,4),y=r['py']+rng.uniform(-4,4),
             speed=speed,vx=speed*math.cos(rng.uniform(-.2,.2)),
             vy=speed*math.sin(rng.uniform(-.2,.2)),
             yaw=rng.uniform(-math.pi,math.pi),slip=rng.uniform(-.34,.34),
             yaw_rate=rng.uniform(-2,2),steering=rng.uniform(-.2,.2),
             requested_acceleration=rng.uniform(-4,4))
    cases.append((f'stress{i:03}',st,rng.uniform(-.3,.3),rng.uniform(-3,6)))

numerical_fields=('steering_rad','stopping_time_s','stopping_distance_m',
                  'max_abs_slip_rad','max_lateral_accel_mps2')
py_elapsed=[];native_elapsed=[];mismatch=[]
for key,s,a,b in cases:
    start=time.perf_counter_ns();old=py.evaluate(s,a,b);py_elapsed.append(time.perf_counter_ns()-start)
    start=time.perf_counter_ns();new=native.evaluate(s,a,b);native_elapsed.append(time.perf_counter_ns()-start)
    if (old.qualified!=new.qualified or old.reason!=new.reason or old.steps!=new.steps):
        mismatch.append((key,'DECISION',vars(old),vars(new)))
        continue
    for field in numerical_fields:
        v=getattr(old,field);w=getattr(new,field)
        if not math.isclose(v,w,rel_tol=2e-9,abs_tol=2e-8):
            mismatch.append((key,field,v,w))
            break

# Native deadline must fail closed; never approve when time is already over.
expired=0
for key,s,a,b in cases[:30]:
    result=native.evaluate(s,a,b,deadline_ns=time.perf_counter_ns()-1)
    assert not result.qualified and result.reason=='PROOF_BUDGET_UNPROVEN'
    expired+=1
native.close()
summary={'cases':len(cases),'mismatches':len(mismatch),'native_expired_deadline_rejected':expired,
         'python_median_us':round(statistics.median(py_elapsed)/1000,3),
         'native_median_us':round(statistics.median(native_elapsed)/1000,3),
         'examples':mismatch[:10], 'status':'PARITY_PASS' if not mismatch else 'PARITY_FAIL'}
print(json.dumps(summary,indent=2,ensure_ascii=False))
if mismatch:raise SystemExit(1)
print('EXPERIMENTAL_NATIVE_PARITY_PASS; NOT_DEPLOYMENT_QUALIFIED; ROS_UNCHANGED')
