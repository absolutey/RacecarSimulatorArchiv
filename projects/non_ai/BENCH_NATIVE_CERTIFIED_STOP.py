#!/usr/bin/env python3
"""Known-safe >=60-step full stop benchmark, not a hard RT certificate."""
import importlib.util
import json
import statistics
import sys
import time
from pathlib import Path
from dataclasses import replace
import yaml
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src/environment/f1tenth_racing_sim'))
from f1tenth_racing_sim.physics.vehicle import State
from f1tenth_racing_sim.safety.stopping_envelope import StoppingEnvelope
from f1tenth_racing_sim.safety.native_stop_proof import ExperimentalNativeStopProof
from f1tenth_racing_sim.world.map import OccupancyMap
if len(sys.argv)!=2:raise SystemExit('USAGE: bench library.so')
sim=ROOT/'src/environment/f1tenth_racing_sim'
cfg=yaml.safe_load((sim/'config/competition_environment.yaml').read_text())
occ=OccupancyMap(sim/cfg['map']['image_file'],cfg['map'])
stop=StoppingEnvelope(cfg['dynamics'],occ,cfg['collision']['footprint_relative_to_base_link'])
native=ExperimentalNativeStopProof(stop,sys.argv[1])
results=[]
for name in ('FIRST_P0_TICK_FAILURE_SNAPSHOT.json','FIRST_STOP_SNAPSHOT_508.json'):
  incident=json.loads((ROOT/name).read_text());raw=incident['last_state'];cmd=incident['intent']
  z=State(x=raw['px'],y=raw['py'],yaw=raw['yaw'],speed=raw['v'],vx=raw['vx'],vy=raw['vy'],
          yaw_rate=raw['omega'],acceleration=raw['a'],ax=raw['ax'],ay=raw['ay'],
          slip=raw['slip_angle'],requested_acceleration=raw['accel'],jerk=raw['jerk'],
          steering=raw['steer'],steering_rate=raw['steer_vel'])
  zz=replace(z);stop.dynamics.step(zz,cmd['steering_rad'],cmd['acceleration_mps2'],.025)
  original=stop.evaluate(zz,cmd['steering_rad'],cmd['acceleration_mps2'])
  compiled=native.evaluate(zz,cmd['steering_rad'],cmd['acceleration_mps2'])
  assert original.qualified and original.steps>=50 and original==compiled,(name,original,compiled)
  wall_python=[];wall_native=[]
  for _ in range(350):
      start=time.perf_counter_ns();stop.evaluate(zz,cmd['steering_rad'],cmd['acceleration_mps2']);wall_python.append(time.perf_counter_ns()-start)
      start=time.perf_counter_ns();native.evaluate(zz,cmd['steering_rad'],cmd['acceleration_mps2']);wall_native.append(time.perf_counter_ns()-start)
  def p99(values):return sorted(values)[int((len(values)-1)*.99)]/1000.
  results.append({'case':name,'qualified':original.qualified,'steps':original.steps,
      'python_median_us':round(statistics.median(wall_python)/1000.,3),
      'native_median_us':round(statistics.median(wall_native)/1000.,3),
      'python_p99_us':round(p99(wall_python),3),'native_p99_us':round(p99(wall_native),3),
      'speedup_median':round(statistics.median(wall_python)/statistics.median(wall_native),2)})
print(json.dumps(results,ensure_ascii=False,indent=2))
print('NATIVE_CERTIFIED_STOP_BENCH_PARITY_PASS; OFFLINE_SPEED_NOT_RT_WCET')
native.close()
