#!/usr/bin/env python3
"""Real (not stubbed) unchanged stop envelope replay for P0 intent #252.

No ROS/time qualification: this is solely off-line physics/map conditional stop proof.
"""
import json,sys,yaml,hashlib
from pathlib import Path
from dataclasses import replace
ROOT=Path(__file__).resolve().parent
SYS=ROOT/'src/environment/f1tenth_racing_sim'
sys.path.insert(0,str(SYS))
from f1tenth_racing_sim.physics.vehicle import State
from f1tenth_racing_sim.safety.stopping_envelope import StoppingEnvelope
from f1tenth_racing_sim.world.map import OccupancyMap
cfg=yaml.safe_load((SYS/'config/competition_environment.yaml').read_text())
s=json.loads((ROOT/'FIRST_P0_TICK_FAILURE_SNAPSHOT.json').read_text())
image=SYS/cfg['map']['image_file']
assert hashlib.sha256(image.read_bytes()).hexdigest()==s['identity']['map_sha256'],'MAP_SHA_CHANGED'
assert float(cfg['simulation']['physics_timestep_s'])==.025
occ=OccupancyMap(image,cfg['map'])
e=StoppingEnvelope(cfg['dynamics'],occ,cfg['collision']['footprint_relative_to_base_link'],reaction_s=.25,uncertainty_padding_m=.03)
r=s['last_state'];i=s['intent']
z=State(x=r['px'],y=r['py'],yaw=r['yaw'],speed=r['v'],vx=r['vx'],vy=r['vy'],yaw_rate=r['omega'],acceleration=r['a'],ax=r['ax'],ay=r['ay'],slip=r['slip_angle'],requested_acceleration=r['accel'],jerk=r['jerk'],steering=r['steer'],steering_rate=r['steer_vel'])
z1=replace(z)
e.dynamics.step(z1,i['steering_rad'],i['acceleration_mps2'],.025)
result=e.evaluate(z1,i['steering_rad'],i['acceleration_mps2'])
assert result.qualified and result.reason=='CONDITIONAL_STOP_ONLY'
assert result.steps==65
assert abs(result.stopping_time_s-1.625)<1e-10
assert abs(result.stopping_distance_m-2.403464209576472)<1e-10
print('P0_FAILED_INTENT_252_UNCHANGED_STOP_ENVELOPE_REPLAY_PASS',json.dumps(vars(result),sort_keys=True))
print('NOT_A_GUARD_DEADLINE_OR_RACING_QUALIFICATION')
