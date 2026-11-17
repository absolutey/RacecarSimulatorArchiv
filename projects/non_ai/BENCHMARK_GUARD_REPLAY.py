from pathlib import Path
import json,sys,time,statistics,hashlib,yaml,random
from dataclasses import replace
p=Path(__file__).resolve().parent/'src/environment/f1tenth_racing_sim';sys.path.insert(0,str(p))
from f1tenth_racing_sim.physics.vehicle import State
from f1tenth_racing_sim.safety.stopping_envelope import StoppingEnvelope
from f1tenth_racing_sim.world.map import OccupancyMap
j=json.loads((Path(__file__).resolve().parent/'GUARD_FAILED_CERTIFICATE_INPUT.json').read_text());cfg=yaml.safe_load((p/'config/competition_environment.yaml').read_text())
occ=OccupancyMap(p/cfg['map']['image_file'],cfg['map'])
env=StoppingEnvelope(cfg['dynamics'],occ,cfg['collision']['footprint_relative_to_base_link'],reaction_s=.25,uncertainty_padding_m=.03)
r=j['last_state'];cmd=j['intent'];state=State(x=r['px'],y=r['py'],yaw=r['yaw'],speed=r['v'],vx=r['vx'],vy=r['vy'],yaw_rate=r['omega'],acceleration=r['a'],ax=r['ax'],ay=r['ay'],slip=r['slip_angle'],requested_acceleration=r['accel'],jerk=r['jerk'],steering=r['steer'],steering_rate=r['steer_vel'])
def run():
 z=replace(state);env.dynamics.step(z,cmd['steering_rad'],cmd['acceleration_mps2'],.025);return env.evaluate(z,cmd['steering_rad'],cmd['acceleration_mps2'])
for _ in range(40):run()
a=[];c=[]
for _ in range(350):
 st=time.perf_counter_ns();ct=time.process_time_ns();res=run();a.append((time.perf_counter_ns()-st)/1000);c.append((time.process_time_ns()-ct)/1000)
def q(vals):
 vals=sorted(vals);return {'min':vals[0],'p50':statistics.median(vals),'p95':vals[int(.95*(len(vals)-1))],'p99':vals[int(.99*(len(vals)-1))],'max':vals[-1]}
print(json.dumps({'wall_us':q(a),'cpu_us':q(c),'result':vars(res),'scope':'offline-only same-host; NOT ROS worst case or qualification'}))
