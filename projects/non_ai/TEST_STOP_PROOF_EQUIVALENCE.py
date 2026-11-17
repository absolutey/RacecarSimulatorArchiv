#!/usr/bin/env python3
import sys,json,random,yaml,hashlib,math
from pathlib import Path
from dataclasses import replace
root=Path(__file__).resolve().parent; sys.path.insert(0,str(root/'src/environment/f1tenth_racing_sim'))
from f1tenth_racing_sim.physics.vehicle import State
from f1tenth_racing_sim.safety.stopping_envelope import StoppingEnvelope
from f1tenth_racing_sim.world.swept_collision import swept_footprint_collision,footprint_collision
from f1tenth_racing_sim.world.map import OccupancyMap
R=json.loads((root/'GUARD_FAILED_CERTIFICATE_INPUT.json').read_text()); cfg=yaml.safe_load((root/'src/environment/f1tenth_racing_sim/config/competition_environment.yaml').read_text())
occ=OccupancyMap(root/'src/environment/f1tenth_racing_sim'/cfg['map']['image_file'],cfg['map'])
assert hashlib.sha256((root/'src/environment/f1tenth_racing_sim'/cfg['map']['image_file']).read_bytes()).hexdigest()==R['identity']['map_sha256']
f=cfg['collision']['footprint_relative_to_base_link'];stop=StoppingEnvelope(cfg['dynamics'],occ,f,reaction_s=.25,uncertainty_padding_m=.03)
r=R['last_state'];m=R['intent']
def state(**override):
 vals=dict(x=r['px'],y=r['py'],yaw=r['yaw'],speed=r['v'],vx=r['vx'],vy=r['vy'],yaw_rate=r['omega'],acceleration=r['a'],ax=r['ax'],ay=r['ay'],slip=r['slip_angle'],requested_acceleration=r['accel'],jerk=r['jerk'],steering=r['steer'],steering_rate=r['steer_vel']);vals.update(override);return State(**vals)
res={}; z=state();z1=replace(z);stop.dynamics.step(z1,m['steering_rad'],m['acceleration_mps2'],.025);res['live591']=vars(stop.evaluate(z1,m['steering_rad'],m['acceleration_mps2']))
rng=random.Random(418170)
for i in range(175):
 dx=rng.uniform(-6,6);dy=rng.uniform(-6,6);speed=rng.uniform(-5,19.9)
 z=state(x=r['px']+dx,y=r['py']+dy,speed=speed,vx=speed,yaw=rng.uniform(-3.14,3.14),slip=rng.uniform(-.3,.3),steering=rng.uniform(-.2,.2),requested_acceleration=rng.uniform(-3,3),yaw_rate=rng.uniform(-2,2))
 steer=rng.uniform(-.25,.25);acc=rng.uniform(-2,5)
 z1=replace(z)
 try:
  stop.dynamics.step(z1,steer,acc,.025)
  cert=stop.evaluate(z1,steer,acc)
  res['stop%03d'%i]=vars(cert)
 except (ArithmeticError,ValueError,OverflowError) as e:res['stop%03d'%i]='ERROR:'+type(e).__name__
for i in range(330):
 x=rng.uniform(-7,20);y=rng.uniform(-7,20);s=state(x=x,y=y,yaw=rng.uniform(-math.pi,math.pi)); t=state(x=x+rng.uniform(-.25,.25),y=y+rng.uniform(-.25,.25),yaw=s.yaw+rng.uniform(-.3,.3))
 res['swept%03d'%i]=swept_footprint_collision(occ,s,t,f)
 res['foot%03d'%i]=footprint_collision(occ,s,f,.03)
# Rect check; tests exact O(1) full region predicate against baseline pixels.
for i in range(520):
 row0=rng.randrange(occ.height);row1=min(occ.height-1,row0+rng.randrange(12));col0=rng.randrange(occ.width);col1=min(occ.width-1,col0+rng.randrange(12))
 if hasattr(occ,'rectangle_is_free'):
  new=occ.rectangle_is_free(row0,row1,col0,col1)
  original=bool((occ.pixel_array[row0:row1+1,col0:col1+1]>=occ.threshold).all())
  assert new==original,('RECT_MISMATCH',row0,row1,col0,col1)
 res['rectangle%03d'%i]=bool((occ.pixel_array[row0:row1+1,col0:col1+1]>=occ.threshold).all())
expected=json.loads((root/'STOP_PROOF_GOLDEN_ORACLE.json').read_text())
assert res.keys()==expected.keys(), 'GOLDEN_CASE_SET_CHANGED'
for name, value in res.items():
    prior=expected[name]
    if isinstance(value,dict):
        for field,v in value.items():
            old=prior[field]
            if isinstance(v,float):
                assert math.isclose(v,old,abs_tol=1e-11,rel_tol=1e-12),(name,field,v,old)
            else:
                assert v==old,(name,field,v,old)
    else:
        assert value==prior,(name,value,prior)
print('STOP_PROOF_AND_SWEPT_COLLISION_GOLDEN_PASS',len(res), 'case outcomes unchanged')
