"""L4: strict F110 Gym adapter (real) and explicitly NON-PHYSICAL unit-test double."""
import math
import numpy as np
from .l3_driver import action_from_normalized

class PhysicsUnavailable(RuntimeError):pass

class SpeedLimiter:
    def __init__(self,dt=0.01,accel=6.0,brake=10.0,vmax=20.0):
        self.dt=float(dt);self.a=float(accel);self.b=float(brake);self.vmax=float(vmax);self.last=0.0
    def reset(self):self.last=0.0
    def apply(self,steer,target):
        if not math.isfinite(steer) or not math.isfinite(target):raise ValueError('NONFINITE_CONTROL')
        target=float(np.clip(target,0,min(self.vmax,20.0)))
        bounded=np.clip(target,self.last-self.b*self.dt,self.last+self.a*self.dt)
        self.last=float(np.clip(bounded,0,min(self.vmax,20.0)))
        return float(steer),self.last

class F110GymBackend:
    """No fake original-Pacejka claim. Uses installed f110_gym and its reported dynamics."""
    qualified_physics='F110_GYM_INSTALLED_DYNAMICS_NOT_ORIGINAL_PACEJKA_CERTIFIED'
    def __init__(self,course,dt=.01):
        try:import gym
        except ImportError as e:raise PhysicsUnavailable('INSTALL_COMPATIBLE_GYM_AND_F110_GYM_FIRST') from e
        try: import f110_gym  # noqa
        except ImportError as e:raise PhysicsUnavailable('F110_GYM_NOT_INSTALLED') from e
        # f110 gym expects base w/o ext and matching YAML and image stems.
        yp=course.yaml_path;img=course.image_path
        if yp.stem!=img.stem:raise PhysicsUnavailable('F110_GYM_REQUIRES_MATCHING_IMAGE_YAML_STEMS')
        try:self.env=gym.make('f110_gym:f110-v0',map=str(yp.with_suffix('')),map_ext=img.suffix,num_agents=1,timestep=dt)
        except Exception as e:raise PhysicsUnavailable(f'F110_GYM_CREATE_FAILED:{e}') from e
        self.dt=dt;self.limiter=SpeedLimiter(dt=dt);self.last_obs=None
    def reset(self,pose):
        self.limiter.reset()
        raw=self.env.reset(np.array([pose],dtype=float))
        return self._decode(raw[0] if isinstance(raw,tuple) else raw)
    def step(self,action):
        steer,target=action_from_normalized(action)
        steer,v=self.limiter.apply(steer,target)
        raw=self.env.step(np.array([[steer,v]],dtype=float))
        if len(raw)==4:obs,rew,done,info=raw;truncated=False
        elif len(raw)==5:obs,rew,done,truncated,info=raw
        else:raise PhysicsUnavailable('F110_GYM_STEP_SIGNATURE_UNKNOWN')
        state=self._decode(obs)
        return state,bool(done or truncated),info
    def _decode(self,raw):
        idx=int(raw.get('ego_idx',0));scan=np.asarray(raw['scans'][idx],dtype=float)
        obs={'x':float(raw['poses_x'][idx]),'y':float(raw['poses_y'][idx]),'yaw':float(raw['poses_theta'][idx]),
             'v':float(raw['linear_vels_x'][idx]),'yaw_rate':float(raw.get('ang_vels_z',[0])[idx]),
             'scan':scan,'collision':bool(raw['collisions'][idx]), 'sim_label':self.qualified_physics}
        if any(not math.isfinite(obs[k]) for k in ('x','y','yaw','v','yaw_rate')):raise PhysicsUnavailable('SIM_NONFINITE')
        if obs['v']>20.0001:raise PhysicsUnavailable('SPEED_LIMIT_BREACH')
        self.last_obs=obs;return obs
    def close(self):self.env.close()

class NonPhysicalTestDouble:
    """For integration unit tests ONLY. Cannot earn physics or racing qualification."""
    qualified_physics='TEST_DOUBLE_NO_PHYSICS_CERTIFICATION'
    def __init__(self,course,dt=.05):self.course=course;self.dt=dt;self.limiter=SpeedLimiter(dt=dt)
    def reset(self,pose):
        self.pose=np.asarray(pose,dtype=float);self.v=0.;self.limiter.reset()
        return self._observe()
    def step(self,action):
        steer,target=action_from_normalized(action)
        steer,goal=self.limiter.apply(steer,target)
        self.v=float(goal);self.pose[2]+=self.v*math.tan(steer)/.34*self.dt
        self.pose[:2]+=self.v*self.dt*np.array([math.cos(self.pose[2]),math.sin(self.pose[2])]);return self._observe(),False,{}
    def _observe(self):
        return {'x':float(self.pose[0]),'y':float(self.pose[1]),'yaw':float(self.pose[2]),'v':self.v,
                'yaw_rate':0.,'scan':np.full(108,10.0),'collision':False,'sim_label':self.qualified_physics}
    def close(self):pass
