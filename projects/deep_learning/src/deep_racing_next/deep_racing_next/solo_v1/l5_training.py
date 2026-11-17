"""L5: start-grid episodes, unwrapped route progress, time/collision reward and PPO update."""
from __future__ import annotations
from pathlib import Path
import hashlib,json,math,random,time
import numpy as np
import torch
import torch.nn.functional as F
from .l1_observation import build_features
from .l3_driver import ActorCritic
from .l4_physics import NonPhysicalTestDouble,F110GymBackend

class EpisodeGate:
    def __init__(self,course,dt,max_steps,collision_tol=.0):
        self.course=course;self.dt=float(dt);self.max_steps=int(max_steps);self.start=None
        self.s_prev=None;self.progress=0.;self.steps=0;self.finished=False
    def start_pose(self):
        x,y,yaw=self.course.frame(0);self.start=(x,y,yaw)
        self.s_prev=0.;self.progress=0.;self.steps=0;self.finished=False
        return [x,y,yaw]
    def evaluate(self,obs,env_done=False):
        self.steps+=1
        if self.start is None:raise ValueError('GATE_NOT_INITIALIZED')
        s,offset,index=self.course.projection(obs['x'],obs['y']); L=self.course.total_m
        ds=(s-self.s_prev+L/2)%L-L/2
        # Disallow teleportation / backwards exploitation (one step physically plausible distance).
        allowed=20.0*self.dt*2+0.35
        bad_jump=abs(ds)>allowed
        advance=0. if bad_jump else ds
        self.progress+=advance
        self.s_prev=s
        collision=bool(obs['collision']); maxed=self.steps>=self.max_steps
        # Must traverse >=~full-lap and be inside defined start box with same heading.
        near=np.linalg.norm(np.array([obs['x'],obs['y']])-np.array(self.start[:2]))<0.8
        direction=abs(math.atan2(math.sin(obs['yaw']-self.start[2]),math.cos(obs['yaw']-self.start[2])))<math.pi/3
        finished=(self.progress>=.98*L and near and direction and not collision and not bad_jump)
        self.finished=finished
        done=bool(finished or collision or env_done or maxed or bad_jump)
        reason=('COLLISION' if collision else 'INVALID_TELEPORT' if bad_jump else
                'FINISH' if finished else 'SIM_DONE' if env_done else 'TIMEOUT' if maxed else 'RUNNING')
        reward=float(2.0*advance-max(self.dt,0)*0.2-(70. if collision else 0.)+(100. if finished else 0.))
        return reward,done,{'reason':reason,'progress_m':float(self.progress),'lap_valid':bool(finished),
              'steps':self.steps,'lap_time_s':round(self.steps*self.dt,5) if finished else None,
              's_m':s,'lateral_offset_m':float(offset),'maxed':maxed}

class SoloEnv:
    def __init__(self,course,backend='gym',dt=.01,max_steps=5000,planner_checkpoint=None):
        self.course=course;self.dt=float(dt)
        if backend=='gym':self.sim=F110GymBackend(course,dt)
        elif backend=='original_pacejka':
            from .l4_original_pacejka import OriginalPacejkaBackend
            self.sim=OriginalPacejkaBackend(course,dt)
        elif backend=='test_double':self.sim=NonPhysicalTestDouble(course,dt)
        else:raise ValueError('UNRECOGNIZED_BACKEND')
        self.gate=EpisodeGate(course,dt,max_steps)
        self.planner_outputs=None
        if planner_checkpoint is not None:
            from .l2_planner import NeuralPlanner,map_features
            net=NeuralPlanner();raw=torch.load(planner_checkpoint,map_location='cpu',weights_only=True)
            net.load_state_dict(raw['state_dict']);net.eval()
            with torch.no_grad(): self.planner_outputs=net(torch.as_tensor(map_features(course))).numpy()
        self.physics_label=self.sim.qualified_physics
    def reset(self):
        pose=self.gate.start_pose();obs=self.sim.reset(pose)
        # Verify real backend's initial zero speed contract, not inferred from reset input.
        if abs(obs['v'])>1e-4:raise RuntimeError('START_SPEED_NONZERO')
        self.last_obs=obs
        return self._features(obs,0.)
    def step(self,action):
        obs,sim_done,extra=self.sim.step(action)
        reward,done,info=self.gate.evaluate(obs,sim_done)
        info['sim_physics']=self.physics_label
        feat=self._features(obs,max(self.gate.progress,0))
        self.last_obs=obs
        return feat,reward,done,info
    def _features(self,obs,progress):
        if self.planner_outputs is None:return build_features(obs,self.course,progress,2.)
        _,_,j=self.course.projection(obs['x'],obs['y'])
        offset,speed=self.planner_outputs[j]
        return build_features(obs,self.course,progress,float(speed),float(offset))
    def close(self):self.sim.close()

def _sha(file):
    h=hashlib.sha256();
    with open(file,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

class PPOTrainer:
    def __init__(self,seed=202,device='cpu',lr=3e-4):
        torch.set_num_threads(min(4,torch.get_num_threads()))
        random.seed(seed);np.random.seed(seed);torch.manual_seed(seed)
        self.device=device;self.net=ActorCritic().to(device)
        self.optim=torch.optim.Adam(self.net.parameters(),lr=lr)
        self.seed=seed;self.update_count=0
    def collect(self,env,steps=512):
        states=[];actions_pre=[];logps=[];values=[];rews=[];ends=[]
        episodes=[];feat=env.reset();episode_reward=0.
        for _ in range(steps):
            t=torch.tensor(feat,dtype=torch.float32,device=self.device)[None,:]
            with torch.no_grad():a,pre,lp,v=self.net.sample(t)
            action=a.squeeze(0).cpu().numpy()
            nxt,reward,done,info=env.step(action)
            states.append(feat);actions_pre.append(pre.squeeze(0).cpu().numpy());logps.append(float(lp.item()));values.append(float(v.item()))
            rews.append(float(reward));ends.append(bool(done))
            episode_reward+=reward
            if done:
                episodes.append({'reward':float(episode_reward),**info});feat=env.reset();episode_reward=0.
            else:feat=nxt
        with torch.no_grad():
            _,tail_value=self.net.distribution(torch.tensor(feat,dtype=torch.float32,device=self.device)[None,:]);tail=float(tail_value.item())
        return {'obs':np.array(states,dtype='float32'),'pre':np.array(actions_pre,dtype='float32'),
                'logp':np.array(logps,dtype='float32'),'values':np.array(values,dtype='float32'),
                'rew':np.array(rews,dtype='float32'),'done':np.array(ends,dtype='bool'),
                'tail':tail,'episodes':episodes}
    def update(self,batch,epochs=4,minibatch=128,gamma=.99,gae_lambda=.95,clip=.2):
        n=len(batch['rew']);adv=np.zeros(n,dtype='float32');ret=np.zeros(n,dtype='float32');last=0.
        for i in reversed(range(n)):
            nonterminal=0. if batch['done'][i] else 1.
            nextv=batch['values'][i+1] if i<n-1 else batch['tail']
            delta=batch['rew'][i]+gamma*nonterminal*nextv-batch['values'][i]
            last=delta+gamma*gae_lambda*nonterminal*last;adv[i]=last
        ret=adv+batch['values']
        A=torch.tensor(adv,device=self.device);A=(A-A.mean())/(A.std(unbiased=False)+1e-8)
        R=torch.tensor(ret,device=self.device)
        X=torch.tensor(batch['obs'],device=self.device);P=torch.tensor(batch['pre'],device=self.device)
        OLD=torch.tensor(batch['logp'],device=self.device)
        last_loss=0.
        for _ in range(int(epochs)):
            for ix in torch.randperm(n,device=self.device).split(minibatch):
                lp,v,ent=self.net.evaluate(X[ix],P[ix]);rat=torch.exp(lp-OLD[ix])
                policy_loss=-torch.minimum(rat*A[ix],torch.clamp(rat,1-clip,1+clip)*A[ix]).mean()
                value_loss=F.mse_loss(v,R[ix])
                loss=policy_loss+.5*value_loss-.003*ent.mean()
                if not torch.isfinite(loss):raise RuntimeError('NONFINITE_LOSS')
                self.optim.zero_grad();loss.backward();torch.nn.utils.clip_grad_norm_(self.net.parameters(),.5);self.optim.step()
                last_loss=float(loss.detach())
        self.update_count+=1
        return {'ppo_loss':last_loss,'updates':self.update_count,'mean_reward_per_step':float(np.mean(batch['rew'])),
                'finished_episodes':sum(ep['lap_valid'] for ep in batch['episodes']),
                'terminated_episodes':len(batch['episodes'])}
    def checkpoint(self,path,course,physics_label,metrics):
        p=Path(path);p.parent.mkdir(parents=True,exist_ok=True)
        torch.save({'model':self.net.cpu().state_dict(),'optimizer':self.optim.state_dict(),'updates':self.update_count,
                    'seed':self.seed,'map_identity':course.fingerprint,'physics':physics_label,
                    'qualified_for_ros_actuation':False,'metrics':metrics},p)
        self.net.to(self.device)
        return {'checkpoint':str(p),'sha256':_sha(p),'actuation_certified':False}
    def load(self,path,expected_map=None):
        # PyTorch >=2.6 weights_only, no arbitrary pickle execution.
        raw=torch.load(path,map_location=self.device,weights_only=True)
        if expected_map is not None and raw['map_identity']!=expected_map:raise RuntimeError('CHECKPOINT_MAP_MISMATCH')
        self.net.load_state_dict(raw['model']);self.optim.load_state_dict(raw['optimizer']);self.update_count=int(raw['updates'])
        return raw

def run_train(course,backend='gym',steps=1024,updates=4,dt=.01,out=None,seed=202,max_steps=4500,planner_checkpoint=None):
    if out is None:raise ValueError('OUTPUT_REQUIRED')
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    env=SoloEnv(course,backend=backend,dt=dt,max_steps=max_steps,planner_checkpoint=planner_checkpoint)
    trainer=PPOTrainer(seed=seed)
    summary={'map':course.name,'map_identity':course.fingerprint,'backend':env.physics_label,
             'physics_certified':False,'source_fingerprints':getattr(env.sim,'fingerprints',None),
             'r_a_complete':False,'racing_complete':False,
             'episodes_start_at_rest':True,'speed_cap_mps':20.0,'planner_checkpoint':str(planner_checkpoint) if planner_checkpoint else None,'updates':[]}
    try:
        for i in range(int(updates)):
            batch=trainer.collect(env,int(steps));record=trainer.update(batch)
            record['episode_results']=batch['episodes'];summary['updates'].append(record)
            ck=trainer.checkpoint(out/'checkpoints'/f'ppo_update_{i+1:04}.pt',course,env.physics_label,record)
            summary['last_checkpoint']=ck
            (out/'RUN_SUMMARY.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False,default=str))
        return summary
    finally:env.close()

@torch.no_grad()
def run_eval(course,checkpoint,backend='gym',episodes=2,dt=.01,max_steps=4500,planner_checkpoint=None):
    env=SoloEnv(course,backend=backend,dt=dt,max_steps=max_steps,planner_checkpoint=planner_checkpoint)
    trainer=PPOTrainer();raw=trainer.load(checkpoint)
    results=[]
    try:
        for i in range(int(episodes)):
            state=env.reset()
            for t in range(max_steps):
                a=trainer.net.deterministic(torch.as_tensor(state,dtype=torch.float32)[None,:])[0].numpy()
                state,rew,done,info=env.step(a)
                if done:break
            results.append(info)
        return {'map':course.name,'results':results,'physics':env.physics_label,'trained_map_identity':raw['map_identity'],'zero_shot_map':raw['map_identity'] != course.fingerprint,
                'lap_times_s':[e['lap_time_s'] for e in results if e['lap_valid']],
                'racing_certified':False}
    finally:env.close()
