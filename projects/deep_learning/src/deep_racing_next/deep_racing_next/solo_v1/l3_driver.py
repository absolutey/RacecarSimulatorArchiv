"""L3: PPO tanh-normal stochastic MLP driver; proposals only."""
import math
import numpy as np
import torch
from torch import nn
from torch.distributions import Normal
from .l1_observation import FEATURE_COUNT

class ActorCritic(nn.Module):
    def __init__(self):
        super().__init__()
        self.backbone=nn.Sequential(nn.Linear(FEATURE_COUNT,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh())
        self.actor=nn.Linear(64,2)
        self.critic=nn.Linear(64,1)
        self.log_std=nn.Parameter(torch.full((2,),-0.6))
    def distribution(self,obs):
        h=self.backbone(obs)
        return Normal(self.actor(h),torch.exp(self.log_std).expand_as(self.actor(h))),self.critic(h).squeeze(-1)
    def sample(self,obs):
        d,v=self.distribution(obs);pre=d.rsample(); a=torch.tanh(pre)
        logp=(d.log_prob(pre)-torch.log(1-a*a+1e-6)).sum(-1)
        return a,pre,logp,v
    def evaluate(self,obs,pre):
        d,v=self.distribution(obs);a=torch.tanh(pre)
        logp=(d.log_prob(pre)-torch.log(1-a*a+1e-6)).sum(-1)
        return logp,v,d.entropy().sum(-1)
    @torch.no_grad()
    def deterministic(self,obs):
        d,_=self.distribution(obs)
        return torch.tanh(d.mean)

def action_from_normalized(action,steering_max=0.4,target_speed_max=20):
    a=np.asarray(action,dtype=float)
    if a.shape!=(2,) or not np.isfinite(a).all() or np.any(np.abs(a)>1.0001):raise ValueError('ACTION_INVALID')
    return float(np.clip(a[0],-1,1)*steering_max),float((np.clip(a[1],-1,1)+1)*.5*target_speed_max)
