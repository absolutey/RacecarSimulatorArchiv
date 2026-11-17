"""L2: trainable map-based MLP speed prior, optionally supervised by prior raceline CSV."""
from pathlib import Path
import numpy as np
import torch
from torch import nn

class NeuralPlanner(nn.Module):
    def __init__(self):
        super().__init__();self.net=nn.Sequential(nn.Linear(8,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,2))
    def forward(self,x):
        y=self.net(x);return torch.stack((torch.tanh(y[...,0]),20*torch.sigmoid(y[...,1])),dim=-1)

def map_features(course):
    xy=course.xy
    d=np.roll(xy,-1,axis=0)-xy
    angles=np.unwrap(np.arctan2(d[:,1],d[:,0]))
    ds=np.linalg.norm(d,axis=1)
    kappa=(np.roll(angles,-1)-angles)/np.maximum(ds,.1)
    left=np.nan_to_num(course.width_left,nan=0,posinf=0,neginf=0)
    right=np.nan_to_num(course.width_right,nan=0,posinf=0,neginf=0)
    return np.stack([np.clip(kappa,-1,1),np.clip(np.roll(kappa,-1),-1,1),
       np.clip(np.roll(kappa,-2),-1,1),np.clip(np.roll(kappa,-3),-1,1),
       np.clip(left/5,0,2),np.clip(right/5,0,2),np.sin(angles),np.cos(angles)],axis=1).astype('float32')

def teacher_targets(course):
    """Supervised path/speed teacher only when matching, finite, numeric reference exists; else HOLD."""
    candidates=list(course.yaml_path.parent.glob('*raceline.csv'))
    if len(candidates)!=1:raise ValueError('NO_UNIQUE_RACELINE_TEACHER')
    rows=[]
    for line in candidates[0].read_text(encoding='utf-8-sig').splitlines():
        if not line.strip() or line.lstrip().startswith('#'):continue
        try: r=[float(t.strip()) for t in line.split(';')]
        except ValueError:continue
        if len(r)>=6:rows.append(r)
    if len(rows)<8:raise ValueError('INVALID_TEACHER_RACELINE')
    r=np.asarray(rows)
    if not np.isfinite(r).all():raise ValueError('NONFINITE_TEACHER')
    # Associate nearest teacher with each centerline node; signed normal offset.
    xy=course.xy; diff=r[:,1:3][None,:,:]-xy[:,None,:]
    nearest=np.argmin((diff*diff).sum(-1),axis=1)
    tang=np.roll(xy,-1,axis=0)-xy;unit=tang/np.linalg.norm(tang,axis=1)[:,None]
    displacement=r[nearest,1:3]-xy
    offset=unit[:,0]*displacement[:,1]-unit[:,1]*displacement[:,0]
    return np.stack([np.clip(offset,-1,1),np.clip(r[nearest,5],0,20)],axis=1).astype('float32')

def fit_planner(courses,out,epochs=40,seed=202,device='cpu'):
    torch.manual_seed(seed);features=[];targets=[];used=[];held={}
    for course in courses:
        try:
            x=map_features(course);t=teacher_targets(course)
            features.append(x);targets.append(t);used.append(course.name)
        except ValueError as e:held[course.name]=str(e)
    if not features:raise ValueError('NO_VALID_TEACHER_DATA')
    X=torch.from_numpy(np.concatenate(features)).to(device);Y=torch.from_numpy(np.concatenate(targets)).to(device)
    net=NeuralPlanner().to(device);optim=torch.optim.Adam(net.parameters(),lr=1e-3)
    for i in range(int(epochs)):
        pred=net(X);loss=torch.mean(((pred[:,0]-Y[:,0])*.5)**2+((pred[:,1]-Y[:,1])/20)**2)
        optim.zero_grad();loss.backward();optim.step()
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    torch.save({'state_dict':net.cpu().state_dict(),'seed':seed,'maps_used':used,'teacher_type':'REFERENCE_RACELINE_NOT_PHYSICS_CERTIFIED'},out)
    return {'loss':float(loss.detach()),'maps_used':used,'held':held,'certified':False}

@torch.no_grad()
def speed_hint(course,checkpoint=None,default=2.0):
    if checkpoint is None:return float(default)
    net=NeuralPlanner();state=torch.load(checkpoint,map_location='cpu',weights_only=True)
    net.load_state_dict(state['state_dict']);net.eval()
    return float(np.clip(net(torch.from_numpy(map_features(course)[:1]))[0,1].item(),0,20))
