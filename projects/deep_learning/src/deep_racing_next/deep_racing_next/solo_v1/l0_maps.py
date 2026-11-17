"""L0: audit a ROS map and construct a finite closed polyline. Fail closed on missing topology."""
from __future__ import annotations
import csv, hashlib, math
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import yaml

class MapHold(RuntimeError): pass

def sha(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()

@dataclass
class Course:
    name:str
    yaml_path:Path
    image_path:Path
    centerline:np.ndarray
    arc:np.ndarray
    total_m:float
    width_left:np.ndarray
    width_right:np.ndarray
    fingerprint:dict
    @property
    def xy(self): return self.centerline[:, :2]
    def projection(self,x,y):
        """Project onto every polyline segment, return progress s, lateral offset and nearest segment index."""
        points=self.xy; a=points; b=np.roll(points,-1,axis=0)
        d=b-a; den=(d*d).sum(1)
        if np.any(den<1e-10): raise MapHold('DEGENERATE_ROUTE_SEGMENT')
        alpha=np.clip(((np.array([x,y])-a)*d).sum(1)/den,0,1)
        q=a+alpha[:,None]*d
        dist=((q-np.array([x,y]))**2).sum(1)
        j=int(np.argmin(dist)); dx,dy=d[j]
        cross=(dx*(y-q[j,1])-dy*(x-q[j,0]))/np.linalg.norm(d[j])
        return float(self.arc[j]+alpha[j]*np.linalg.norm(d[j])),float(cross),j
    def frame(self,index):
        i=index%len(self.xy)
        delta=self.xy[(i+1)%len(self.xy)]-self.xy[i]
        return float(self.xy[i,0]),float(self.xy[i,1]),float(math.atan2(delta[1],delta[0]))
    def lookahead(self,s,distance=1.5):
        tar=(s+distance)%self.total_m
        j=int(np.searchsorted(self.arc,tar,side='right')-1)%len(self.xy)
        end=(j+1)%len(self.xy)
        segment=self.arc[j+1]-self.arc[j]
        p=min(max((tar-self.arc[j])/max(segment,1e-8),0),1)
        point=(1-p)*self.xy[j]+p*self.xy[end]
        return np.asarray(point,dtype=np.float32), j

def _load_centerline(path):
    # Supports standard comma+header centerlines or numeric comma files. Never eval text.
    lines=path.read_text(encoding='utf-8-sig').splitlines()
    useful=[s.strip() for s in lines if s.strip() and not s.lstrip().startswith('#')]
    if len(useful)<8: raise MapHold('CENTERLINE_TOO_SHORT')
    sep=';' if useful[0].count(';')>useful[0].count(',') else ','
    rows=list(csv.reader(useful,delimiter=sep,skipinitialspace=True))
    header=[s.strip().lower() for s in rows[0]]
    with_header = ('x_m' in header and 'y_m' in header) or (not all(_isnum(x) for x in rows[0][:2]))
    if with_header:
        if 'x_m' not in header or 'y_m' not in header: raise MapHold('UNRECOGNIZED_CENTERLINE_COLUMNS')
        xi,yi=header.index('x_m'),header.index('y_m')
        li=header.index('w_tr_left_m') if 'w_tr_left_m' in header else None
        ri=header.index('w_tr_right_m') if 'w_tr_right_m' in header else None
        data=rows[1:]
    else: xi,yi,li,ri=0,1,(2 if len(rows[0])>2 else None),(3 if len(rows[0])>3 else None);data=rows
    try:
        xy=np.array([[float(r[xi]),float(r[yi])] for r in data],dtype=float)
        left=np.array([float(r[li]) for r in data],dtype=float) if li is not None else np.full(len(xy),np.nan)
        right=np.array([float(r[ri]) for r in data],dtype=float) if ri is not None else np.full(len(xy),np.nan)
    except (ValueError,IndexError) as e: raise MapHold('CENTERLINE_NUMERIC_PARSE_FAILED') from e
    if not np.isfinite(xy).all(): raise MapHold('NONFINITE_CENTERLINE')
    # Explicitly remove duplicate closure point (not arbitrary thinning).
    if np.linalg.norm(xy[0]-xy[-1]) < 1e-5: xy=xy[:-1];left=left[:-1];right=right[:-1]
    if len(xy)<8 or np.any(np.linalg.norm(np.roll(xy,-1,axis=0)-xy,axis=1)<1e-6):
        raise MapHold('DEGENERATE_CENTERLINE')
    return xy,left,right

def _isnum(x):
    try: float(x);return True
    except ValueError:return False

def load_course(root: str|Path, name:str)->Course:
    root=Path(root).resolve(); f=root/name
    if not f.is_dir() or f.is_symlink() or not f.resolve().is_relative_to(root):raise MapHold('MAP_FOLDER_INVALID')
    ys=[p for p in f.glob('*.yaml') if p.is_file() and not p.is_symlink()]
    if len(ys)!=1: raise MapHold('MAP_YAML_UNIQUE_REQUIRED')
    yml=ys[0];m=yaml.safe_load(yml.read_text())
    if not isinstance(m,dict) or not {'image','origin','resolution'}<=m.keys(): raise MapHold('MAP_YAML_FIELDS_MISSING')
    image_rel=Path(str(m['image']))
    if image_rel.is_absolute() or '..' in image_rel.parts:raise MapHold('IMAGE_TRAVERSAL')
    img=f/image_rel
    if not img.is_file() or img.is_symlink() or not img.resolve().is_relative_to(f.resolve()):raise MapHold('IMAGE_MISSING')
    resolution=float(m['resolution']);origin=np.asarray(m['origin'],dtype=float)
    if not math.isfinite(resolution) or resolution<=0 or origin.shape!=(3,) or not np.isfinite(origin).all():raise MapHold('METADATA_INVALID')
    cs=[p for p in f.glob('*centerline*.csv') if p.is_file() and not p.is_symlink()]
    if len(cs)!=1:raise MapHold('CENTERLINE_MISSING_OR_AMBIGUOUS: EXTRACTION_NOT_YET_IMPLEMENTED')
    xy,l,r=_load_centerline(cs[0]);seg=np.linalg.norm(np.roll(xy,-1,axis=0)-xy,axis=1)
    if not np.isfinite(seg).all() or max(seg)>10.0:raise MapHold('UNVERIFIED_ROUTE_GAP_GT_10M')
    arc=np.r_[0,np.cumsum(seg)];total=float(arc[-1])
    if total<5:raise MapHold('ROUTE_LENGTH_INVALID')
    return Course(name,yml,img,xy,arc,total,l,r,{'map_yaml_sha256':sha(yml),'image_sha256':sha(img),'centerline_sha256':sha(cs[0]),'route_length_m':total,'points':len(xy),'resolution_m':resolution})
