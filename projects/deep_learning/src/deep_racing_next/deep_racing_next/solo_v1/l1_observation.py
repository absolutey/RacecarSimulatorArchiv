"""L1: finite ego plus LiDAR and route-relative features."""
import math
import numpy as np

FEATURE_COUNT=24

def build_features(obs,course,progress_s:float, target_speed_mps:float, planned_offset_m:float=0.):
    x,y,yaw,v=[float(obs[k]) for k in ('x','y','yaw','v')]
    s,offset,j=course.projection(x,y)
    point,ti=course.lookahead(s,2.0+0.35*v)
    direction_xy=course.xy[(ti+1)%len(course.xy)]-course.xy[ti]
    normal=np.array([-direction_xy[1],direction_xy[0]])/max(np.linalg.norm(direction_xy),1e-6)
    left,right=course.width_left[ti],course.width_right[ti]
    # No verified boundary widths => disable learned lateral shift; still allow speed hint.
    safe_left=max(0.,float(left)-.4) if math.isfinite(left) else 0.
    safe_right=max(0.,float(right)-.4) if math.isfinite(right) else 0.
    limited=float(np.clip(planned_offset_m,-safe_right,safe_left))
    point=point+normal*limited
    direction=math.atan2(point[1]-y,point[0]-x)
    err=math.atan2(math.sin(direction-yaw),math.cos(direction-yaw))
    scan=np.asarray(obs['scan'],dtype=float).reshape(-1)
    if len(scan)<16 or np.any(np.isnan(scan)) or np.any(scan<0):raise ValueError('INVALID_LIDAR_SCAN')
    scan=np.minimum(scan,20)
    # fixed beams independently of lidar native resolution
    beams=np.percentile(np.array_split(scan,16),20,axis=1) if len(scan)%16==0 else np.array([np.percentile(a,20) for a in np.array_split(scan,16)])
    curvature=0.0
    a=course.xy[(j-1)%len(course.xy)]; b=course.xy[j];c=course.xy[(j+1)%len(course.xy)]
    ab=b-a;bc=c-b
    curvature=float((ab[0]*bc[1]-ab[1]*bc[0])/(max(np.linalg.norm(ab)*np.linalg.norm(bc)*np.linalg.norm(c-a),1e-6))*2)
    out=np.r_[np.clip(beams/20,0,1),np.clip(v/20,0,1),np.clip(offset/3,-1,1),
       math.sin(err),math.cos(err),np.clip(curvature*2,-1,1),np.clip(target_speed_mps/20,0,1),
       np.clip(progress_s/course.total_m,0,1),float(obs.get('yaw_rate',0))/10].astype(np.float32)
    if out.shape!=(FEATURE_COUNT,) or not np.isfinite(out).all():raise ValueError('NONFINITE_FEATURES')
    return out
