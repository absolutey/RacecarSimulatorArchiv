"""Fail-closed offline building block for Red dynamic occupancy.

This is *not* a deployed H2H Guard certificate. It must not be used to lift
Guard's Solo-only interlock. Does check time coverage/freshness, source
identity and conservative capsule separation for a candidate Ego path.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class SpaceTimePoint:
    t_s: float
    x_m: float
    y_m: float
    radius_m: float

@dataclass(frozen=True)
class OccupancyFinding:
    safe: bool
    reason: str
    first_bad_t_s: float


def check_dynamic_occupancy(ego, red, *, predicted_at_s,
                            now_s, max_age_s, required_horizon_s,
                            margin_m):
    """Conservatively reject any gap or temporally unproven occupancy.

    Each time slice bounds Ego/Red with uncertainty-expanded disks. A swept
    segment check uses relative-motion minimum distance to cover intersample
    collisions, and requires both trajectories' *matching* time grids.
    """
    src=tuple(ego);other=tuple(red)
    cfg=(predicted_at_s,now_s,max_age_s,required_horizon_s,margin_m)
    if not all(math.isfinite(float(x)) for x in cfg) or max_age_s<0 or required_horizon_s<=0 or margin_m<0:
        return OccupancyFinding(False,'INVALID_TIME_OR_MARGIN',0.)
    if now_s<predicted_at_s or now_s-predicted_at_s>max_age_s:
        return OccupancyFinding(False,'STALE_OR_FUTURE_RED_PREDICTION',0.)
    if len(src)<2 or len(src)!=len(other):
        return OccupancyFinding(False,'DYNAMIC_HORIZON_UNCOVERED',0.)
    for i,(a,b) in enumerate(zip(src,other)):
        v=(a.t_s,a.x_m,a.y_m,a.radius_m,b.t_s,b.x_m,b.y_m,b.radius_m)
        if not all(math.isfinite(float(x)) for x in v):
            return OccupancyFinding(False,'NONFINITE_OCCUPANCY',float(i))
        if a.radius_m<0 or b.radius_m<0 or a.t_s!=b.t_s or a.t_s < 0:
            return OccupancyFinding(False,'INVALID_RADIUS_OR_TIME_IDENTITY',a.t_s)
        if i==0 and a.t_s>0:
            return OccupancyFinding(False,'DYNAMIC_INITIAL_GAP',a.t_s)
        if i and (a.t_s<=src[i-1].t_s or b.t_s<=other[i-1].t_s):
            return OccupancyFinding(False,'NONMONOTONIC_OCCUPANCY',a.t_s)
        if math.hypot(a.x_m-b.x_m,a.y_m-b.y_m) <= a.radius_m+b.radius_m+margin_m:
            return OccupancyFinding(False,'DYNAMIC_COLLISION_AT_SAMPLE',a.t_s)
        if i:
            p,q=src[i-1],other[i-1]
            # relative point moving from prior separation to current; bound
            # radii by maximum endpoint expansion along the swept interval.
            x0=p.x_m-q.x_m;y0=p.y_m-q.y_m
            dx=(a.x_m-b.x_m)-x0;dy=(a.y_m-b.y_m)-y0
            dot=dx*dx+dy*dy
            u=max(0.,min(1.,-(x0*dx+y0*dy)/dot)) if dot>0 else 0.
            separation=math.hypot(x0+u*dx,y0+u*dy)
            bound=max(p.radius_m,a.radius_m)+max(q.radius_m,b.radius_m)+margin_m
            if separation<=bound:
                return OccupancyFinding(False,'DYNAMIC_SWEPT_COLLISION',a.t_s)
    if src[-1].t_s < required_horizon_s:
        return OccupancyFinding(False,'DYNAMIC_HORIZON_UNCOVERED',src[-1].t_s)
    return OccupancyFinding(True,'CONDITIONAL_GEOMETRIC_CLEARANCE_ONLY',src[-1].t_s)
