"""Independent fail-closed direction/corridor-based physical lap oracle.

The evaluator's first forward seam crossing starts a lap but does not
complete it. This oracle requires the starting seam and then an additional
full clockwise/counterclockwise travel in the declared track direction.
It does not infer a full lap from a publisher-provided lap_count field.
"""
from dataclasses import dataclass
from math import atan2,hypot,isfinite

@dataclass(frozen=True)
class Observation:
    time_s: float
    x_m: float
    y_m: float
    collision: bool = False

@dataclass(frozen=True)
class LapVerdict:
    qualified_laps: int
    valid: bool
    reason: str
    forward_distance_m: float


def _projection(x,y,points,lengths,total):
    best=None
    at=0.
    n=len(points)
    for i,(p,q) in enumerate(zip(points,points[1:]+points[:1])):
        vx=q[0]-p[0];vy=q[1]-p[1];l2=vx*vx+vy*vy
        if l2<=1e-15:raise ValueError('DEGENERATE_CENTERLINE')
        u=max(0.,min(1.,((x-p[0])*vx+(y-p[1])*vy)/l2))
        nx=p[0]+u*vx;ny=p[1]+u*vy
        d=hypot(x-nx,y-ny)
        if best is None or d<best[0]:best=(d,(at+u*lengths[i])%total)
        at+=lengths[i]
    return best


def qualify_physical_laps(observations, centerline_xy, *,
                           corridor_half_width_m, maximum_progress_step_fraction=0.25,
                           max_vehicle_speed_mps=None):
    obs=tuple(observations); pts=[(float(p[0]),float(p[1])) for p in centerline_xy]
    if len(pts)<3 or len(obs)<3:
        return LapVerdict(0,False,'INSUFFICIENT_EVIDENCE',0.)
    if (not isfinite(corridor_half_width_m) or corridor_half_width_m<=0 or
            not (0<maximum_progress_step_fraction<.5) or
            max_vehicle_speed_mps is None or not isfinite(float(max_vehicle_speed_mps)) or
            max_vehicle_speed_mps<=0):
        return LapVerdict(0,False,'INVALID_TRACK_PARAMETERS',0.)
    lengths=[hypot(q[0]-p[0],q[1]-p[1]) for p,q in zip(pts,pts[1:]+pts[:1])]
    length=sum(lengths)
    if not isfinite(length) or length<=0 or min(lengths)<=0:
        return LapVerdict(0,False,'DEGENERATE_CENTERLINE',0.)
    previous_time=-float('inf');previous_station=None
    total_forward=0.;start_seen=False;from_start=0.;complete=0
    # Start-line crossing requires a forward segment through s=0; just an
    # initial message labelled lap=1 is NOT start/finish evidence.
    for o in obs:
        if not all(isfinite(float(x)) for x in (o.time_s,o.x_m,o.y_m)) or o.time_s<=previous_time:
            return LapVerdict(complete,False,'INVALID_OBSERVATION_TIME',total_forward)
        elapsed_s=o.time_s-previous_time
        previous_time=o.time_s
        if o.collision:return LapVerdict(complete,False,'COLLISION',total_forward)
        try:d,s=_projection(o.x_m,o.y_m,pts,lengths,length)
        except ValueError:return LapVerdict(complete,False,'INVALID_CENTERLINE',total_forward)
        if d>corridor_half_width_m:return LapVerdict(complete,False,'OUTSIDE_TRACK_CORRIDOR',total_forward)
        if previous_station is None:previous_station=s;continue
        signed=(s-previous_station+0.5*length)%length-0.5*length
        # Jump beyond a quarter lap is ambiguous and must never count as
        # forward progress. Small reverse motion also invalidates a lap.
        if (signed>length*maximum_progress_step_fraction or signed< -1e-5 or
                # Projected-centerline travel can exceed Cartesian travel by
                # two corridor deviations; require a physical upper bound.
                signed>max_vehicle_speed_mps*elapsed_s+2*corridor_half_width_m):
            return LapVerdict(complete,False,'REVERSE_OR_AMBIGUOUS_PROGRESS',total_forward)
        total_forward+=signed
        seam=previous_station>s and signed>=0
        if start_seen:
            from_start+=signed
            if seam:
                if from_start>=0.9*length:
                    complete+=1
                else:
                    return LapVerdict(complete,False,'PREMATURE_SEAM_REENTRY',total_forward)
                from_start=0.
        elif seam:
            start_seen=True;from_start=0.
        previous_station=s
    if complete:
        return LapVerdict(complete,True,'PHYSICAL_LAP_VERIFIED',total_forward)
    return LapVerdict(0,False,'NO_COMPLETE_LAP_AFTER_START_SEAM',total_forward)
