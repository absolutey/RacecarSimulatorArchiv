"""Fail-closed per-stop-rollout Red occupancy witness; not a runtime certification.

Uses only independently received R-D prediction samples, verified map/track,
source timestamps, bounded intervals, and a conservative swept disk enclosure.
It NEVER grants an actuation certificate on its own. The Guard's own
StoppingEnvelope tests ego physics and wall collision first.
"""
from dataclasses import dataclass
from bisect import bisect_left
import math


@dataclass(frozen=True)
class RedDisk:
    t: float
    x: float
    y: float
    r: float


def _stamp_s(value):
    return float(value.sec)+float(value.nanosec)*1e-9


def _finite(*values):
    return all(math.isfinite(float(v)) for v in values)


class RedDynamicWitness:
    def __init__(self, prediction, *, now_s, track_id, max_age_s,
                 minimum_horizon_s, ego_radius_m, margin_m=0.12,
                 red_acceleration_uncertainty_mps2=1.0):
        """Raise ValueError on ANY missing/future/uncovered/untrusted Red proof.

        Acceleration uncertainty is a conservative additive positional bound;
        for long-horizon prediction it may deliberately block a manoeuvre.
        The prediction is not trusted as ground truth: freshness and source
        identity are necessary, NOT sufficient, for safe physical deployment.
        """
        source_s=_stamp_s(prediction.source_observation_time)
        age=float(now_s)-source_s
        if (not prediction.valid or prediction.schema_version!=1 or
                prediction.frame_id!='map' or prediction.track_id!=track_id or
                not prediction.source_opponent_id or
                not _finite(now_s,source_s,max_age_s,minimum_horizon_s,
                            ego_radius_m,margin_m,red_acceleration_uncertainty_mps2) or
                not (0<=age<=max_age_s) or
                min(max_age_s,minimum_horizon_s,ego_radius_m)<=0 or
                min(margin_m,red_acceleration_uncertainty_mps2)<0):
            raise ValueError('RED_IDENTITY_AGE_OR_NUMERIC_UNPROVEN')
        if (prediction.source_lifecycle!='TRACKED' or
                not _finite(prediction.source_confidence) or
                not (0.0<float(prediction.source_confidence)<=1.0) or
                isinstance(prediction.source_hit_count, bool) or
                prediction.source_hit_count<2):
            raise ValueError('RED_SOURCE_NOT_CONFIRMED')
        samples=tuple(prediction.samples)
        if len(samples)<2:raise ValueError('RED_PREDICTION_EMPTY')
        disks=[]
        last=0.
        for sm in samples:
            t=float(sm.dt_s)
            length=float(sm.occupancy_half_length_m)
            width=float(sm.occupancy_half_width_m)
            if (not _finite(t,sm.x_m,sm.y_m,sm.vx_mps,sm.vy_mps,length,width) or
                    t<=last or t-last>0.21 or min(length,width)<=0):
                raise ValueError('RED_PREDICTION_SAMPLE_INVALID_OR_GAPPED')
            # Use enclosing disk for rotated rectangle; all points inside.
            r=math.hypot(length,width)+0.5*red_acceleration_uncertainty_mps2*t*t
            disks.append(RedDisk(t,float(sm.x_m),float(sm.y_m),r))
            last=t
        if disks[0].t>0.21:raise ValueError('RED_INITIAL_PREDICTION_GAP')
        # Bound the initial sample by reverse constant-velocity extrapolation,
        # inflate radius to cover the unobserved initial interval.
        first=samples[0]; d=disks[0]
        disks.insert(0,RedDisk(0.,d.x-float(first.vx_mps)*d.t,
                               d.y-float(first.vy_mps)*d.t,
                               d.r+0.5*red_acceleration_uncertainty_mps2*d.t*d.t))
        if disks[-1].t<age+minimum_horizon_s+0.025:
            raise ValueError('RED_STOP_HORIZON_UNCOVERED')
        self.disks=tuple(disks)
        self.times=tuple(d.t for d in disks)
        self.age_s=age
        self.ego_radius_m=float(ego_radius_m)
        self.margin_m=float(margin_m)

    def _at(self,t):
        if not math.isfinite(t) or t<0 or t>self.times[-1]:
            raise ValueError('RED_TIME_UNCOVERED')
        if t==self.times[0]:return self.disks[0]
        j=bisect_left(self.times,t)
        if j>=len(self.disks):raise ValueError('RED_TIME_UNCOVERED')
        a,b=self.disks[j-1],self.disks[j]
        weight=(t-a.t)/(b.t-a.t)
        return RedDisk(t, a.x+(b.x-a.x)*weight,a.y+(b.y-a.y)*weight,
                       # Maximum radius + motion-interpolation allowance.
                       max(a.r,b.r)+0.01)

    def collision_free(self, before, after, begin_s, end_s):
        """Conservative relative swept motion over ONE Ego physics timestep."""
        if (not _finite(before.x,before.y,after.x,after.y,begin_s,end_s) or
                end_s<begin_s or begin_s<0 or end_s-begin_s>0.025000001):
            return False
        try:
            red0=self._at(begin_s+self.age_s)
            red1=self._at(end_s+self.age_s)
        except ValueError:return False
        dx0=before.x-red0.x;dy0=before.y-red0.y
        vx=(after.x-red1.x)-dx0;vy=(after.y-red1.y)-dy0
        vv=vx*vx+vy*vy
        u=max(0.,min(1.,-(dx0*vx+dy0*vy)/vv)) if vv>0 else 0.
        separation=math.hypot(dx0+u*vx,dy0+u*vy)
        radius=self.ego_radius_m+max(red0.r,red1.r)+self.margin_m
        return bool(separation>radius)
