"""Conservative closed-track execution-speed envelope, never an actuation certificate.

A plan may have 20 m/s authored velocities but no full-stop certificate within
8 s at 2 m/s2. This module adds *necessary* local constraints and periodic
forward/backward reachability. Every actuator command still needs current and
N+1 GuardAhead certificates and an independent Guard certificate.

The values come from the verified vehicle model and active stop horizon, not
historical arbitrary fixed 5.0/5.6/5.8 m/s qualification caps.
"""
from dataclasses import dataclass, replace
import math

@dataclass(frozen=True)
class ExecutionSpeedAudit:
    source_peak_mps: float
    planned_peak_mps: float
    horizon_necessary_speed_mps: float
    reduced_samples: int
    limit_reason: str
    # Not a qualified stop certificate.
    qualified: bool = False


def derive_closed_loop_execution_speed(samples, station_period_m, *,
                                       vehicle_speed_bound_mps,
                                       braking_mps2,
                                       acceleration_mps2,
                                       proof_horizon_s,
                                       reaction_s,
                                       lateral_accel_bound_mps2,
                                       physics_step_s,
                                       wheelbase_m=None,
                                       steering_rate_limit_rad_s=None,
                                       jerk_limit_mps3=None):
    """Return immutable lowered speed samples and a read-only audit.

    Crucial: this is a conservative *proposal filter*, not a guarantee that a
    steering, wall corridor, tire-slip or complete stop path is safe.
    Non-finite/out-of-order data raises; the caller must reject the reference.
    The period wraps last -> first and both forward and backward kinematic
    inequalities are enforced to an algebraic fixed point.
    """
    src=tuple(samples)
    n=len(src)
    if n<3:raise ValueError('SPEED_AUTH_TOO_FEW_SAMPLES')
    values=(station_period_m,vehicle_speed_bound_mps,braking_mps2,
            acceleration_mps2,proof_horizon_s,reaction_s,
            lateral_accel_bound_mps2,physics_step_s)
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError('SPEED_AUTH_NONFINITE_CONFIG')
    length,v_bound,brake,accel,horizon,reaction,lat,dt=map(float,values)
    if min(length,v_bound,brake,accel,horizon,lat,dt)<=0 or reaction<0 or reaction>=horizon:
        raise ValueError('SPEED_AUTH_INVALID_CONFIG')
    ss=[float(s.s_m) for s in src]
    if ss[0]!=0.0 or not all(0<=a<b<length for a,b in zip(ss,ss[1:])):
        raise ValueError('SPEED_AUTH_BAD_STATIONS')
    if not (0 <= ss[-1] < length):raise ValueError('SPEED_AUTH_BAD_LAST_STATION')
    ds=[(ss[(i+1)%n]-ss[i])%length for i in range(n)]
    if not all(math.isfinite(x) and x>0 for x in ds):raise ValueError('SPEED_AUTH_BAD_SEGMENTS')
    # The reaction and a complete discrete 25ms braking step both consume
    # the finite proof horizon. The 1-step margin is a planning convention,
    # not an additional certificate nor a fixed speed cap.
    necessary_horizon_speed=max(0.,brake*(horizon-reaction-dt))
    if necessary_horizon_speed<=0:raise ValueError('SPEED_AUTH_NO_STOP_HORIZON')
    v=[]
    for s in src:
        desired=float(s.target_speed_mps)
        curvature=float(s.curvature_per_m)
        if not (math.isfinite(desired) and math.isfinite(curvature) and desired>=0):
            raise ValueError('SPEED_AUTH_INVALID_REFERENCE')
        corner=math.sqrt(lat/abs(curvature)) if abs(curvature)>1e-12 else math.inf
        v.append(min(desired,v_bound,necessary_horizon_speed,corner))
    # On a circular track both inequalities can propagate across the seam.
    # Constraints only ever tighten and converge. A finite pass limit with
    # fail closed guards floating instability (there is no permission on fail).
    for _ in range(n*4+4):
        changed=False
        for i in range(n-1,-1,-1):
            j=(i+1)%n
            permitted=math.sqrt(max(0.,v[j]*v[j]+2*brake*ds[i]))
            if v[i]>permitted+1e-12:v[i]=permitted;changed=True
        for i in range(n):
            j=(i+1)%n
            permitted=math.sqrt(max(0.,v[i]*v[i]+2*accel*ds[i]))
            if v[j]>permitted+1e-12:v[j]=permitted;changed=True
        if not changed:break
    else:raise ValueError('SPEED_AUTH_DID_NOT_CONVERGE')
    for i in range(n):
        j=(i+1)%n
        if (v[i]**2 > v[j]**2+2*brake*ds[i]+1e-7 or
            v[j]**2 > v[i]**2+2*accel*ds[i]+1e-7):
            raise ValueError('SPEED_AUTH_KINEMATIC_CONSTRAINT_FAILURE')
    # Discrete sampled actuator-feasibility envelope. Uniform lowering cannot
    # violate the acceleration, braking or curvature caps already established,
    # and avoids reintroducing an arbitrary 5m/s threshold. Under a uniform
    # velocity scale alpha: steering-rate ~alpha, acceleration ~alpha^2,
    # discrete jerk ~alpha^3. This is NOT an independent runtime certificate.
    discrete_limits=(wheelbase_m,steering_rate_limit_rad_s,jerk_limit_mps3)
    if any(x is not None for x in discrete_limits):
        if any(x is None or not math.isfinite(float(x)) or float(x)<=0
               for x in discrete_limits):
            raise ValueError('SPEED_AUTH_INCOMPLETE_ACTUATOR_LIMITS')
        wheelbase=float(wheelbase_m)
        steer_limit=float(steering_rate_limit_rad_s)
        jerk_limit=float(jerk_limit_mps3)
        steers=[math.atan(wheelbase*float(s.curvature_per_m)) for s in src]
        edge_times=[2*ds[i]/max(v[i]+v[(i+1)%n],1e-9) for i in range(n)]
        steering_rate=max(abs(steers[(i+1)%n]-steers[i])/edge_times[i]
                          for i in range(n))
        accelerations=[(v[(i+1)%n]**2-v[i]**2)/(2*ds[i]) for i in range(n)]
        # Worst of the two adjacent cell dwell times prevents a large jump
        # being hidden by a long preceding low-speed station interval.
        sampled_jerk=max(abs(accelerations[i]-accelerations[(i-1)%n]) /
                         min(edge_times[i],edge_times[(i-1)%n]) for i in range(n))
        alpha=min(1.0,steer_limit/max(steering_rate,1e-15),
                  (jerk_limit/max(sampled_jerk,1e-15))**(1/3))
        if not math.isfinite(alpha) or alpha<=0:
            raise ValueError('SPEED_AUTH_ACTUATOR_SCALE_INVALID')
        v=[velocity*alpha for velocity in v]
        # Confirm the final discrete sampled constraints (roundoff margin).
        after_times=[2*ds[i]/max(v[i]+v[(i+1)%n],1e-9) for i in range(n)]
        after_accel=[(v[(i+1)%n]**2-v[i]**2)/(2*ds[i]) for i in range(n)]
        if any(abs(steers[(i+1)%n]-steers[i])/after_times[i]>
               steer_limit*(1+1e-8) for i in range(n)):
            raise ValueError('SPEED_AUTH_STEER_RATE_DISCRETE_FAILURE')
        if any(abs(after_accel[i]-after_accel[(i-1)%n])/
               min(after_times[i],after_times[(i-1)%n])>
               jerk_limit*(1+1e-8) for i in range(n)):
            raise ValueError('SPEED_AUTH_JERK_DISCRETE_FAILURE')
    result=[]
    for i,s in enumerate(src):
        j=(i+1)%n
        a=(v[j]*v[j]-v[i]*v[i])/(2*ds[i])
        # use namespace-independent snapshot with same field names
        from types import SimpleNamespace
        result.append(SimpleNamespace(**{
            **vars(s), 'target_speed_mps':v[i],
            'target_accel_mps2':max(-brake,min(accel,a))}))
    return tuple(result),ExecutionSpeedAudit(
        max(float(s.target_speed_mps) for s in src),max(v),
        necessary_horizon_speed,
        sum(1 for s,t in zip(src,result) if t.target_speed_mps < float(s.target_speed_mps)-1e-10),
        'NECESSARY_HORIZON_CURVATURE_ACCEL_BRAKE_STEERING_JERK_DISCRETE_NOT_GUARD_QUALIFIED' if all(x is not None for x in discrete_limits) else 'NECESSARY_HORIZON_CURVATURE_ACCEL_BRAKE_ONLY_NOT_GUARD_QUALIFIED',False)
