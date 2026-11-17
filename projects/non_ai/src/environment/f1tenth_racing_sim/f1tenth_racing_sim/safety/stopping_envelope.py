"""OFFLINE STOPPING CERTIFICATE PROTOTYPE -- not runtime qualification.

Fail closed: UNKNOWN is never permission to accelerate. Uses the Simulator's
Dynamics and swept polygon. Certificates are conditional on the tested state,
model, reaction time and margin, not universal invariants.
"""
from dataclasses import dataclass, replace
import math
import time


from f1tenth_racing_sim.physics.vehicle import Dynamics, State
from f1tenth_racing_sim.world.swept_collision import swept_footprint_collision


@dataclass(frozen=True)
class StopCertificate:
    qualified: bool
    reason: str
    steering_rad: float
    stopping_time_s: float
    stopping_distance_m: float
    max_abs_slip_rad: float
    max_lateral_accel_mps2: float
    steps: int


class StoppingEnvelope:
    def __init__(self, dynamics_config, occupancy_map, footprint,
                 dt_s=0.025, max_time_s=8.0,
                 brake_mps2=2.0, reaction_s=0.25,
                 uncertainty_padding_m=0.03,
                 slip_limit_rad=0.35,
                 lateral_acceleration_limit_mps2=6.0):
        values=(dt_s,max_time_s,brake_mps2,reaction_s,uncertainty_padding_m,
                slip_limit_rad,lateral_acceleration_limit_mps2)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('NONFINITE_STOP_CONFIGURATION')
        if (dt_s<=0 or max_time_s<=0 or brake_mps2<=0 or reaction_s<0 or
                uncertainty_padding_m<0 or slip_limit_rad<=0 or
                lateral_acceleration_limit_mps2<=0):
            raise ValueError('INVALID_STOP_CONFIGURATION')
        self.dynamics=Dynamics(dynamics_config)
        self.map=occupancy_map
        self.footprint=footprint
        self.dt=float(dt_s)
        self.max_time=float(max_time_s)
        self.brake=float(brake_mps2)
        self.reaction=float(reaction_s)
        self.padding=float(uncertainty_padding_m)
        self.slip_limit=float(slip_limit_rad)
        self.lat_limit=float(lateral_acceleration_limit_mps2)
        self._inflated_footprint={
            'min_x_m':float(self.footprint['min_x_m'])-self.padding,
            'max_x_m':float(self.footprint['max_x_m'])+self.padding,
            'min_y_m':float(self.footprint['min_y_m'])-self.padding,
            'max_y_m':float(self.footprint['max_y_m'])+self.padding,
        }
        self._inflated_radius=max(
            math.hypot(x,y)
            for x in (float(self._inflated_footprint[k])
                      for k in ('min_x_m','max_x_m'))
            for y in (float(self._inflated_footprint[k])
                      for k in ('min_y_m','max_y_m'))
        )

    def _fail(self,reason,steer,elapsed,travel,slip,lat,steps):
        return StopCertificate(False,reason,steer,elapsed,travel,slip,lat,steps)

    def evaluate_bounded(self,state,steering_rad,last_command_accel_mps2,deadline_ns,*,dynamic_checker=None):
        """A deadline never grants a certificate; it can only cut work short.

        Same model, reaction, jerk, collision and stop checks as evaluate().
        Use only for a known perf_counter_ns timebase and a finite deadline.
        """
        if not isinstance(deadline_ns,int) or deadline_ns<=0:
            raise ValueError('INVALID_STOP_PROOF_DEADLINE')
        return self.evaluate(state,steering_rad,last_command_accel_mps2,
                             deadline_ns=deadline_ns,dynamic_checker=dynamic_checker)

    def evaluate(self,state,steering_rad,last_command_accel_mps2,*,deadline_ns=None,dynamic_checker=None):
        """Check jerk-limited braking with conservative collision padding.

        The reaction interval holds the previous steering/acceleration. A
        candidate is accepted only after simulated speed, yaw rate AND
        actuator acceleration settle into a nearly-stationary condition.
        """
        if deadline_ns is not None and time.perf_counter_ns()>=deadline_ns:
            return self._fail('PROOF_BUDGET_UNPROVEN',float(steering_rad),0.,0.,0.,0.,0)
        values=[float(getattr(state,name)) for name in vars(state)]
        if not all(math.isfinite(v) for v in values):
            return self._fail('NONFINITE_INITIAL_STATE',float(steering_rad),0.,0.,0.,0.,0)
        steer=float(steering_rad)
        accel=float(last_command_accel_mps2)
        if not math.isfinite(steer) or not math.isfinite(accel):
            return self._fail('NONFINITE_COMMAND',steer,0.,0.,0.,0.,0)
        if abs(state.speed)>self.dynamics.c['limits']['speed_abs_max_m_s']:
            return self._fail('SPEED_OUTSIDE_MODEL',steer,0.,0.,0.,0.,0)
        z=replace(state)
        origin=(z.x,z.y)
        max_slip=abs(z.slip);max_lat=abs(z.speed*z.yaw_rate)
        elapsed=0.;steps=0
        n_reaction=math.ceil(self.reaction/self.dt)
        n_total=math.ceil(self.max_time/self.dt)
        inflated=self._inflated_footprint
        # swept collision needs only the preceding x/y/yaw; the reversal
        # check additionally needs speed. Reuse one independent snapshot
        # instead of allocating/copying all 15 vehicle fields on every step.
        before=State()
        if dynamic_checker is not None and not dynamic_checker(z,z,0.,0.):
            return self._fail('RED_DYNAMIC_INITIAL_COLLISION_OR_UNPROVEN',steer,0.,0.,max_slip,max_lat,0)
        for k in range(n_total):
            if (deadline_ns is not None and (k & 3)==0 and
                    time.perf_counter_ns()>=deadline_ns):
                return self._fail('PROOF_BUDGET_UNPROVEN',steer,elapsed,
                                  math.dist(origin,(z.x,z.y)),max_slip,max_lat,steps)
            before.x=z.x
            before.y=z.y
            before.yaw=z.yaw
            before.speed=z.speed
            if k<n_reaction:
                commanded_accel=accel
            else:
                commanded_accel=(-self.brake if z.speed>self.brake*self.dt
                                 else self.brake if z.speed< -self.brake*self.dt
                                 else -z.speed/self.dt)
            try:
                self.dynamics.step(z,steer,commanded_accel,self.dt)
            except (ArithmeticError,ValueError,OverflowError):
                return self._fail('MODEL_FAILED',steer,elapsed,math.dist(origin,(z.x,z.y)),max_slip,max_lat,steps)
            elapsed+=self.dt;steps+=1
            if not all(math.isfinite(v) for v in vars(z).values()):
                return self._fail('NONFINITE_ROLLOUT',steer,elapsed,math.dist(origin,(z.x,z.y)),max_slip,max_lat,steps)
            max_slip=max(max_slip,abs(z.slip))
            max_lat=max(max_lat,abs(z.speed*z.yaw_rate))
            if abs(z.slip)>self.slip_limit or abs(z.speed*z.yaw_rate)>self.lat_limit:
                return self._fail('DYNAMIC_LIMIT',steer,elapsed,math.dist(origin,(z.x,z.y)),max_slip,max_lat,steps)
            if swept_footprint_collision(self.map,before,z,inflated,
                                         cached_radius_m=self._inflated_radius):
                return self._fail('SWEPT_COLLISION',steer,elapsed,math.dist(origin,(z.x,z.y)),max_slip,max_lat,steps)
            if dynamic_checker is not None and not dynamic_checker(
                    before,z,elapsed-self.dt,elapsed):
                return self._fail('RED_DYNAMIC_SWEEP_COLLISION_OR_UNPROVEN',steer,elapsed,
                                  math.dist(origin,(z.x,z.y)),max_slip,max_lat,steps)
            if (k>=n_reaction and abs(z.speed)<=0.025 and
                    abs(z.yaw_rate)<=0.1 and abs(z.requested_acceleration)<=0.2):
                return StopCertificate(True,'CONDITIONAL_STOP_ONLY',steer,elapsed,
                                       math.dist(origin,(z.x,z.y)),max_slip,max_lat,steps)
            if before.speed>0.025 and z.speed< -0.025:
                return self._fail('REVERSED_BEFORE_HOLD',steer,elapsed,math.dist(origin,(z.x,z.y)),max_slip,max_lat,steps)
            if before.speed< -0.025 and z.speed>0.025:
                return self._fail('REVERSED_BEFORE_HOLD',steer,elapsed,math.dist(origin,(z.x,z.y)),max_slip,max_lat,steps)
        return self._fail('TIMEOUT_NO_FULL_STOP',steer,elapsed,math.dist(origin,(z.x,z.y)),max_slip,max_lat,steps)
