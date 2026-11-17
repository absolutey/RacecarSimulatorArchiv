"""Post-surgery coverage: preserve low-speed accepted request AND unsafe-primary fallback."""
import time
from types import SimpleNamespace

from f1tenth_racing_control.guard_ahead import GuardAheadAdmission

LIMITS = {
    'steering_angle_abs_max_rad': 0.5,
    'steering_rate_abs_max_rad_s': 4.0,
    'jerk_abs_max_m_s3': 100.0,
    'deceleration_max_m_s2': 40.0,
    'acceleration_max_m_s2': 40.0,
}

class Dynamics:
    def step(self, state, steer, accel, dt):
        state.x += state.speed * dt
        state.speed += accel * dt
        state.steering = steer
        state.requested_acceleration = accel

class Stop:
    def __init__(self, allow):
        self.allow = allow
        self.dynamics = Dynamics()
    def evaluate(self, state, steer, accel):
        return SimpleNamespace(qualified=self.allow(state, steer, accel))

def car(speed=0.2, applied=4.0):
    return SimpleNamespace(px=0.,py=0.,yaw=0.,v=speed,vx=speed,vy=0.,
        omega=0.,a=applied,ax=0.,ay=0.,slip_angle=0.,
        accel=applied,jerk=0.,steer=-0.19,steer_vel=0.)

def choose(msg, raw, allow=lambda *_: True, started=None):
    return GuardAheadAdmission(Stop(allow), LIMITS).choose(
        msg, -0.215, raw, time.perf_counter_ns() if started is None else started)

def test_low_speed_unchanged_safe_original_remains_original():
    r=choose(car(speed=0.2,applied=4.),4.)
    assert r.proved and r.acceleration_mps2 == 4. and r.checks == 2

def test_low_speed_deescalating_safe_original_remains_original():
    r=choose(car(speed=0.2,applied=4.),3.)
    assert r.proved and r.acceleration_mps2 == 3. and r.checks == 2

def test_low_speed_original_unsafe_rejected_moderate_certified():
    r=choose(car(speed=0.2,applied=4.),4.,lambda _z,_s,a: a <= 2.)
    assert r.proved and r.acceleration_mps2 == 2. and r.checks >= 3

def test_first_fail_actual_state_escalating_uses_jerk_valid_moderation():
    applied=6.238996982574463
    proposed=8.735594529798796
    r=choose(car(speed=0.4340500593185425,applied=applied),
             proposed,lambda _z,_s,a: a < 6.)
    expected=applied-2.5
    assert r.proved and abs(r.acceleration_mps2-expected) < 1e-9 and r.checks == 2
    assert r.acceleration_mps2 != proposed

def test_no_certificate_never_grants_positive():
    r=choose(car(),4.,lambda *_:False)
    assert not r.proved and r.acceleration_mps2 <= 0

def test_budget_exhaustion_never_grants_positive():
    r=choose(car(),4.,started=time.perf_counter_ns()-30_000_000)
    assert not r.proved and r.acceleration_mps2 <= 0
