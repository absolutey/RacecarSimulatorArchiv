"""V6 GuardAhead: non-finite measured/stepped physics is never certified."""
import math
import time
from types import SimpleNamespace
import pytest
from f1tenth_racing_control.guard_ahead import GuardAheadAdmission

LIMITS = {
    'steering_angle_abs_max_rad': 0.5,
    'steering_rate_abs_max_rad_s': 4.0,
    'jerk_abs_max_m_s3': 100.,
    'deceleration_max_m_s2': 40.,
    'acceleration_max_m_s2': 40.,
}


def car():
    return SimpleNamespace(px=1., py=2., yaw=0.1, v=1., vx=1., vy=0.,
        omega=0., a=0., ax=0., ay=0., slip_angle=0., accel=0.,
        jerk=0., steer=0.02, steer_vel=0.)


class NeverRejectProof:
    def __init__(self, poison=False):
        self.dynamics = SimpleNamespace(step=self.step)
        self.calls = 0
        self.poison = poison

    def step(self, state, steer, accel, dt):
        if self.poison:
            state.x = math.nan
        else:
            state.x += state.speed * dt

    def evaluate(self, state, steer, accel):
        self.calls += 1
        return SimpleNamespace(qualified=True)


@pytest.mark.parametrize('field', [
    'px','py','yaw','v','vx','vy','omega','a','ax','ay','slip_angle',
    'accel','jerk','steer','steer_vel',
])
def test_nonfinite_carstate_never_reaches_authorizing_proof(field):
    m = car()
    setattr(m, field, math.nan)
    proof = NeverRejectProof()
    outcome = GuardAheadAdmission(proof, LIMITS).choose(m, 0.02, 0.0, time.perf_counter_ns())
    assert not outcome.proved
    assert proof.calls == 0


def test_poisoned_one_tick_rollout_cannot_get_positive_certificate():
    proof = NeverRejectProof(poison=True)
    outcome = GuardAheadAdmission(proof, LIMITS).choose(car(), 0.02, 0.5,
                                                        time.perf_counter_ns())
    assert not outcome.proved
    assert proof.calls == 0


def test_finite_backend_still_requires_two_certificates():
    proof = NeverRejectProof()
    outcome = GuardAheadAdmission(proof, LIMITS).choose(car(), 0.02, 0.0,
                                                        time.perf_counter_ns())
    assert outcome.proved
    assert outcome.checks == proof.calls == 2
