"""Fresh guarded-startup selection tests. No ROS graph, map or actuation."""
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


class StoppingEnvelopeStub:
    def __init__(self, check):
        self.dynamics = Dynamics()
        self.check = check

    def evaluate(self, z, steer, accel):
        return SimpleNamespace(qualified=self.check(z, steer, accel))


def car():
    return SimpleNamespace(px=0.0, py=0.0, yaw=0.0, v=1.0, vx=1.0, vy=0.0,
        omega=0.0, a=4.0, ax=0.0, ay=0.0, slip_angle=0.0,
        accel=4.0, jerk=0.0, steer=-0.19, steer_vel=0.0)


def test_original_full_accel_rejected_lower_accel_both_ticks_accepted():
    selector = GuardAheadAdmission(StoppingEnvelopeStub(lambda _z, _s, a: a <= 2.0), LIMITS)
    result = selector.choose(car(), -0.215, 4.0, time.perf_counter_ns())
    assert result.proved and result.acceleration_mps2 == 2.0
    assert result.checks == 2


def test_bounded_moderate_startup_is_preferred_above_blend():
    selector = GuardAheadAdmission(StoppingEnvelopeStub(lambda _z, _s, _a: True), LIMITS)
    result = selector.choose(car(), -0.215, 4.0, time.perf_counter_ns())
    assert result.proved and result.acceleration_mps2 == 2.0 and result.checks == 2


def test_original_safe_proposal_is_preserved_below_blend():
    selector = GuardAheadAdmission(StoppingEnvelopeStub(lambda _z, _s, _a: True), LIMITS)
    state=car();state.v=0.2
    result = selector.choose(state, -0.215, 4.0, time.perf_counter_ns())
    assert result.proved and result.acceleration_mps2 == 4.0 and result.checks == 2


def test_all_failed_certificates_cannot_authorize_acceleration():
    selector = GuardAheadAdmission(StoppingEnvelopeStub(lambda _z, _s, _a: False), LIMITS)
    result = selector.choose(car(), -0.215, 4.0, time.perf_counter_ns())
    assert not result.proved and result.acceleration_mps2 <= 0.0


def test_exhausted_control_deadline_fails_closed():
    selector = GuardAheadAdmission(StoppingEnvelopeStub(lambda _z, _s, _a: True), LIMITS)
    result = selector.choose(car(), -0.215, 4.0, time.perf_counter_ns()-30_000_000)
    assert not result.proved and result.acceleration_mps2 <= 0.0


def test_physical_one_tick_rate_and_jerk_are_enforced():
    selector = GuardAheadAdmission(StoppingEnvelopeStub(lambda _z, _s, a: a == -2.0), LIMITS)
    # From physically applied +4m/s^2, a -2m/s^2 proposal exceeds 100 * .025.
    result = selector.choose(car(), -0.215, 4.0, time.perf_counter_ns())
    assert not result.proved


def test_current_state_only_certificate_never_counts_as_two_tick_proof():
    # Every candidate is safe only for the first one-tick projection.
    # A next-tick swept obstacle must veto the candidate, even when the
    # independent current-state certificate reports qualified.
    selector = GuardAheadAdmission(
        StoppingEnvelopeStub(lambda z, _s, _a: z.x < 0.045), LIMITS
    )
    result = selector.choose(car(), -0.215, 4.0, time.perf_counter_ns())
    assert not result.proved and result.acceleration_mps2 <= 0.0
    assert result.checks >= 2
