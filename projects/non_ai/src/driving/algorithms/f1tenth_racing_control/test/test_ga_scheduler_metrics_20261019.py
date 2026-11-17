"""GuardAhead 7ms timebase preserved, scheduler/cpu evidence only."""
import time
from types import SimpleNamespace
from f1tenth_racing_control.guard_ahead import GuardAheadAdmission

LIMITS={'steering_angle_abs_max_rad': .5,
        'steering_rate_abs_max_rad_s': 5.,
        'jerk_abs_max_m_s3': 200.,
        'deceleration_max_m_s2': 8.,
        'acceleration_max_m_s2': 8.}


def car():
    return SimpleNamespace(px=0.,py=0.,yaw=0.,v=0.,vx=0.,vy=0.,omega=0.,
                           a=0.,ax=0.,ay=0.,slip_angle=0.,accel=0.,
                           jerk=0.,steer=0.,steer_vel=0.)


class Proof:
    def __init__(self):
        self.dynamics=SimpleNamespace(step=lambda z,s,a,dt:None)
        self.calls=0

    def evaluate_bounded(self,z,steer,accel,deadline):
        self.calls+=1
        return SimpleNamespace(qualified=True)


def test_two_proofs_remain_mandatory_and_7ms_unchanged():
    p=Proof()
    a=GuardAheadAdmission(p,LIMITS)
    assert a.MAX_EXTRA_NS==7_000_000
    out=a.choose(car(),0.,0.,time.perf_counter_ns())
    assert out.proved and p.calls==2
    assert out.scheduler_gap_us>=0
    assert 0<=out.budget_left_at_start_us<=7000
    assert out.elapsed_us>=out.cpu_elapsed_us-500


def test_no_compute_when_full_cycle_slot_already_expired():
    p=Proof()
    a=GuardAheadAdmission(p,LIMITS)
    out=a.choose(car(),0.,0.,time.perf_counter_ns()-30_000_000)
    assert not out.proved and p.calls==0
    assert out.reason=='GUARD_AHEAD_NO_TIME_SLOT'


def test_invalid_input_remains_fail_closed():
    p=Proof()
    m=car();m.v=float('nan')
    out=GuardAheadAdmission(p,LIMITS).choose(m,0.,0.,time.perf_counter_ns())
    assert not out.proved and p.calls==0
