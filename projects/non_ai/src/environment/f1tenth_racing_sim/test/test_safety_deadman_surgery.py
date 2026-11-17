from f1tenth_racing_sim.safety_deadman import (stale_brake_target,
                                                      stop_crossed_zero, hold_stationary)
from f1tenth_racing_sim.physics.vehicle import Dynamics, State
import pytest


def test_deadman_brakes_positive_and_negative_speeds():
    assert stale_brake_target(3.0, 2.0) == -2.0
    assert stale_brake_target(-3.0, 2.0) == 2.0
    assert stale_brake_target(0.0, 2.0) == 0.0
    with pytest.raises(ValueError): stale_brake_target(float('nan'), 2.0)


def test_deadman_stops_with_jerk_limit_no_reverse():
    limits={'steering_angle_abs_max_rad':0.4, 'steering_rate_abs_max_rad_s':2.0,
            'speed_abs_max_m_s':20.0, 'acceleration_max_m_s2':4.0,
            'deceleration_max_m_s2':3.0, 'jerk_abs_max_m_s3':25.0}
    # Keep test straight, with zero lateral forces.
    config={'limits':limits,'mass_kg':3.0,'front_axle_from_cg_m':0.16,
            'rear_axle_from_cg_m':0.16,'yaw_inertia_kg_m2':0.04,
            'front_tire':{'pacejka_b':1,'pacejka_c':1,'pacejka_d_n':0},
            'rear_tire':{'pacejka_b':1,'pacejka_c':1,'pacejka_d_n':0}}
    state=State(speed=5.0,vx=5.0)
    dyn=Dynamics(config)
    for _ in range(500):
        old=state.speed
        target=stale_brake_target(state.speed,limits['deceleration_max_m_s2'])
        dyn.step(state,0.0,target,0.025)
        if stop_crossed_zero(old,state.speed): hold_stationary(state)
        if state.speed==0.0: break
    assert state.speed==0.0
    assert state.x > 0 and state.x < 20
    for _ in range(5):
        old=state.speed
        dyn.step(state,0.0,stale_brake_target(state.speed,3),0.025)
        if stop_crossed_zero(old,state.speed): hold_stationary(state)
        assert state.speed==0.0
