"""Recoverable ABORT only after independent parked, nominal and Red evidence."""
from types import SimpleNamespace as O
import math
import pytest
# Load the pure contract directly for offline validation without ROS 2 imports.
import importlib.util
import sys
from pathlib import Path
module_path=Path(__file__).resolve().parents[1]/'f1tenth_racing_reference'/'abort_recovery_gate.py'
spec=importlib.util.spec_from_file_location('abort_recovery_gate_v5_pure',module_path)
module=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=module
spec.loader.exec_module(module)
evaluate_abort_recovery=module.evaluate_abort_recovery


def stamp(value):
    return O(sec=int(value),nanosec=int(round((value-int(value))*1e9)))


def good(now=10.0):
    nominal = [O(x_m=float(i),y_m=0.,heading_rad=0.) for i in range(6)]
    ego = O(px=1.5,py=0.,yaw=0.,v=0.,vx=0.,vy=0.,omega=0.,a=0.,accel=0.,
            physics_stamp=stamp(now-0.025),physics_sequence=10)
    red = O(valid=True,schema_version=1,source_lifecycle='TRACKED',
            source_opponent_id='red',source_track_revision=1,source_confidence=0.95,
            source_hit_count=4,track_id='iccas2025',frame_id='map',
            source_observation_time=stamp(now-0.05),samples=[])
    for j in range(1,41):
        t=.1*j
        red.samples.append(O(dt_s=t,x_m=10+t,y_m=0.,vx_mps=1.,vy_mps=0.,
                             occupancy_half_length_m=.20, occupancy_half_width_m=.10))
    return nominal,ego,red


def gate(nominal,ego,red,**kw):
    args=dict(now_ns=int(10*1e9),ego_radius_m=.35,
              collision_known=True,collision_active=False)
    args.update(kw)
    return evaluate_abort_recovery(nominal,ego,red,**args)


def test_parked_fresh_red_far():
    assert gate(*good()).allowed


def test_no_collision_witness():
    assert gate(*good(),collision_known=False).reason=='RECOVERY_COLLISION_NOT_CLEARED'


def test_collision_latch():
    assert gate(*good(),collision_active=True).reason=='RECOVERY_COLLISION_NOT_CLEARED'


def test_no_prediction():
    n,e,r=good();assert not gate(n,e,None).allowed


def test_ego_stale():
    n,e,r=good();e.physics_stamp=stamp(9.8)
    assert gate(n,e,r).reason=='RECOVERY_EGO_STALE_OR_FUTURE'


def test_red_stale():
    n,e,r=good();r.source_observation_time=stamp(9)
    assert gate(n,e,r).reason=='RECOVERY_RED_STALE_OR_FUTURE'


def test_red_future():
    n,e,r=good();r.source_observation_time=stamp(11)
    assert gate(n,e,r).reason=='RECOVERY_RED_STALE_OR_FUTURE'


@pytest.mark.parametrize('key,value',[
 ('v',.1),('vx',.1),('omega',.1),('a',.5),('accel',.2)])
def test_moving_denied(key,value):
    n,e,r=good();setattr(e,key,value)
    assert gate(n,e,r).reason=='RECOVERY_EGO_NOT_PARKED'


def test_off_nominal_denied():
    n,e,r=good();e.py=.25
    assert gate(n,e,r).reason=='RECOVERY_NOMINAL_ALIGNMENT_UNPROVEN'


def test_heading_error_denied():
    n,e,r=good();e.yaw=.5
    assert gate(n,e,r).reason=='RECOVERY_NOMINAL_ALIGNMENT_UNPROVEN'


def test_near_red_denied():
    n,e,r=good()
    for sm in r.samples:sm.x_m=1.6
    assert gate(n,e,r).reason=='RECOVERY_RED_SWEPT_OCCUPANCY_UNPROVEN'


def test_short_red_horizon_denied():
    n,e,r=good();r.samples=r.samples[:8]
    assert gate(n,e,r).reason=='RECOVERY_RED_HORIZON_UNCOVERED'


def test_sample_gap_denied():
    n,e,r=good();r.samples[1].dt_s=.5
    assert gate(n,e,r).reason=='RECOVERY_RED_SAMPLING_INVALID'


def test_low_confidence_denied():
    n,e,r=good();r.source_hit_count=0
    assert gate(n,e,r).reason=='RECOVERY_RED_IDENTITY_UNPROVEN'


def test_non_finite_denied():
    n,e,r=good();e.px=math.nan
    assert gate(n,e,r).reason=='RECOVERY_NONFINITE'


def test_worse_uncertainty_denied():
    n,e,r=good()
    for sm in r.samples:sm.x_m=4.0;sm.vx_mps=0.
    assert not gate(n,e,r).allowed


def test_nominal_missing_denied():
    n,e,r=good()
    assert gate(n[:2],e,r).reason=='RECOVERY_NOMINAL_UNAVAILABLE'


def test_horizon_not_accidentally_require_nine_seconds():
    n,e,r=good();r.samples=r.samples[:23]
    assert gate(n,e,r).allowed


def test_nan_red_confidence_denied():
    n,e,r=good();r.source_confidence=math.nan
    assert gate(n,e,r).reason=='RECOVERY_RED_IDENTITY_UNPROVEN'


def test_nan_red_acceleration_bound_denied():
    n,e,r=good()
    assert gate(n,e,r,red_accel_uncertainty_mps2=math.nan).reason=='RECOVERY_NONFINITE'
