import math
from types import SimpleNamespace as NS
import pytest
from f1tenth_racing_control.red_dynamic_guard import RedDynamicWitness
from f1tenth_racing_control.actuator_guard_contract import ActuatorGuardContract, ActuatorIntent
from f1tenth_racing_sim.world.collision import swept_vehicle_disk_overlap


def prediction(*, x=10., vx=0., lifecycle='TRACKED', valid=True, horizon=9.,
               radius=.3, observed=100., source='opponent0'):
    samples=[]
    t=0.1
    while t<=horizon+1e-9:
        samples.append(NS(dt_s=round(t,6),x_m=x+vx*t,y_m=0.,vx_mps=vx,
                          vy_mps=0.,occupancy_half_length_m=radius,
                          occupancy_half_width_m=radius))
        t+=.1
    return NS(valid=valid,schema_version=1,frame_id='map',track_id='iccas2025',
              source_opponent_id=source,source_lifecycle=lifecycle,source_confidence=.9,
              source_hit_count=4,source_observation_time=NS(sec=int(observed),
                  nanosec=int(round((observed%1)*1e9))),samples=samples)


def witness(p, now=100.1):
    return RedDynamicWitness(p,now_s=now,track_id='iccas2025',max_age_s=.3,
                             minimum_horizon_s=8.,ego_radius_m=.25)


def test_safe_far_red():
    w=witness(prediction(x=100.))
    a=NS(x=0.,y=0.);b=NS(x=.25,y=0.)
    assert w.collision_free(a,b,0.,.025)


def test_unsafe_near_and_crossing():
    w=witness(prediction(x=.1))
    assert not w.collision_free(NS(x=0.,y=0.),NS(x=0.,y=0.),0.,.025)
    w=witness(prediction(x=1.))
    assert not w.collision_free(NS(x=0.,y=0.),NS(x=2.,y=0.),0.,.025)


@pytest.mark.parametrize('change',[
    lambda p: setattr(p,'valid',False),
    lambda p: setattr(p,'frame_id','odom'),
    lambda p: setattr(p,'track_id','spielberg'),
    lambda p: setattr(p,'source_lifecycle','COASTING'),
    lambda p: setattr(p,'source_hit_count',0),
    lambda p: setattr(p,'source_confidence',float('nan')),
    lambda p: setattr(p,'source_confidence',float('inf')),
    lambda p: setattr(p,'source_confidence',1.2),
    lambda p: setattr(p.samples[1],'x_m',float('nan')),
    lambda p: p.samples.__delitem__(slice(2,6)),
])
def test_reject_invalid_red(change):
    p=prediction();change(p)
    with pytest.raises(ValueError):witness(p)


def test_reject_stale_and_short_horizon():
    with pytest.raises(ValueError):witness(prediction(),now=100.31)
    with pytest.raises(ValueError):witness(prediction(horizon=1.0))


def test_swept_mid_interval_collision():
    fp={'min_x_m':-.2,'max_x_m':.2,'min_y_m':-.1,'max_y_m':.1}
    a=NS(x=-1.,y=0.);b=NS(x=1.,y=0.)
    c=NS(x=1.,y=0.);d=NS(x=-1.,y=0.)
    assert swept_vehicle_disk_overlap(a,b,c,d,fp)
    assert not swept_vehicle_disk_overlap(a,NS(x=-.8,y=0.),NS(x=6.,y=0.),NS(x=5.8,y=0.),fp)


def test_reference_transfer_preserves_replay_and_clock():
    c=ActuatorGuardContract()
    c.arm(run_epoch='e',source_id='R_B',reference_id='old',track_id='iccas',
          map_sha256='hash',ros_time_ns=10,steady_time_ns=10)
    first=ActuatorIntent('R_B','e',1,11,100,'old','iccas','hash',11,0.,0.)
    assert c.validate(first,ros_time_ns=12,steady_time_ns=12,
        state_received_steady_ns=11,validated_stop_certificate=True).accepted
    assert c.transfer_reference(next_reference_id='new',ros_time_ns=13,
                                steady_time_ns=13).accepted
    assert c.previous_sequence==1 and c.run_epoch=='e'
    reused=ActuatorIntent('R_B','e',1,14,100,'new','iccas','hash',14,0.,0.)
    assert not c.validate(reused,ros_time_ns=15,steady_time_ns=15,
        state_received_steady_ns=14,validated_stop_certificate=True).accepted


def test_reference_transfer_rejects_idle():
    assert not ActuatorGuardContract().transfer_reference(
        next_reference_id='x',ros_time_ns=1,steady_time_ns=1).accepted
