"""Red lifecycle confidence must never bypass Guard with NaN comparison."""
from types import SimpleNamespace as N
import math
import pytest
from f1tenth_racing_control.red_dynamic_guard import RedDynamicWitness


def red(confidence):
    samples=[N(dt_s=i*0.1,x_m=100.0,y_m=0.,vx_mps=0.,vy_mps=0.,
        occupancy_half_length_m=0.4,occupancy_half_width_m=0.2)
        for i in range(1,61)]
    return N(valid=True,schema_version=1,frame_id='map',track_id='spielberg',
        source_opponent_id='red',source_lifecycle='TRACKED',
        source_confidence=confidence,source_hit_count=3,
        source_observation_time=N(sec=100,nanosec=0),samples=samples)


def proof(conf):
    return RedDynamicWitness(red(conf),now_s=100.1,track_id='spielberg',
        max_age_s=0.35,minimum_horizon_s=3.0,ego_radius_m=.3)


@pytest.mark.parametrize('confidence',[math.nan,math.inf,-math.inf,1.2])
def test_nonfinite_or_out_of_range_confidence_denied(confidence):
    with pytest.raises(ValueError,match='RED_SOURCE_NOT_CONFIRMED'):
        proof(confidence)
