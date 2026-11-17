"""New Red witness guards: future stamp, nonfinite confidence, replay poison."""
from dataclasses import replace
import pytest
from test_successor_q0 import fixture, prediction

@pytest.mark.parametrize('field,value,expected', [
    ('source_time_s', float('nan'), 'PREDICTION_SOURCE_NUMERICS_INVALID'),
    ('source_time_s', float('inf'), 'PREDICTION_SOURCE_NUMERICS_INVALID'),
    ('source_time_s', -float('inf'), 'PREDICTION_SOURCE_NUMERICS_INVALID'),
    ('source_time_s', 10.01, 'PREDICTION_FROM_FUTURE'),
    ('confidence', float('nan'), 'PREDICTION_SOURCE_NUMERICS_INVALID'),
    ('confidence', float('inf'), 'PREDICTION_SOURCE_NUMERICS_INVALID'),
    ('confidence', -0.01, 'PREDICTION_SOURCE_NUMERICS_INVALID'),
    ('confidence', 1.01, 'PREDICTION_SOURCE_NUMERICS_INVALID'),
    ('hit_count', -1, 'PREDICTION_SOURCE_NUMERICS_INVALID'),
    ('miss_count', -1, 'PREDICTION_SOURCE_NUMERICS_INVALID'),
    ('source_revision', -1, 'PREDICTION_SOURCE_NUMERICS_INVALID'),
    ('source_time_ns', 0, 'PREDICTION_SOURCE_NUMERICS_INVALID'),
])
def test_poisoned_red_witness_denied(field,value,expected):
    planner,_=fixture()
    p=replace(prediction(planner,now_s=10.0),**{field:value})
    assert planner._prediction_status(p,10.0)==expected

@pytest.mark.parametrize('now', [float('nan'),float('inf'),-float('inf')])
def test_bad_clock_never_qualifies(now):
    planner,_=fixture()
    assert planner._prediction_status(prediction(planner,now_s=10.0),now)=='PREDICTION_SOURCE_NUMERICS_INVALID'

def test_valid_fresh_red_remains_eligible_at_source_time():
    planner,_=fixture()
    assert planner._prediction_status(prediction(planner,now_s=10.0),10.0)=='PREDICTION_QUALIFIED'
