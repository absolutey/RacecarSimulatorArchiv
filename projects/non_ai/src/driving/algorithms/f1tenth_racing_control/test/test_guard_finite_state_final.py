from types import SimpleNamespace
import math
import pytest
from f1tenth_racing_control.guard_state_integrity import finite_physics_state,_FIELDS


def good():
    return SimpleNamespace(**{key:0.0 for key in _FIELDS})


def test_good_state_all_fields_finite():
    assert finite_physics_state(good())


@pytest.mark.parametrize('key',_FIELDS)
def test_no_single_nonfinite_physics_field_can_authorize_lease(key):
    state=good();setattr(state,key,float('nan'))
    assert not finite_physics_state(state)
    setattr(state,key,float('inf'))
    assert not finite_physics_state(state)
    setattr(state,key,-float('inf'))
    assert not finite_physics_state(state)


def test_missing_sensor_or_broken_message_denied():
    state=good();delattr(state,'omega')
    assert not finite_physics_state(state)
    assert not finite_physics_state(None)
    assert not finite_physics_state(object())


def test_boolean_poison_denied():
    state=good();state.v=True
    assert not finite_physics_state(state)
