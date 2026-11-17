"""Real Frenet geometry/temporal occupancy integration, without ROS truth bypass."""
from dataclasses import replace
import pytest
from test_successor_q0 import fixture, prediction, manual_candidate, state
from f1tenth_racing_overtaking.successor_core import WireState


def _scenario(ahead, d=0.25, enabled=True):
    planner, nominal = fixture()
    planner.config = replace(planner.config, emergency_rejoin_enabled=enabled)
    old_pred=prediction(planner)
    old=planner.commit_candidate(manual_candidate(planner,nominal),old_pred,0.,10.).plan
    # Force the frozen plan to be rejected. Candidate rejoin must still pass
    # actual _rejoin_geometry, _trackability, _temporal_safety (not mocked).
    planner._fresh_frozen_safety=lambda *a,**k:(False,-0.1)
    ego=state(planner,0.8,d)
    red=prediction(planner,blue_s=0.8,ahead=ahead,now_s=10.1)
    return planner,nominal,old,red,ego


def test_moving_conflict_replans_only_new_safe_geometry():
    planner,nominal,old,red,ego=_scenario(ahead=8.0,d=0.25)
    action=planner.update(nominal,ego.x,ego.y,2.,red,10.1,ego.heading)
    assert action.action=='PUBLISH_OVERRIDE'
    assert action.plan.state==WireState.MERGE.value
    assert action.plan.reason=='EMERGENCY_AVOIDANCE:FRESH_R_D_SAFE_REJOIN'
    assert action.plan.revision>old.revision
    assert action.plan.source_prediction_id==red.identity
    assert action.plan.identity_digest!=old.identity_digest
    assert action.plan.minimum_predicted_separation_m>0
    assert action.plan.minimum_corridor_clearance_m>=planner.config.corridor_clearance_m


def test_same_conflict_without_optin_still_aborts():
    planner,nominal,old,red,ego=_scenario(ahead=8.0,enabled=False)
    result=planner.update(nominal,ego.x,ego.y,2.,red,10.1,ego.heading)
    assert result.action=='PUBLISH_ABORT_DEGRADE'
    assert result.plan.identity_digest==old.identity_digest


def test_red_near_ego_causes_abort_not_moving_rejoin():
    planner,nominal,old,red,ego=_scenario(ahead=0.3,d=0.0)
    result=planner.update(nominal,ego.x,ego.y,2.,red,10.1,ego.heading)
    assert result.action=='PUBLISH_ABORT_DEGRADE'
    assert result.plan.state==WireState.ABORT.value
    assert result.plan.identity_digest==old.identity_digest


def test_prediction_stale_does_not_make_new_rejoin():
    planner,nominal,old,red,ego=_scenario(ahead=8.0)
    result=planner.update(nominal,ego.x,ego.y,2.,red,10.9,ego.heading)
    assert result.action=='PUBLISH_ABORT_DEGRADE'
    assert result.plan.state==WireState.ABORT.value


def test_no_trackability_no_rejoin():
    planner,nominal,old,red,ego=_scenario(ahead=8.0)
    planner._rejoin_geometry=lambda *a,**k:(None,float('-inf'),None,'NO_CORRIDOR')
    result=planner.update(nominal,ego.x,ego.y,2.,red,10.1,ego.heading)
    assert result.action=='PUBLISH_ABORT_DEGRADE'
    assert result.plan.identity_digest==old.identity_digest


def test_invalid_flag_configuration_denied():
    planner,_=fixture()
    with pytest.raises(ValueError,match='boolean'):
        replace(planner.config, emergency_rejoin_enabled='true')
