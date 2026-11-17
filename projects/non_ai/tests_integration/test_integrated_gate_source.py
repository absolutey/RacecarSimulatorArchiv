"""Offline gate verification with NO ROS 2 import or fabricated runtime claims."""
import ast
import hashlib
import json
from pathlib import Path
import pytest
import yaml
from f1tenth_racing_sim.physics.vehicle import State
from f1tenth_racing_sim.safety.stopping_envelope import StoppingEnvelope

ROOT=Path(__file__).resolve().parents[1]

def profile_function():
    path=ROOT/'src/platform/f1tenth_racing_bringup/launch/race_foundation.launch.py'
    tree=ast.parse(path.read_text())
    node=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='validate_execution_profile')
    code=compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec')
    env={};exec(code,env)
    return env['validate_execution_profile']


def kwargs(profile,solo=True,kind='mpc',trajectory='scp',red=False):
    return dict(profile=profile,solo=solo,use_r_b0=True,
        controller_kind=kind,trajectory_candidate=trajectory,
        use_opponent_perception=red,use_opponent_prediction=red,use_overtaking=red)


def test_solo_mpc_profile_valid():
    f=profile_function()
    assert f(**kwargs('SOLO_MPC_EXPERIMENTAL'))=='SOLO_MPC_EXPERIMENTAL'


def test_h2h_requires_complete_pipeline():
    f=profile_function()
    assert f(**kwargs('H2H_MPC_DYNAMIC_EXPERIMENTAL',solo=False,red=True))
    with pytest.raises(RuntimeError):f(**kwargs('H2H_MPC_DYNAMIC_EXPERIMENTAL',solo=False,red=False))
    with pytest.raises(RuntimeError):f(**kwargs('SOLO_MPC_EXPERIMENTAL',solo=False,red=True))
    with pytest.raises(RuntimeError):f(**kwargs('H2H_MPC_DYNAMIC_EXPERIMENTAL',solo=True,red=True))


def test_spielberg_profile_requires_baseline():
    f=profile_function()
    assert f(**kwargs('SOLO_SPIELBERG_MPC_PRELIMINARY',trajectory='baseline'))
    with pytest.raises(RuntimeError):f(**kwargs('SOLO_SPIELBERG_MPC_PRELIMINARY',trajectory='scp'))


def test_spielberg_source_integrity_and_unqualified_status():
    data=ROOT/'src/driving/algorithms/f1tenth_racing_raceline/data'
    policy=json.loads((ROOT/'src/platform/f1tenth_racing_reference/config/spielberg_nominal_reference.json').read_text())
    env=yaml.safe_load((ROOT/'src/environment/f1tenth_racing_sim/config/spielberg_environment.yaml').read_text())
    assert policy['map_image_sha256']==env['map']['image_sha256']
    assert policy['track_id']=='spielberg'
    candidate=policy['candidates']['baseline']
    meta=json.loads((data/candidate['metadata']).read_text())
    assert meta['qualification_state']=='OFFLINE_CENTERLINE_PRELIMINARY_UNQUALIFIED'
    assert meta['runtime_tracking_qualified'] is False
    assert meta['trajectory_id']==candidate['source_trajectory_id']
    assert hashlib.sha256((data/candidate['csv']).read_bytes()).hexdigest()==meta['csv_sha256']


class FreeMap:
    width=2000;height=2000;resolution=.1
    origin_x=-50.;origin_y=-50.;origin_yaw=0.;flip_y=False
    def rectangle_is_free(self,row0,row1,col0,col1):return True


def test_real_stopping_envelope_calls_red_checker():
    env=yaml.safe_load((ROOT/'src/environment/f1tenth_racing_sim/config/competition_environment.yaml').read_text())
    stop=StoppingEnvelope(env['dynamics'],FreeMap(),env['collision']['footprint_relative_to_base_link'])
    stationary=State(x=0.,y=0.,yaw=0.,speed=0.)
    called=[]
    def checker(prev,z,t0,t1):
        called.append((t0,t1))
        return False
    verdict=stop.evaluate(stationary,0.,0.,dynamic_checker=checker)
    assert not verdict.qualified
    assert verdict.reason=='RED_DYNAMIC_INITIAL_COLLISION_OR_UNPROVEN'
    assert len(called)==1
    called.clear()
    verdict=stop.evaluate(stationary,0.,0.,dynamic_checker=lambda a,b,c,d: True)
    assert verdict.qualified


def test_spielberg_nominal_explicit_preliminary_gate():
    # Ament is part of the ROS runtime, not available in the offline container.
    import pytest
    pytest.importorskip('ament_index_python')
    from f1tenth_racing_reference import select_nominal
    from f1tenth_racing_raceline import load_racing_trajectory
    data=ROOT/'src/driving/algorithms/f1tenth_racing_raceline/data'
    policy=json.loads((ROOT/'src/platform/f1tenth_racing_reference/config/spielberg_nominal_reference.json').read_text())
    entry=policy['candidates']['baseline']
    trajectory=load_racing_trajectory(data/entry['csv'],data/entry['metadata'])
    kwargs=dict(expected_track_id='spielberg',expected_map_image_sha256=policy['map_image_sha256'],
                expected_source_trajectory_id=entry['source_trajectory_id'])
    rejected=select_nominal(trajectory,**kwargs)
    assert not rejected.active_reference_valid
    admitted=select_nominal(trajectory,allow_preliminary_spielberg=True,**kwargs)
    assert admitted.active_reference_valid
    assert admitted.source_qualification=='OFFLINE_CENTERLINE_PRELIMINARY_UNQUALIFIED'
    assert admitted.source_runtime_tracking_qualified is False
    # Identity spoofing must never relax qualified ICCAS candidate or foreign track.
    foreign=select_nominal(trajectory,allow_preliminary_spielberg=True,
        expected_track_id='iccas2025',expected_map_image_sha256=policy['map_image_sha256'])
    assert not foreign.active_reference_valid


def test_guard_checks_first_25ms_red_sweep_in_source():
    source=(ROOT/'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/actuator_guard_node.py').read_text()
    assert 'witness.collision_free(z,z1,0.0,0.025)' in source
    assert 'begin_s+0.025,end_s+0.025' in source
