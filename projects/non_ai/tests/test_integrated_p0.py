"""Offline tests of an isolated candidate, never a ROS/runtime qualification."""
import ast
import csv
import math
from pathlib import Path
from types import SimpleNamespace
import time

ROOT=Path(__file__).resolve().parents[1]/'src'
P=ROOT/'driving/algorithms/f1tenth_racing_control/f1tenth_racing_control'

class FakePublisher:
    def __init__(self): self.items=[]
    def publish(self,v): self.items.append(v)
class FakeClock:
    def now(self): return SimpleNamespace(nanoseconds=time.monotonic_ns(),to_msg=lambda:object())
class FakeDrive:
    def __init__(self):
        self.header=SimpleNamespace(stamp=None)
        self.drive=SimpleNamespace(steering_angle=42,acceleration=42)
def fake_guard_method(name):
    code=(P/'actuator_guard_node.py').read_text()
    tree=ast.parse(code)
    guard=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='IndependentActuatorGuard')
    method=next(n for n in guard.body if isinstance(n,ast.FunctionDef) and n.name==name)
    namespace={'math':math, 'time':time, 'json':__import__('json'), 'AckermannDriveStamped':FakeDrive}
    # Standalone method works with mocked self if it does not use ROS imports.
    exec(compile(ast.Module(body=[method],type_ignores=[]),'<isolated_guard_method>','exec'),namespace)
    return namespace[name]

def init_guard():
    g=SimpleNamespace(
       pending_future_intent=None,pending_future_deadline_steady_ns=None,
       pending_future_received_steady_ns=None,last_reason='RUNNING',
       last_certificate=True, contract=SimpleNamespace(state='RUNNING'),
       emergency_unrecoverable=False, collision=False,
       last_state_sequence=123,last_accepted_sequence=0,last_accepted_acceleration_mps2=0.2,
       last_state=SimpleNamespace(v=0.1,vx=0.1,vy=0,omega=0,a=0,accel=0),
       state_received_steady_ns=time.monotonic_ns(),
       startup_waiting_for_fresh_state=False,
       final_pub=FakePublisher(),get_clock=lambda:FakeClock(),
       get_logger=lambda:SimpleNamespace(error=lambda *args:None))
    return g

def test_guard_hard_brake_revokes_running_and_cannot_auto_clear():
    g=init_guard()
    fake_guard_method('_brake')(g,'GUARD_REJECT:STOP_CERTIFICATE_NOT_VERIFIED')
    assert g.contract.state=='SAFE_HOLD'
    assert g.emergency_unrecoverable
    assert not g.last_certificate
    assert g.last_reason.startswith('GUARD_REJECT:')
    assert len(g.final_pub.items)==1

def test_missing_state_while_parked_only_enters_fresh_state_recovery():
    g=init_guard();g.last_state.v=0;g.last_state.vx=0;g.last_accepted_acceleration_mps2=0
    fake_guard_method('_brake')(g,'STATE_WATCHDOG_TIMEOUT')
    assert g.contract.state=='SAFE_HOLD'
    assert not g.emergency_unrecoverable
    assert g.startup_waiting_for_fresh_state

def test_next_intent_is_blocked_when_hard_fault_latched():
    g=init_guard();g.emergency_unrecoverable=True
    g.start_ns=1
    fake_guard_method('on_intent')(g,SimpleNamespace())
    assert g.last_reason=='NOT_RUNNING_OR_ARMED'
    assert len(g.final_pub.items)==0

def test_missing_state_regression_enters_brake_with_no_pending():
    g=init_guard();g.previous_state=None
    g._brake=lambda why: fake_guard_method('_brake')(g,why)
    fake_guard_method('on_state')(g,SimpleNamespace(physics_sequence=123))
    assert g.emergency_unrecoverable
    assert g.contract.state=='SAFE_HOLD'


def test_longitudinal_full_iccas_reference_start_has_positive_requested_accel():
    from f1tenth_racing_control.core import PursuitReference,ModelAidedPursuit,EgoState,ControlConfig
    data=ROOT/'driving/algorithms/f1tenth_racing_raceline/data/iccas2025_r_a2_racing_trajectory.csv'
    with data.open() as f: rows=list(csv.DictReader(f))
    samples=tuple(SimpleNamespace(**{
        key: (int(value) if key=='sample_index' else float(value))
        for key,value in r.items()}) for r in rows)
    period=samples[-1].s_m+samples[-1].ds_to_next_m
    ref=PursuitReference(samples,period)
    cfg=ControlConfig(wheelbase_m=0.34,lookahead_base_m=0.7,lookahead_speed_gain_s=0.1,
        lookahead_min_m=0.6,lookahead_max_m=1.0,speed_feedback_gain_per_s=2,
        qualified_speed_cap_mps=20.0,steering_max_rad=0.5,
        steering_rate_max_rad_s=3,acceleration_max_mps2=40,braking_max_mps2=40,
        jerk_max_mps3=100,degraded_deceleration_mps2=2)
    model=ModelAidedPursuit(cfg)
    s=samples[0]
    stationary=model.compute(ref,EgoState(s.x_m,s.y_m,s.heading_rad,0.0),0,0,.025)
    assert not stationary.qualification_cap_active
    assert stationary.target_speed_mps > 5
    assert 0<stationary.acceleration_mps2<=2.5+1e-9
    # A target acceleration of -34.9 m/s² at the first sample must not
    # cancel positive velocity error at a standing start.
    assert s.target_accel_mps2 < -30


def test_upcoming_braking_constraint_is_not_ignored():
    from f1tenth_racing_control.core import PursuitReference,ModelAidedPursuit,EgoState,ControlConfig
    from types import SimpleNamespace
    points=((0.,0.),(10.,0.),(10.,10.),(0.,10.))
    speeds=(19.,5.,19.,19.)
    samples=tuple(SimpleNamespace(sample_index=i,s_m=i*10.,x_m=p[0],y_m=p[1],
      heading_rad=0.,curvature_per_m=0.,lateral_offset_m=0.,
      target_speed_mps=speeds[i],target_accel_mps2=(-30. if i==0 else 0.),ds_to_next_m=10.)
      for i,p in enumerate(points))
    ref=PursuitReference(samples,40)
    c=ControlConfig(wheelbase_m=.34,lookahead_base_m=.7,lookahead_speed_gain_s=.1,
       lookahead_min_m=.6,lookahead_max_m=1, speed_feedback_gain_per_s=2.,
       qualified_speed_cap_mps=20., steering_max_rad=.5,steering_rate_max_rad_s=3.,
       acceleration_max_mps2=10.,braking_max_mps2=2.,jerk_max_mps3=100.,
       degraded_deceleration_mps2=2.)
    d=ModelAidedPursuit(c).compute(ref,EgoState(0.,0.,0.,0.),0.,0.,.025)
    assert 5.<d.target_speed_mps<19.
    assert d.acceleration_mps2>0

def test_future_tick_39ms_is_deferred_without_any_actuator_publish():
    instant=time.monotonic_ns()
    def timestamp(value):
        return SimpleNamespace(sec=value//1_000_000_000,nanosec=value%1_000_000_000)
    g=init_guard()
    g.get_clock=lambda: SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=instant,to_msg=lambda:None))
    g.start_ns=instant-1_000_000_000
    g.contract=SimpleNamespace(state='RUNNING',previous_sequence=10)
    g.last_state_sequence=200
    g.last_state.physics_stamp=timestamp(instant-49_398_000)
    g.identity=('epoch','R_B','track','maphash')
    g.reference_id='ref'
    g.reference_revision=1
    g.pending_future_diagnostics_emitted=12
    g.get_logger=lambda:SimpleNamespace(warning=lambda *args:None)
    msg=SimpleNamespace(source_state_sequence=201,
        source_state_stamp=timestamp(instant-10_000_000),
        created_ros_stamp=timestamp(instant-20_000_000),
        expires_ros_stamp=timestamp(instant+60_000_000),
        sequence=11,source_id='R_B',run_epoch='epoch',track_id='track',
        map_sha256='maphash',active_reference_id='ref',active_revision=1)
    namespace={ 'ns':lambda x: x.sec*1_000_000_000+x.nanosec,
       'time':time, 'json':__import__('json') }
    code=(P/'actuator_guard_node.py').read_text(); guard=next(c for c in ast.parse(code).body
        if isinstance(c,ast.ClassDef) and c.name=='IndependentActuatorGuard')
    m=next(m for m in guard.body if isinstance(m,ast.FunctionDef) and m.name=='on_intent')
    exec(compile(ast.Module(body=[m],type_ignores=[]),'<on_intent>','exec'),namespace)
    namespace['on_intent'](g,msg)
    assert g.pending_future_intent is msg
    assert g.last_reason=='FUTURE_STATE_PENDING_NO_ACTUATION'
    assert not g.final_pub.items


def test_guard_static_h2h_interlock_preserved():
    code=(P/'actuator_guard_node.py').read_text()
    assert "raise ValueError('H2H_REQUIRES_INDEPENDENT_DYNAMIC_RED_GUARD')" in code
    assert "raise ValueError('H2H_ONLINE_SIM_ONLY')" in code
    assert 'witness.collision_free(z,z1,0.0,0.025)' in code
    assert 'not self.collision and not self.emergency_unrecoverable' in code
    assert "self.contract.state='SAFE_HOLD'" in code


def test_environment_authority_is_injected_to_all_runtime_consumers():
    launch=(ROOT/'platform/f1tenth_racing_bringup/launch/race_foundation.launch.py').read_text()
    for package in ('perception','prediction','overtaking'):
        node=(ROOT/f'driving/algorithms/f1tenth_racing_{package}/f1tenth_racing_{package}'/
              ('successor_node.py' if package=='overtaking' else 'node.py')).read_text()
        assert "declare_parameter('environment_file'" in node or 'declare_parameter("environment_file"' in node
        assert 'expected_track_id' in node and 'expected_map_sha256' in node
    assert '"environment_file": environment_file' in launch
    assert '"expected_map_sha256": expected_map_sha256' in launch
