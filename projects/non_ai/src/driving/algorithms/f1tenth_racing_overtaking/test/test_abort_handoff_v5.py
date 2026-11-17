"""ROS adapter contract: no automatic ABORT un-latch without parked evidence."""
from dataclasses import dataclass
import time
from types import SimpleNamespace as O
from f1tenth_racing_reference.node import NominalActiveReferenceNode
from f1tenth_racing_overtaking.successor_node import SuccessorOvertakingPlannerNode


@dataclass(frozen=True)
class Nominal:
    trajectory: object
    track_id: str = 'iccas2025'
    frame_id: str = 'map'
    status_reason: str = ''


class FakeReference:
    def __init__(self):
        self.messages=[]
        self._nominal=Nominal(O(samples=[O(x_m=float(i),y_m=0.,heading_rad=0.)
                                           for i in range(6)]))
        self._active=O(active_reference_valid=False,source_kind='NOMINAL')
        self._expired_plan=O(maneuver_state='ABORT',plan_id='sha256:abort42',plan_revision=3)
        self._plan=None
        self._recovery_radius=.35
        self._recovery_collision_ego=False
        self._recovery_collision_red=False
        self._recovery_collision_ego_steady_ns=time.monotonic_ns()
        self._recovery_collision_red_steady_ns=time.monotonic_ns()
        self._recovery_good_ticks=0
        self._recovery_last_sequence=-1
        self._recovery_evidence=''
        self._recovery_state=self.state(100)
        self._recovery_prediction=self.red()

    def get_clock(self):
        return O(now=lambda: O(nanoseconds=10_000_000_000))

    def get_logger(self):
        return O(warning=lambda _: None)

    def publish(self, item):
        self.messages.append(item)
        self._active=item

    @staticmethod
    def state(seq):
        return O(physics_sequence=seq,px=1.5,py=0.,yaw=0.,v=0.,vx=0.,vy=0.,omega=0.,
                 a=0.,accel=0.,physics_stamp=O(sec=9,nanosec=975_000_000))

    @staticmethod
    def red():
        red=O(schema_version=1,valid=True,source_lifecycle='TRACKED',
              source_opponent_id='red',source_confidence=.99,source_hit_count=9,
              track_id='iccas2025',frame_id='map',
              source_observation_time=O(sec=9,nanosec=950_000_000),samples=[])
        for j in range(1,31):
            t=j*.1
            red.samples.append(O(dt_s=t,x_m=10+t,y_m=0.,vx_mps=1.,vy_mps=0.,
                                 occupancy_half_length_m=.2,occupancy_half_width_m=.1))
        return red


def test_s0r_requires_three_distinct_parked_states():
    fake=FakeReference()
    for seq in (100,100,101):
        fake._recovery_state=fake.state(seq)
        NominalActiveReferenceNode._attempt_abort_recovery(fake)
    assert fake.messages==[]
    fake._recovery_state=fake.state(102)
    NominalActiveReferenceNode._attempt_abort_recovery(fake)
    assert len(fake.messages)==1
    assert fake.messages[0].status_reason == (
        'H2H_ABORT_RECOVERY_STATIONARY_REJOIN:sha256:abort42:3')
    assert fake._expired_plan is None


def test_s0r_denies_unknown_collision_and_near_red():
    fake=FakeReference()
    fake._recovery_collision_red=None
    NominalActiveReferenceNode._attempt_abort_recovery(fake)
    assert not fake.messages
    fake._recovery_collision_red=False
    for sm in fake._recovery_prediction.samples:sm.x_m=1.6
    NominalActiveReferenceNode._attempt_abort_recovery(fake)
    assert not fake.messages


def test_s0r_collision_latches_across_future_false_samples():
    fake=FakeReference()
    NominalActiveReferenceNode.receive_ego_collision(fake,O(data=True))
    NominalActiveReferenceNode.receive_ego_collision(fake,O(data=False))
    assert fake._recovery_collision_ego is True
    for i in range(5):
        fake._recovery_state=fake.state(100+i)
        NominalActiveReferenceNode._attempt_abort_recovery(fake)
    assert not fake.messages


def test_reject_replayed_or_unmatched_recovery_at_re_planner():
    seen=[]
    plan=O(identity_digest='sha256:expected',revision=5,state='ABORT')
    planner=O(track_id='iccas2025',frame_id='map',map_image_sha256='sha256:map',
              plan=plan,acknowledge_abort_recovery=lambda:seen.append(True))
    # Production R-E uses the ROS clock to fence predictions at the exact
    # ABORT recovery publication. The test double must supply the same API.
    recovery_ns = 10_000_000_000
    fake=O(planner=planner,nominal=None,awaiting_nominal_release=False,
           awaiting_abort_release=True,abort_release_started_ns=500,
           required_fresh_red_after_abort_ns=None,
           prediction=O(identity='old_pre_abort_red'),
           get_clock=lambda: O(now=lambda: O(nanoseconds=recovery_ns)))
    names=('sample_index','s_m','x_m','y_m','heading_rad','curvature_per_m',
           'lateral_offset_m','target_speed_mps','target_accel_mps2','ds_to_next_m')
    samples=[]
    for i in range(3):
        sample=O()
        for name in names:setattr(sample,name, 1 if name=='ds_to_next_m' else float(i))
        samples.append(sample)
    msg=O(active_reference_valid=True,source_kind='NOMINAL',sample_count=3,
          track_id='iccas2025',frame_id='map',map_image_sha256='sha256:map',
          status_reason='H2H_ABORT_RECOVERY_STATIONARY_REJOIN:sha256:old:5',
          samples=samples)
    SuccessorOvertakingPlannerNode.receive_reference(fake,msg)
    assert not seen and fake.awaiting_abort_release
    msg.status_reason='H2H_ABORT_RECOVERY_STATIONARY_REJOIN:sha256:expected:5'
    SuccessorOvertakingPlannerNode.receive_reference(fake,msg)
    assert seen==[True] and not fake.awaiting_abort_release
    assert fake.prediction is None
    # A successful release MUST establish a strictly fresh Red observation
    # fence. Existing/latching pre-ABORT predictions cannot pass as new.
    assert fake.required_fresh_red_after_abort_ns == recovery_ns
    from f1tenth_racing_overtaking.post_abort_freshness import (
        accept_post_abort_red_observation,
    )
    assert not accept_post_abort_red_observation(
        recovery_ns, fake.required_fresh_red_after_abort_ns, recovery_ns + 1)
    assert not accept_post_abort_red_observation(
        recovery_ns - 1, fake.required_fresh_red_after_abort_ns, recovery_ns + 1)
    assert accept_post_abort_red_observation(
        recovery_ns + 1, fake.required_fresh_red_after_abort_ns, recovery_ns + 2)


def test_s0r_rejects_stale_collision_heartbeat():
    fake=FakeReference()
    fake._recovery_collision_ego_steady_ns=time.monotonic_ns()-500_000_000
    NominalActiveReferenceNode._attempt_abort_recovery(fake)
    assert fake._recovery_evidence=='RECOVERY_COLLISION_NOT_CLEARED'
    assert not fake.messages
