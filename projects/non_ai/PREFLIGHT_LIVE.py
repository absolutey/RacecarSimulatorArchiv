#!/usr/bin/env python3
"""Only observes pre-start ROS state; never grants safety authority."""
import json,sys,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from control_msgs.msg import ActiveTargetTrajectory,ActuatorGuardStatus,CarState
from std_msgs.msg import String
from f1tenth_racing_control.rb_state_qos_contract import evaluate_rb_state_path

class Probe(Node):
    def __init__(self):
        super().__init__('rab_prestart_observer')
        q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference=None; self.guard=None; self.controller=None; self.states=[]
        self.rb_worker=None
        self.create_subscription(ActiveTargetTrajectory,'/active_target_trajectory',self.on_ref,q)
        self.create_subscription(ActuatorGuardStatus,'/actuator_guard_status0',self.on_guard,q)
        self.create_subscription(String,'/controller_execution_receipt',self.on_controller,q)
        self.create_subscription(String,'/control_worker_metrics',self.on_rb_worker,
            QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(CarState,'/state0',self.on_state,QoSProfile(depth=16,reliability=ReliabilityPolicy.RELIABLE))
    def on_ref(self,x): self.reference=x
    def on_guard(self,x): self.guard=x
    def on_controller(self,x):
        try:self.controller=json.loads(x.data)
        except ValueError:pass
    def on_rb_worker(self,x):
        try:self.rb_worker=json.loads(x.data)
        except (ValueError,TypeError):pass
    def rb_state_path(self):
        def endpoints(items):
            return [{'name':e.node_name,
                     'reliability':e.qos_profile.reliability.name,
                     'durability':e.qos_profile.durability.name,
                     'depth':int(e.qos_profile.depth)} for e in items]
        try:
            return evaluate_rb_state_path(
                endpoints(self.get_subscriptions_info_by_topic('/state0')),
                endpoints(self.get_publishers_info_by_topic('/state0')),
                self.rb_worker)
        except Exception as exc:
            # Graph introspection failure is NOT evidence of valid DDS QoS.
            # A blocked START is preferable to repeating the 357ms latch.
            return False, {'rb_state_graph_probe_error': type(exc).__name__,
                           'rb_state_graph_verified': False, 'pass': False}
    def on_state(self,x):
        now=self.get_clock().now().nanoseconds
        stamp=int(x.physics_stamp.sec)*10**9+int(x.physics_stamp.nanosec)
        self.states.append((int(x.physics_sequence),now-stamp,time.monotonic_ns()))
        self.states=self.states[-16:]
    def verdict(self):
        ref=self.reference;g=self.guard;c=self.controller;s=self.states
        facts={'ref_received':ref is not None,'guard_received':g is not None,
               'controller_received':c is not None,'recent_state_count':len(s),
               'guard_mode':(g.mode if g else None),
               'guard_reason':(g.reason if g else None),
               'reference_id':(ref.active_reference_id if ref else None),
               'guard_reference_id':(g.active_reference_id if g else None),
               'controller_event':(c.get('event') if c else None),
               'controller_reference_id':(c.get('active_reference_id') if c else None)}
        rb_ok, rb_facts = self.rb_state_path()
        facts.update(rb_facts)
        if not ref or not g or not c or len(s)<5:return False,facts
        ages=[age for _,age,_ in s[-5:]]
        sequences=[seq for seq,_,_ in s[-5:]]
        monotonic_ok=all(a<b for a,b in zip(sequences,sequences[1:]))
        fresh=all(0<=age<=100_000_000 for age in ages)
        recent=time.monotonic_ns()-s[-1][2]<=100_000_000
        ok=(bool(ref.active_reference_valid) and bool(ref.active_reference_id) and
            ref.sample_count>=3 and int(ref.active_revision)>=1 and
            g.mode=='ARMED' and bool(g.ready) and not bool(g.emergency_unrecoverable) and
            g.active_reference_id==ref.active_reference_id and
            c.get('event')=='REFERENCE_ADOPTED' and
            c.get('active_reference_id')==ref.active_reference_id and
            c.get('active_revision')==ref.active_revision and
            monotonic_ok and fresh and recent and rb_ok)
        facts.update({'consecutive_state_sequence':monotonic_ok,
                      'physics_ages_ns':ages,'latest_state_steady_recent':recent,
                      'pass':ok})
        return ok,facts

def main():
    dest=Path(sys.argv[1]);rclpy.init()
    p=Probe();finish=time.monotonic()+12.;facts={}
    try:
        while time.monotonic()<finish:
            rclpy.spin_once(p,timeout_sec=0.1)
            ok,facts=p.verdict()
            if ok:break
        dest.write_text(json.dumps(facts,indent=2,sort_keys=True)+'\n')
        print('LIVE_PREFLIGHT_'+('PASS' if ok else 'FAIL'),dest)
        return 0 if ok else 2
    finally:
        p.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
