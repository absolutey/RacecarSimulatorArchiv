"""Separately scheduled ROS process: independent fail-closed actuator VETO.

Never publishes a command or certifies any stopping trajectory. The simulator
requires both existing GuardStatus and this process's fresh witness. In a
fault, missing watchdog messages themselves expire the authorization.
"""
import json
import time

import rclpy
from control_msgs.msg import ActuatorGuardStatus, CarState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from f1tenth_racing_sim.guard_status_lease import GuardStatusLease
from .independent_watchdog_core import IndependentWatchdogCore


def _stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


class IndependentGuardWatchdog(Node):
    def __init__(self):
        super().__init__('independent_guard_watchdog0')
        self.declare_parameter('run_epoch','')
        self.declare_parameter('producer','R_B')
        epoch=str(self.get_parameter('run_epoch').value)
        producer=str(self.get_parameter('producer').value)
        if not epoch or producer not in ('R_B','FOUNDATION_BASELINE'):
            raise ValueError('WATCHDOG_RUN_IDENTITY_MISSING')
        self.epoch=epoch
        self.producer=producer
        self.core=IndependentWatchdogCore(guard_lease=GuardStatusLease(
            run_epoch=epoch,producer=producer,max_age_ns=100_000_000))
        self.verdict_sequence=0
        status_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        state_qos=QoSProfile(depth=16,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(ActuatorGuardStatus,'actuator_guard_status0',
                                 self.on_guard_status,status_qos)
        self.create_subscription(CarState,'state0',self.on_state,state_qos)
        self.create_subscription(Bool,'collision0',self.on_collision,state_qos)
        self.create_subscription(Bool,'vehicle_collision',self.on_collision,state_qos)
        self.verdict_pub=self.create_publisher(String,'independent_guard_watchdog_lease0',
                                              state_qos)
        self.create_timer(0.025,self.publish_verdict)

    def on_guard_status(self,msg):
        publishers=self.get_publishers_info_by_topic('/actuator_guard_status0')
        if len(publishers)!=1 or publishers[0].node_name!='actuator_guard0':
            self.core.guard_lease.revoke()
            return
        self.core.guard_lease.observe(msg,
            now_ros_ns=self.get_clock().now().nanoseconds,
            now_steady_ns=time.monotonic_ns())

    def on_state(self,msg):
        publishers=self.get_publishers_info_by_topic('/state0')
        if len(publishers)!=1 or publishers[0].node_name!='competition_simulator0':
            self.core.state=None
            return
        self.core.receive_state(sequence=msg.physics_sequence,
            stamp_ns=_stamp_ns(msg.physics_stamp),
            now_ros_ns=self.get_clock().now().nanoseconds,
            now_steady_ns=time.monotonic_ns())

    def on_collision(self,msg):
        # Positive collision latches; missing collision monitor messages do
        # not override Guard's own collision gate or physics staleness gate.
        self.core.on_collision(msg.data)

    def publish_verdict(self):
        ros_now=self.get_clock().now().nanoseconds
        steady_now=time.monotonic_ns()
        allowed,reason=self.core.decide(now_ros_ns=ros_now,
                                        now_steady_ns=steady_now)
        self.verdict_sequence+=1
        state=self.core.state
        guard=self.core.guard_lease.latest
        m={'schema':1,'run_epoch':self.epoch,'producer':self.producer,
           'sequence':self.verdict_sequence,'emitted_ros_ns':int(ros_now),
           'physics_sequence':int(state.sequence) if state else 0,
           'guard_accepted_sequence':int(guard.accepted_sequence) if guard else 0,
           'verdict':'ALLOW' if allowed else 'REVOKE','reason':reason}
        self.verdict_pub.publish(String(data=json.dumps(m,sort_keys=True)))


def main():
    rclpy.init()
    node=IndependentGuardWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
