#!/usr/bin/env python3
"""Subscribe to same state stream with reliable depth=1, 16 and best-effort depth=1."""
import rclpy, json, time, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from control_msgs.msg import CarState
class Probe(Node):
 def __init__(self):
  super().__init__('f1tenth_forensic_state_qos_probe')
  self.out=open(sys.argv[1],'a',buffering=1)
  for depth,reliability,tag in ((1,ReliabilityPolicy.RELIABLE,'reliable1'),
                                (16,ReliabilityPolicy.RELIABLE,'reliable16'),
                                (1,ReliabilityPolicy.BEST_EFFORT,'best_effort1')):
   self.create_subscription(CarState,'state0',lambda m,t=tag:self.record(t,m),
                            QoSProfile(depth=depth,reliability=reliability))
 def record(self,tag,m):
  p=m.physics_stamp;stamp=p.sec*1000000000+p.nanosec
  t=time.time_ns()
  self.out.write(json.dumps({'tag':tag,'seq':m.physics_sequence,'physics_stamp_ns':stamp,
                             'callback_wall_ns':t,'age_ms':(t-stamp)/1e6})+'\n')
def main():
 rclpy.init();p=Probe()
 try:rclpy.spin(p)
 except KeyboardInterrupt:pass
 finally:p.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
