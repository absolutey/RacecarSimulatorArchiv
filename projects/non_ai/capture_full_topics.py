#!/usr/bin/env python3
"""Read-only, exact ROS msg.data and state/intent callback receipt recorder.

Records diagnostic data ONLY; publishes no topics and invokes no services.
Requires ROS Humble + F1TENTH NEXT overlay already sourced.
"""
import argparse
import json
import time
from pathlib import Path


def ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from builtin_interfaces.msg import Time
    from control_msgs.msg import CarState, ActuatorIntent, ActuatorGuardStatus

    out=Path(args.out).expanduser().resolve()
    out.mkdir(parents=True,exist_ok=True)
    qos=QoSProfile(depth=80, reliability=ReliabilityPolicy.BEST_EFFORT)
    transient=QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class Recorder(Node):
        def __init__(self):
            super().__init__('f1tenth_next_readonly_rca2_recorder')
            self.out={}
            self.counts={}
            for stem in ('compute_full','states','intents','guard','start'):
                self.out[stem]=open(out/(stem+'.jsonl'),'w',encoding='utf-8',buffering=1)
                self.counts[stem]=0
            self.create_subscription(String, '/r_b_compute_stages0', self.stages,qos)
            self.create_subscription(CarState, '/state0',self.state,qos)
            self.create_subscription(ActuatorIntent,'/actuator_intent0',self.intent,qos)
            self.create_subscription(ActuatorGuardStatus,'/actuator_guard_status0',self.guard,transient)
            self.create_subscription(Time,'/race_start_time',self.start,transient)
        def emit(self,key,record):
            record['recv_steady_ns']=time.monotonic_ns()
            record['recv_wall_ns']=time.time_ns()
            self.out[key].write(json.dumps(record,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')
            self.counts[key]+=1
        def stages(self,msg):
            try:
                data=json.loads(msg.data)    # FULL text: avoids ros2 topic echo abbreviation
                record={'parsed':True,'data':data}
            except (ValueError,TypeError):
                record={'parsed':False,'data':msg.data}
            self.emit('compute_full',record)
        def state(self,msg):
            self.emit('states',{'seq':int(msg.physics_sequence),
                'physics_stamp_ns':ns(msg.physics_stamp),'v':float(msg.v),
                'accel':float(msg.accel),'steer':float(msg.steer)})
        def intent(self,msg):
            self.emit('intents',{'seq':int(msg.sequence),
                'source_state_seq':int(msg.source_state_sequence),
                'source_state_stamp_ns':ns(msg.source_state_stamp),
                'created_ros_ns':ns(msg.created_ros_stamp),
                'expires_ros_ns':ns(msg.expires_ros_stamp),
                'mode':str(msg.candidate_mode),'accel':float(msg.acceleration_mps2),
                'steer':float(msg.steering_rad),'reference':str(msg.active_reference_id)})
        def guard(self,msg):
            self.emit('guard',{'mode':str(msg.mode),'reason':str(msg.reason),
                'last_accepted_sequence':int(msg.last_accepted_sequence),
                'ready':bool(msg.ready),'certificate_verified':bool(msg.certificate_verified),
                'emergency_unrecoverable':bool(msg.emergency_unrecoverable)})
        def start(self,msg):
            self.emit('start',{'start_ros_ns':ns(msg)})
        def close(self):
            for f in self.out.values():f.close()
            (out/'subscriber_summary.json').write_text(
                json.dumps({'counts':self.counts,'read_only':True},indent=2)+'\n')

    rclpy.init()
    node=Recorder()
    print('FULL_TOPIC_CAPTURE_READY',flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        try:node.destroy_node()
        except (KeyboardInterrupt,Exception):pass
        if rclpy.ok():rclpy.shutdown()
        print('FULL_TOPIC_CAPTURE_FINALIZED',flush=True)

if __name__=='__main__':main()
