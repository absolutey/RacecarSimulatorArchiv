"""L6: ROS2 synchronized LiDAR/Odom -> MLP SHADOW proposals only, never actuator topics."""
from pathlib import Path
import hashlib,json,math
import numpy as np
SHADOW_ONLY=True
NO_ACTUATOR_TOPIC=True

def sha256(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def shadow_node(checkpoint,map_root,map_name):
    try:
        import rclpy,torch
        from rclpy.node import Node
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from .l3_driver import ActorCritic,action_from_normalized
        from .l1_observation import build_features
        from .l0_maps import load_course
    except ImportError as e:raise RuntimeError('ROS2_HUMBLE_TORCH_NAV_MSGS_REQUIRED') from e
    course=load_course(map_root,map_name)
    raw=torch.load(checkpoint,map_location='cpu',weights_only=True)
    if raw.get('qualified_for_ros_actuation') is not False:raise RuntimeError('INVALID_CHECKPOINT_AUTHORITY')
    net=ActorCritic();net.load_state_dict(raw['model']);net.eval()
    policy_sha=sha256(checkpoint)
    def stamp_sec(stamp):return stamp.sec+stamp.nanosec*1e-9
    class Shadow(Node):
        def __init__(self):
            super().__init__('deep_racing_solo_v1_shadow')
            self.publisher=self.create_publisher(String,'/deep_racing/shadow_proposal',10)
            self.scan_sub=self.create_subscription(LaserScan,'/scan',self.on_scan,10)
            self.odom_sub=self.create_subscription(Odometry,'/odom',self.on_odom,10)
            self.odom=None
            self.get_logger().info('SHADOW_ONLY: publishes std_msgs/String only. NO DRIVE OR ACTUATOR AUTHORITY')
        def on_odom(self,msg):self.odom=msg
        def on_scan(self,msg):
            result={'mode':'SHADOW_ONLY','policy_sha256':policy_sha,'actuation_authorized':False,'map':map_name}
            try:
                if self.odom is None:raise ValueError('NO_ODOMETRY')
                od=self.odom
                # fail-closed: state/scan must have close timestamps; odom must be map-frame coordinates
                if od.header.frame_id != 'map':raise ValueError('ODOM_NOT_IN_MAP_FRAME')
                if msg.header.frame_id=='' or abs(stamp_sec(od.header.stamp)-stamp_sec(msg.header.stamp))>.1:
                    raise ValueError('STALE_OR_UNSYNC_SENSOR')
                q=od.pose.pose.orientation
                siny=2*(q.w*q.z+q.x*q.y);cosy=1-2*(q.y*q.y+q.z*q.z)
                yaw=math.atan2(siny,cosy)
                x=float(od.pose.pose.position.x);y=float(od.pose.pose.position.y)
                velocity=math.hypot(od.twist.twist.linear.x,od.twist.twist.linear.y)
                ranges=np.asarray(msg.ranges,dtype=float)
                if np.isnan(ranges).any() or (ranges<0).any():raise ValueError('BAD_SCAN')
                s,_,_=course.projection(x,y)
                features=build_features({'x':x,'y':y,'yaw':yaw,'v':velocity,
                               'yaw_rate':float(od.twist.twist.angular.z),'scan':ranges},course,s,2.)
                with torch.no_grad():
                    a=net.deterministic(torch.as_tensor(features)[None,:])[0].numpy()
                steer,target=action_from_normalized(a)
                result.update({'state':'PROPOSED_NOT_AUTHORIZED','steering_rad':steer,
                     'speed_target_mps':target,'sensor_age_s':abs(stamp_sec(od.header.stamp)-stamp_sec(msg.header.stamp))})
            except Exception as e:result.update({'state':'HOLD','reason':str(e)})
            out=String();out.data=json.dumps(result);self.publisher.publish(out)
    rclpy.init();node=Shadow()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
