import json
import math
import time
import threading
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from control_msgs.msg import CarState, GuardedActuatorCommand, ActuatorGuardStatus
from .guarded_actuation_contract import verify_guarded_actuation, minimum_risk_brake_at_tick
from .guard_status_lease import GuardStatusLease
from .external_watchdog_lease import ExternalWatchdogLease
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster

from .physics.vehicle import Dynamics, State
from .safety_deadman import stale_brake_target, stop_crossed_zero, hold_stationary
from .sensing.lidar import scan_occupancy
from .sensing.raycast import ray_box_ranges
from .world.collision import footprints_overlap, swept_vehicle_disk_overlap
from .world.swept_collision import swept_footprint_collision, footprint_collision
from .world.map import OccupancyMap
from .asset_integrity import require_expected_source_fingerprints
from .world.track import load_centerline, pose_at_s


def quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class Simulator(Node):
    def __init__(self):
        super().__init__("competition_simulator")
        share = Path(get_package_share_directory("f1tenth_racing_sim"))
        self.declare_parameter("config_file", str(share / "config/competition_environment.yaml"))
        self.declare_parameter("map_file", str(share / "maps/iccas2025/iccas2025_map.pgm"))
        self.declare_parameter("centerline_file", str(share / "maps/iccas2025/iccas2025_centerline.csv"))
        self.declare_parameter("car_id", 0)
        self.declare_parameter("guard_run_epoch", "")
        self.declare_parameter("guard_producer", "R_B")
        self.declare_parameter("independent_watchdog_required", True)
        self.declare_parameter("initial_track_s_m", 0.0)
        self.declare_parameter("dynamic_opponent_id", -1)
        self.declare_parameter("publish_vehicle_collision", False)
        self.declare_parameter("publish_environment", True)
        self.declare_parameter("publish_lidar", True)
        self.declare_parameter("command_timeout_s", rclpy.Parameter.Type.DOUBLE)
        config = yaml.safe_load(Path(self.get_parameter("config_file").value).read_text())
        self.config = config
        self.car_id = self.get_parameter("car_id").value
        suffix = str(self.car_id)
        self.base_frame = "base_link" + suffix
        self.laser_frame = config["lidar"]["frame_car" + suffix]
        require_expected_source_fingerprints(
            config, self.get_parameter("map_file").value,
            self.get_parameter("centerline_file").value)
        self.map = OccupancyMap(self.get_parameter("map_file").value, config["map"])
        self.centerline = load_centerline(self.get_parameter("centerline_file").value)
        x, y, yaw = pose_at_s(self.centerline, self.get_parameter("initial_track_s_m").value)
        self.state = State(x=x, y=y, yaw=yaw)
        self.opponent_state = None
        self.previous_opponent_state = None
        self.command_origin_identity = None
        # Serializes asynchronous status, command, reset, and physics callbacks.
        self.actuation_lock = threading.RLock()
        self.command_stamp_ns = None
        self.latest_guard_intent_receipts = {}
        self.dynamics = Dynamics(config["dynamics"])
        self.requested_steering = config["actuation"]["initial_steering_angle_rad"]
        self.requested_acceleration = config["actuation"]["initial_acceleration_m_s2"]
        self.command_timeout_ns = round(self.get_parameter("command_timeout_s").value * 1_000_000_000)
        self.last_command_ns = None
        self.last_command_steady_ns = None
        self.physics_sequence = 0
        self.command_sequence = 0
        self.applied_command_sequence = 0
        self.guard_required = self.car_id == 0
        self.guard_run_epoch=str(self.get_parameter("guard_run_epoch").value)
        self.guard_producer=str(self.get_parameter("guard_producer").value)
        self.guard_intent_last_sequence=0
        if self.guard_required and not self.guard_run_epoch:
            raise ValueError('SIM_EGO_REQUIRES_EXPLICIT_GUARD_EPOCH')
        self.guard_status_lease=(GuardStatusLease(run_epoch=self.guard_run_epoch,
            producer=self.guard_producer) if self.guard_required else None)
        # Separate ROS PROCESS veto (not a stop certificate or a second
        # command authority). Never enable an Ego bypass via launch.
        self.watchdog_required=(self.guard_required and
            bool(self.get_parameter('independent_watchdog_required').value))
        if self.guard_required and not self.watchdog_required:
            raise ValueError('EGO_EXTERNAL_WATCHDOG_MANDATORY')
        self.external_watchdog_lease=(ExternalWatchdogLease(
            run_epoch=self.guard_run_epoch,producer=self.guard_producer)
            if self.watchdog_required else None)

        sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
        # state0 is a safety-critical exact-tick input for the independent
        # Guard, unlike latest-only odometry/scan displays. Keep a bounded
        # RELIABLE writer history for short callback stalls (40 Hz => 25ms).
        # Depth=16 is an UNQUALIFIED scheduling hypothesis: require runtime
        # sequence-loss, latency, and stop-proof verification before release.
        state_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=16,
                               reliability=ReliabilityPolicy.RELIABLE)
        command_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if self.guard_required:
            # No raw Ackermann subscription for Ego: a forged/unmatched raw
            # command may be visible to tools, but cannot refresh actuation.
            self.command_sub=self.create_subscription(
                GuardedActuatorCommand,'guarded_actuator_cmd0',
                self.command_guarded,command_qos)
            # Independent simulator-process veto: a stale/crashed Guard can
            # no longer keep an earlier CERTIFIED actuator lease alive.
            self.guard_status_sub=self.create_subscription(
                ActuatorGuardStatus,'actuator_guard_status0',
                self.receive_guard_status,latched_qos)
            # Watchdog publishes RELIABLE/VOLATILE. Subscriber MUST NOT
            # request TRANSIENT_LOCAL (DDS durability mismatch => no data).
            self.external_watchdog_sub=self.create_subscription(
                String,'independent_guard_watchdog_lease0',
                self.receive_external_watchdog,state_qos)
        else:
            self.command_sub = self.create_subscription(
                AckermannDriveStamped, config["actuation"]["command_topic_car" + suffix],
                self.command, command_qos)
        if self.car_id == 0:
            self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "initialpose", self.reset_from_initial_pose, sensor_qos)
        else:
            self.pose_sub = self.create_subscription(PoseStamped, "goal_pose", self.reset_from_goal_pose, sensor_qos)
        self.state_pub = self.create_publisher(CarState, "state" + suffix, state_qos)
        self.odom_pub = self.create_publisher(Odometry, "odom" + suffix, sensor_qos)
        if self.get_parameter("publish_lidar").value:
            self.scan_pub = self.create_publisher(LaserScan, config["lidar"]["topic_car" + suffix], sensor_qos)
        self.collision_pub = self.create_publisher(Bool, config["collision"]["topic_car" + suffix], sensor_qos)
        self.deadman_pub = self.create_publisher(Bool, "actuator_deadman" + suffix, sensor_qos)
        self.applied_receipt_pub = self.create_publisher(String, "actuator_applied_receipt" + suffix, sensor_qos)
        if self.car_id == 0:
            self.create_subscription(String,'actuator_guard_intent_receipt0',
                                     self.receive_guard_intent_receipt,sensor_qos)
        opponent_id = self.get_parameter("dynamic_opponent_id").value
        if opponent_id >= 0:
            self.opponent_sub = self.create_subscription(CarState, "state" + str(opponent_id), self.receive_opponent_state, sensor_qos)
        if self.get_parameter("publish_vehicle_collision").value:
            self.vehicle_collision_pub = self.create_publisher(Bool, "vehicle_collision", sensor_qos)
        if self.get_parameter("publish_environment").value:
            self.map_pub = self.create_publisher(OccupancyGrid, "map", latched_qos)
            self.path_pub = self.create_publisher(PathMsg, "centerline", latched_qos)
        self.tf = TransformBroadcaster(self)
        timers = ReentrantCallbackGroup()
        self.physics_timer = self.create_timer(1.0 / config["simulation"]["physics_rate_hz"], self.physics_step, callback_group=timers)
        if hasattr(self, "scan_pub"):
            self.scan_timer = self.create_timer(1.0 / config["lidar"]["scan_rate_hz"], self.publish_scan, callback_group=timers)
        if hasattr(self, "map_pub"):
            self.map_timer = self.create_timer(1.0 / config["simulation"]["map_and_path_publish_rate_hz"], self.publish_map_and_path, callback_group=timers)
        self.get_logger().info(f"car{suffix} initialized from canonical environment authority")

    def receive_guard_intent_receipt(self,message):
        # OBSERVABILITY ONLY: intentionally not an actuation authority.
        # A final Ackermann command remains subject to the Guard publisher
        # graph check and simulator deadman. Cross-topic order is not assumed.
        try:
            record=json.loads(message.data)
            stamp=int(record['command_stamp_ns'])
            if stamp>0 and record.get('run_epoch') and record.get('intent_sequence') is not None:
                self.latest_guard_intent_receipts[stamp]=record
                if len(self.latest_guard_intent_receipts)>128:
                    for k in sorted(self.latest_guard_intent_receipts)[:-64]:
                        del self.latest_guard_intent_receipts[k]
        except (ValueError,TypeError,KeyError):
            return

    def receive_guard_status(self, message):
        with self.actuation_lock:
            infos=self.get_publishers_info_by_topic('/actuator_guard_status0')
            if len(infos)!=1 or infos[0].node_name!='actuator_guard0':
                self.guard_status_lease.latest=None
                self._revoke_guarded_lease('GUARD_STATUS_PUBLISHER_MISMATCH')
                return
            accepted=self.guard_status_lease.observe(message,
                now_ros_ns=self.get_clock().now().nanoseconds,
                now_steady_ns=time.monotonic_ns())
            live,_=self.guard_status_lease.permits_certified(
                now_ros_ns=self.get_clock().now().nanoseconds,
                now_steady_ns=time.monotonic_ns())
            if not accepted or not live:
                self._revoke_guarded_lease('GUARD_STATUS_REVOKED')

    def _revoke_guarded_lease(self, reason):
        # No command is issued by this method. The simulator-local physics
        # deadman independently applies deceleration at the next physics tick.
        if self.command_origin_identity is not None and \
                self.command_origin_identity.get('authority_mode')=='CERTIFIED':
            self.last_command_steady_ns=None
            self.last_command_ns=None

    def receive_external_watchdog(self, message):
        with self.actuation_lock:
            publishers=self.get_publishers_info_by_topic(
                '/independent_guard_watchdog_lease0')
            if (len(publishers)!=1 or
                    publishers[0].node_name!='independent_guard_watchdog0'):
                self.external_watchdog_lease.revoke()
                self._revoke_guarded_lease('EXTERNAL_WATCHDOG_PUBLISHER_MISMATCH')
                return
            accepted=self.external_watchdog_lease.observe(message.data,
                now_ros_ns=self.get_clock().now().nanoseconds,
                now_steady_ns=time.monotonic_ns())
            allowed,_=self.external_watchdog_lease.permits_certified(
                now_ros_ns=self.get_clock().now().nanoseconds,
                now_steady_ns=time.monotonic_ns(),
                physics_sequence=self.physics_sequence)
            if not accepted or not allowed:
                self._revoke_guarded_lease('EXTERNAL_WATCHDOG_REVOKED')

    def command_guarded(self, message):
        with self.actuation_lock:
            return self._command_guarded_locked(message)

    def _command_guarded_locked(self, message):
        if message.authority_mode=='CERTIFIED':
            live,reason=self.guard_status_lease.permits_certified(
                now_ros_ns=self.get_clock().now().nanoseconds,
                now_steady_ns=time.monotonic_ns())
            if not live:
                self.get_logger().error('ATOMIC_GUARD_STATUS_REJECT:'+reason)
                return
            external_live,external_reason=self.external_watchdog_lease.permits_certified(
                now_ros_ns=self.get_clock().now().nanoseconds,
                now_steady_ns=time.monotonic_ns(),
                physics_sequence=self.physics_sequence)
            if not external_live:
                self.get_logger().error('ATOMIC_EXTERNAL_WATCHDOG_REJECT:'+external_reason)
                return
        infos=self.get_publishers_info_by_topic('/guarded_actuator_cmd0')
        if len(infos)!=1 or infos[0].node_name!='actuator_guard0':
            self.get_logger().error('ATOMIC_GUARD_PUBLISHER_MISMATCH')
            return
        limits=self.config['dynamics']['limits']
        accepted,reason=verify_guarded_actuation(
            message,now_ros_ns=self.get_clock().now().nanoseconds,
            physics_sequence=self.physics_sequence,
            prior_intent_sequence=self.guard_intent_last_sequence,
            run_epoch=self.guard_run_epoch,
            track_id=self.config['environment_identity']['active_track'],
            map_sha256=self.config['map']['image_sha256'],
            producer=self.guard_producer,actual_speed_mps=self.state.speed,
            steer_limit_rad=float(limits['steering_angle_abs_max_rad']),
            accel_limit_mps2=float(limits['acceleration_max_m_s2']),
            brake_limit_mps2=float(limits['deceleration_max_m_s2']))
        if not accepted:
            self.get_logger().error('ATOMIC_GUARD_REJECT:'+reason)
            return
        self.requested_steering=float(message.steering_rad)
        # Keep the original Guard command for lineage. For emergency mode
        # the *physics tick* recomputes safe deceleration from local speed;
        # this stored number is never blindly applied across a zero crossing.
        self.requested_acceleration=float(message.acceleration_mps2)
        self.command_stamp_ns=(int(message.stamp.sec)*1_000_000_000+
                               int(message.stamp.nanosec))
        self.last_command_ns=self.get_clock().now().nanoseconds
        self.last_command_steady_ns=time.monotonic_ns()
        self.command_sequence+=1
        if reason=='CERTIFIED':
            self.guard_intent_last_sequence=int(message.intent_sequence)
        self.command_origin_identity={
            'run_epoch':message.run_epoch,
            'intent_sequence':int(message.intent_sequence) if reason=='CERTIFIED' else None,
            'source_state_sequence':int(message.source_state_sequence),
            'source_id':message.source_id,
            'active_reference_id':message.active_reference_id,
            'active_revision':int(message.active_revision),
            'command_stamp_ns':self.command_stamp_ns,
            'authority_mode':reason,
            'proof_verified_by_guard':bool(message.stop_certificate_verified),
        }

    def command(self, message):
        with self.actuation_lock:
            return self._command_locked(message)

    def _command_locked(self, message):
        if self.guard_required:
            self.get_logger().error('EGO_RAW_COMMAND_FORBIDDEN')
            return
        # A graph identity check is a fail-closed simulation gate, not ROS 2
        # authentication. DDS permission policies are required for adversarial
        # publisher security. Never accept a competing final publisher.
        infos = self.get_publishers_info_by_topic(
            self.config["actuation"]["command_topic_car" + str(self.car_id)])
        expected = "actuator_guard0" if self.car_id == 0 else "red_baseline_controller"
        if len(infos) != 1 or infos[0].node_name != expected:
            self.get_logger().error("ACTUATOR_AUTHORITY_MISMATCH")
            return
        # Simulator-side safety authority: an invalid command cannot refresh the lease.
        if not (math.isfinite(message.drive.steering_angle) and
                math.isfinite(message.drive.acceleration)):
            self.get_logger().error("REJECT_NONFINITE_ACTUATOR_COMMAND")
            return
        self.command_stamp_ns=(int(message.header.stamp.sec)*1_000_000_000+
                               int(message.header.stamp.nanosec))
        self.requested_steering = message.drive.steering_angle
        self.requested_acceleration = message.drive.acceleration
        self.last_command_ns = self.get_clock().now().nanoseconds
        self.last_command_steady_ns = time.monotonic_ns()
        self.command_sequence += 1

    def receive_opponent_state(self, message):
        # Current Red pose is a sample, not an interpolation over one Ego tick.
        self.previous_opponent_state=self.opponent_state
        self.opponent_state = State(x=message.px, y=message.py, yaw=message.yaw)

    def reset(self, pose):
        with self.actuation_lock:
            return self._reset_locked(pose)

    def _reset_locked(self, pose):
        self.state = State(x=pose.position.x, y=pose.position.y, yaw=2.0 * math.atan2(pose.orientation.z, pose.orientation.w))
        self.requested_steering = self.config["actuation"]["initial_steering_angle_rad"]
        self.requested_acceleration = self.config["actuation"]["initial_acceleration_m_s2"]
        self.command_stamp_ns=None
        self.command_origin_identity=None
        self.guard_intent_last_sequence=0
        if self.guard_status_lease is not None:
            self.guard_status_lease.latest=None
        if self.external_watchdog_lease is not None:
            self.external_watchdog_lease.revoke()
        self.last_command_ns = None  # resetting pose invalidates command lease
        self.last_command_steady_ns = None

    def reset_from_initial_pose(self, message):
        self.reset(message.pose.pose)

    def reset_from_goal_pose(self, message):
        self.reset(message.pose)

    def physics_step(self):
        with self.actuation_lock:
            return self._physics_step_locked()

    def _physics_step_locked(self):
        dt = self.config["simulation"]["physics_timestep_s"]
        now_ns = self.get_clock().now().nanoseconds
        steady_ns=time.monotonic_ns()
        heartbeat_live=True
        if (self.guard_required and self.command_origin_identity is not None and
                self.command_origin_identity.get('authority_mode')=='CERTIFIED'):
            heartbeat_live,_=self.guard_status_lease.permits_certified(
                now_ros_ns=now_ns,now_steady_ns=steady_ns)
        watchdog_live=True
        watchdog_reason='NOT_REQUIRED'
        if (self.watchdog_required and self.command_origin_identity is not None and
                self.command_origin_identity.get('authority_mode')=='CERTIFIED'):
            watchdog_live,watchdog_reason=self.external_watchdog_lease.permits_certified(
                now_ros_ns=now_ns,now_steady_ns=steady_ns,
                physics_sequence=self.physics_sequence)
        stale=(not watchdog_live or not heartbeat_live or
               self.last_command_ns is None or self.last_command_steady_ns is None or
               now_ns<self.last_command_ns or
               now_ns-self.last_command_ns>self.command_timeout_ns or
               steady_ns-self.last_command_steady_ns>self.command_timeout_ns)
        steering = self.config["actuation"]["initial_steering_angle_rad"] if stale else self.requested_steering
        old_speed = self.state.speed
        # Preserve start pose for full-tick swept-body wall collision.
        previous_pose = (self.state.x, self.state.y, self.state.yaw)
        ego_before=State(x=self.state.x,y=self.state.y,yaw=self.state.yaw)
        red_before=self.previous_opponent_state
        red_after=self.opponent_state
        self.deadman_pub.publish(Bool(data=stale))
        emergency_brake=(not stale and self.guard_required and
                         self.command_origin_identity is not None and
                         self.command_origin_identity.get('authority_mode')=='MINIMUM_RISK_BRAKE')
        if stale:
            acceleration=stale_brake_target(
                old_speed, self.config['dynamics']['limits']['deceleration_max_m_s2'])
        elif emergency_brake:
            acceleration=minimum_risk_brake_at_tick(
                old_speed, self.config['dynamics']['limits']['deceleration_max_m_s2'],dt)
        else:
            acceleration=self.requested_acceleration
        self.dynamics.step(self.state, steering, acceleration, dt)
        if (stale or emergency_brake) and stop_crossed_zero(old_speed, self.state.speed):
            hold_stationary(self.state)
        stamp = self.get_clock().now().to_msg()
        self.physics_sequence += 1
        self.publish_state(stamp)
        self.applied_command_sequence = self.command_sequence
        self.applied_receipt_pub.publish(String(data=json.dumps({
            "physics_sequence": self.physics_sequence,
            "source_command_sequence": self.applied_command_sequence,
            "guard_intent_identity": (self.command_origin_identity
                                      if self.guard_required and not stale else None),
            "command_stamp_ns":self.command_stamp_ns if not stale else None,
            "physics_stamp_ns": stamp.sec * 1000000000 + stamp.nanosec,
            "deadman": stale,
            "independent_watchdog_live": watchdog_live,
            "independent_watchdog_reason": watchdog_reason,
            "applied_steer_rad": steering,
            "applied_accel_mps2": acceleration,
            "emergency_brake_local_reconciliation": bool(emergency_brake),
            "guard_wire_accel_mps2": (self.requested_acceleration
                                        if emergency_brake else None),
            "physics_speed_before_mps": float(old_speed),
        }, sort_keys=True)))
        self.publish_odom(stamp)
        self.publish_transform(stamp)
        self.collision_pub.publish(Bool(data=self.wall_collision(previous_pose)))
        if hasattr(self, "vehicle_collision_pub"):
            footprint=self.config["collision"]["footprint_relative_to_base_link"]
            collision = bool(red_after is not None and (
                footprints_overlap(self.state,red_after,footprint) or
                (red_before is not None and swept_vehicle_disk_overlap(
                    ego_before,self.state,red_before,red_after,footprint,
                    additional_margin_m=0.03))))
            self.vehicle_collision_pub.publish(Bool(data=collision))

    def publish_state(self, stamp):
        s = self.state
        self.state_pub.publish(CarState(px=s.x, py=s.y, yaw=s.yaw, v=s.speed, vx=s.vx, vy=s.vy, omega=s.yaw_rate, a=s.acceleration, ax=s.ax, ay=s.ay, slip_angle=s.slip, accel=s.requested_acceleration, jerk=s.jerk, steer=s.steering, steer_vel=s.steering_rate, physics_stamp=stamp, physics_sequence=self.physics_sequence))

    def publish_odom(self, stamp):
        s = self.state
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.config["map"]["frame_id"]
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = s.x
        message.pose.pose.position.y = s.y
        message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w = quaternion(s.yaw)
        message.twist.twist.linear.x = s.vx
        message.twist.twist.linear.y = s.vy
        message.twist.twist.angular.z = s.yaw_rate
        self.odom_pub.publish(message)

    def publish_transform(self, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.config["map"]["frame_id"]
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.state.x
        transform.transform.translation.y = self.state.y
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = quaternion(self.state.yaw)
        self.tf.sendTransform(transform)

    def publish_scan(self):
        c = self.config["lidar"]
        mount = self.config["vehicle"]["description"]
        origin_x = self.state.x + mount["lidar_x_m"] * math.cos(self.state.yaw) - mount["lidar_y_m"] * math.sin(self.state.yaw)
        origin_y = self.state.y + mount["lidar_x_m"] * math.sin(self.state.yaw) + mount["lidar_y_m"] * math.cos(self.state.yaw)
        angles = self.state.yaw + c["angle_min_rad"] + np.arange(c["beam_count"]) * c["angle_increment_rad"]
        ranges = scan_occupancy(
            self.map,
            origin_x,
            origin_y,
            angles,
            c["range_min_m"],
            c["range_max_m"],
        )
        if self.opponent_state is not None:
            dynamic = ray_box_ranges(origin_x, origin_y, angles, self.opponent_state, self.config["collision"]["footprint_relative_to_base_link"], c["range_max_m"])
            ranges = np.minimum(ranges, dynamic).tolist()
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.laser_frame
        scan.angle_min = c["angle_min_rad"]
        scan.angle_max = c["angle_max_rad"]
        scan.angle_increment = c["angle_increment_rad"]
        scan.time_increment = c["time_increment_s"]
        scan.scan_time = 1.0 / c["scan_rate_hz"]
        scan.range_min = c["range_min_m"]
        scan.range_max = c["range_max_m"]
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def wall_collision(self, previous_pose=None):
        footprint = self.config["collision"]["footprint_relative_to_base_link"]
        if previous_pose is None:
            return footprint_collision(self.map, self.state, footprint)
        # A pose reset has no previous physics interval; physics_step always
        # supplies its pre-integration pose. The sampled swept hull is
        # inflated to cover intersample rigid-body motion conservatively.
        initial = State(x=previous_pose[0], y=previous_pose[1], yaw=previous_pose[2])
        return swept_footprint_collision(self.map, initial, self.state, footprint)

    def publish_map_and_path(self):
        now = self.get_clock().now().to_msg()
        message = OccupancyGrid()
        message.header.stamp = now
        message.header.frame_id = self.config["map"]["frame_id"]
        message.info.resolution = self.map.resolution
        message.info.width = self.map.width
        message.info.height = self.map.height
        message.info.origin.position.x = self.map.origin_x
        message.info.origin.position.y = self.map.origin_y
        yaw = getattr(self.map, "origin_yaw", 0.0)
        (message.info.origin.orientation.x, message.info.origin.orientation.y,
         message.info.origin.orientation.z, message.info.origin.orientation.w) = quaternion(yaw)
        rows = [self.map.pixels[row * self.map.width:(row + 1) * self.map.width] for row in range(self.map.height)]
        if self.map.flip_y:
            rows.reverse()
        message.data = [100 if pixel < self.map.threshold else 0 for row in rows for pixel in row]
        self.map_pub.publish(message)
        path = PathMsg()
        path.header = message.header
        for point in self.centerline:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point["x_m"])
            pose.pose.position.y = float(point["y_m"])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        # Both publishers are transient-local: one sample is retained for late joiners.
        self.map_timer.cancel()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Simulator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
