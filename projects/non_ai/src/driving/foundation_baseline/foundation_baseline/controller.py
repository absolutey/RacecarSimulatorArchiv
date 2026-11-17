import csv
import hashlib
import math
from pathlib import Path

import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from control_msgs.msg import ActuatorIntent, CarState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class BlueFoundationController(Node):
    def __init__(self):
        super().__init__("blue_foundation_controller")
        sim = Path(get_package_share_directory("f1tenth_racing_sim"))
        self.declare_parameter("guard_run_epoch", "")
        self.declare_parameter("environment_file", str(sim / "config/competition_environment.yaml"))
        self.declare_parameter("centerline_file", "")
        selected_env = Path(self.get_parameter("environment_file").value).resolve(strict=True)
        environment = yaml.safe_load(selected_env.read_text())
        self.track_id = environment["environment_identity"]["active_track"]
        self.map_sha256 = environment["map"]["image_sha256"]
        self.intent_sequence = 0
        self.last_state_stamp = Time()
        self.last_state_sequence = 0
        centerline = Path(self.get_parameter("centerline_file").value or
                          str(sim / environment["map"]["centerline_file"])).resolve(strict=True)
        if hashlib.sha256(centerline.read_bytes()).hexdigest() != environment["map"]["centerline_sha256"]:
            raise ValueError("baseline centerline SHA mismatch")
        self.declare_parameter("target_speed_m_s", rclpy.Parameter.Type.DOUBLE)
        self.target_speed = self.get_parameter("target_speed_m_s").value
        self.wheelbase = environment["dynamics"]["dynamic_wheelbase_m"]
        with centerline.open(newline="") as stream:
            self.points = [(float(row["x_m"]), float(row["y_m"])) for row in csv.DictReader(stream)]
        self.state = None
        self.start_ns = None
        self.pub = self.create_publisher(ActuatorIntent, "actuator_intent0", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(CarState, "state0", self.receive_state, 1)
        start_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Time, "race_start_time", self.receive_start, start_qos)
        self.create_timer(0.05, self.control)

    def receive_state(self, message):
        self.state = message
        self.last_state_stamp = message.physics_stamp
        self.last_state_sequence = message.physics_sequence

    def receive_start(self, message):
        self.start_ns = message.sec * 1_000_000_000 + message.nanosec

    def control(self):
        command = AckermannDriveStamped()
        if self.state is not None and self.start_ns is not None and self.get_clock().now().nanoseconds >= self.start_ns:
            nearest = min(range(len(self.points)), key=lambda i: (self.points[i][0] - self.state.px) ** 2 + (self.points[i][1] - self.state.py) ** 2)
            target = self.points[(nearest + 2) % len(self.points)]
            alpha = wrap(math.atan2(target[1] - self.state.py, target[0] - self.state.px) - self.state.yaw)
            lookahead = max(0.5, math.hypot(target[0] - self.state.px, target[1] - self.state.py))
            command.drive.steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), lookahead)
            command.drive.acceleration = 2.0 * (self.target_speed - self.state.v)
        if self.state is None or self.start_ns is None or self.get_clock().now().nanoseconds < self.start_ns:
            return
        now_ns = self.get_clock().now().nanoseconds
        self.intent_sequence += 1
        intent = ActuatorIntent()
        intent.source_id = "FOUNDATION_BASELINE"
        intent.run_epoch = self.get_parameter("guard_run_epoch").value
        intent.sequence = self.intent_sequence
        intent.created_ros_stamp = self.get_clock().now().to_msg()
        expires = now_ns + 75000000
        intent.expires_ros_stamp = Time(sec=expires//1000000000, nanosec=expires%1000000000)
        intent.source_state_stamp = self.last_state_stamp
        intent.source_state_sequence = self.last_state_sequence
        intent.active_reference_id = "FOUNDATION_BASELINE"
        intent.active_revision = 0
        intent.track_id = self.track_id
        intent.map_sha256 = self.map_sha256
        intent.candidate_mode = "FOUNDATION_BASELINE"
        intent.steering_rad = float(command.drive.steering_angle)
        intent.acceleration_mps2 = float(command.drive.acceleration)
        self.pub.publish(intent)


def main():
    rclpy.init(); node = BlueFoundationController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
