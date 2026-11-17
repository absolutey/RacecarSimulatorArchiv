import csv
import hashlib
import math
from pathlib import Path

import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from control_msgs.msg import CarState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class RedBaselineController(Node):
    def __init__(self):
        super().__init__("red_baseline_controller")
        sim = Path(get_package_share_directory("f1tenth_racing_sim"))
        self.declare_parameter("environment_file",str(sim/"config/competition_environment.yaml"))
        self.declare_parameter("centerline_file","")
        environment=yaml.safe_load(Path(self.get_parameter("environment_file").value).read_text())
        centerline=Path(self.get_parameter("centerline_file").value or
                        str(sim/environment["map"]["centerline_file"])).resolve(strict=True)
        if hashlib.sha256(centerline.read_bytes()).hexdigest()!=environment["map"]["centerline_sha256"]:
            raise ValueError("red selected centerline mismatch")
        self.declare_parameter("target_speed_m_s", rclpy.Parameter.Type.DOUBLE); self.target_speed = self.get_parameter("target_speed_m_s").value; self.wheelbase = environment["dynamics"]["dynamic_wheelbase_m"]
        with centerline.open(newline="") as stream: self.points = [(float(row["x_m"]), float(row["y_m"])) for row in csv.DictReader(stream)]
        self.state = None; self.start_ns = None
        self.pub = self.create_publisher(AckermannDriveStamped, "ackermann_cmd1", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(CarState, "state1", self.receive_state, 1)
        start_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Time, "race_start_time", self.receive_start, start_qos)
        self.create_timer(0.05, self.control)

    def receive_state(self, message): self.state = message
    def receive_start(self, message): self.start_ns = message.sec * 1_000_000_000 + message.nanosec

    def control(self):
        command = AckermannDriveStamped()
        if self.state is not None and self.start_ns is not None and self.get_clock().now().nanoseconds >= self.start_ns:
            nearest = min(range(len(self.points)), key=lambda i: (self.points[i][0] - self.state.px) ** 2 + (self.points[i][1] - self.state.py) ** 2)
            target = self.points[(nearest + 2) % len(self.points)]
            alpha = wrap(math.atan2(target[1] - self.state.py, target[0] - self.state.px) - self.state.yaw); lookahead = max(0.5, math.hypot(target[0] - self.state.px, target[1] - self.state.py))
            command.drive.steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), lookahead); command.drive.acceleration = 2.0 * (self.target_speed - self.state.v)
        self.pub.publish(command)


def main():
    rclpy.init(); node = RedBaselineController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
