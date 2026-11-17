import csv
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from control_msgs.msg import CarState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, Int32


class Evaluator(Node):
    def __init__(self):
        super().__init__("race_evaluator")
        self.valid = True
        self.start_ns = None
        self.laps = [0, 0]
        self.last_index = [None, None]
        self.speeds = [0.0, 0.0]
        self.wall_counts = [0, 0]
        self.wall_levels = [False, False]
        self.last_lap_ns = [None, None]
        sim = Path(get_package_share_directory("f1tenth_racing_sim"))
        self.declare_parameter("centerline_file", str(sim / "maps/iccas2025/iccas2025_centerline.csv"))
        with Path(self.get_parameter("centerline_file").value).open(newline="") as stream:
            self.points = [(float(row["x_m"]), float(row["y_m"])) for row in csv.DictReader(stream)]
        self.valid_pub = self.create_publisher(Bool, "race_valid", 1)
        self.elapsed_pub = self.create_publisher(Float64, "race_elapsed_time", 1)
        self.lap_pubs = [self.create_publisher(Int32, f"lap_count{i}", 1) for i in range(2)]
        self.speed_pubs = [self.create_publisher(Float64, f"speed{i}", 1) for i in range(2)]
        self.wall_pubs = [self.create_publisher(Int32, f"wall_collision_count{i}", 1) for i in range(2)]
        self.lap_time_pubs = [self.create_publisher(Float64, f"lap_time{i}", 10) for i in range(2)]
        self.create_subscription(Bool, "vehicle_collision", self.collision, 1)
        self.create_subscription(CarState, "state0", lambda message: self.state(0, message), 1)
        self.create_subscription(CarState, "state1", lambda message: self.state(1, message), 1)
        self.create_subscription(Bool, "collision0", lambda message: self.wall(0, message), 1)
        self.create_subscription(Bool, "collision1", lambda message: self.wall(1, message), 1)
        start_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Time, "race_start_time", self.start, start_qos)
        self.create_timer(0.1, self.publish)

    def collision(self, message):
        if message.data:
            self.valid = False

    def start(self, message):
        self.start_ns = message.sec * 1_000_000_000 + message.nanosec
        self.last_lap_ns = [self.start_ns, self.start_ns]

    def state(self, car_id, message):
        self.speeds[car_id] = message.v
        if self.start_ns is None or self.get_clock().now().nanoseconds < self.start_ns:
            return
        index = min(range(len(self.points)), key=lambda i: (self.points[i][0] - message.px) ** 2 + (self.points[i][1] - message.py) ** 2)
        previous = self.last_index[car_id]
        if previous is not None and previous > len(self.points) * 0.75 and index < len(self.points) * 0.25:
            now_ns = self.get_clock().now().nanoseconds

            # Blue starts one metre before the start/finish line.
            # Its first forward seam crossing STARTS lap 1; it does not
            # represent a completed lap and therefore publishes no lap time.
            if car_id == 0 and self.laps[car_id] == 0:
                self.laps[car_id] = 1
                self.last_lap_ns[car_id] = now_ns
            else:
                self.laps[car_id] += 1
                self.lap_time_pubs[car_id].publish(
                    Float64(data=(now_ns - self.last_lap_ns[car_id]) / 1_000_000_000)
                )
                self.last_lap_ns[car_id] = now_ns
        self.last_index[car_id] = index

    def wall(self, car_id, message):
        if message.data:
            self.valid = False
        if message.data and not self.wall_levels[car_id]:
            self.wall_counts[car_id] += 1
        self.wall_levels[car_id] = message.data

    def publish(self):
        self.valid_pub.publish(Bool(data=self.valid))
        elapsed = 0.0 if self.start_ns is None else max(0.0, (self.get_clock().now().nanoseconds - self.start_ns) / 1_000_000_000)
        self.elapsed_pub.publish(Float64(data=elapsed))
        for car_id in range(2):
            self.lap_pubs[car_id].publish(Int32(data=self.laps[car_id]))
            self.speed_pubs[car_id].publish(Float64(data=self.speeds[car_id]))
            self.wall_pubs[car_id].publish(Int32(data=self.wall_counts[car_id]))


def main():
    rclpy.init()
    node = Evaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
