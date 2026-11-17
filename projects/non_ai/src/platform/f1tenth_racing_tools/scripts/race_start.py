#!/usr/bin/env python3
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


def main():
    rclpy.init(); node = Node("race_start_client"); client = node.create_client(Trigger, "race_start")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("race_start service unavailable"); node.destroy_node(); rclpy.shutdown(); raise SystemExit(1)
    starts = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Time, "race_start_time", starts.append, qos)
    future = client.call_async(Trigger.Request()); rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    response = future.result()
    if response is None or not response.success:
        node.get_logger().error("race_start failed"); code = 1
    else:
        deadline = node.get_clock().now().nanoseconds + 35_000_000_000
        while not starts and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if starts:
            start = starts[-1]
            node.get_logger().info(f"START_TIME={start.sec}.{start.nanosec:09d}"); code = 0
        else:
            node.get_logger().error(f"race readiness timed out: {response.message}"); code = 1
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(code)


if __name__ == "__main__": main()
