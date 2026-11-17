#!/usr/bin/env python3
import json
import os
import time

import rclpy
from control_msgs.msg import ActiveTargetTrajectory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


rclpy.init()
node = Node("late_active_reference_observer")
print(json.dumps({"domain": os.environ.get("ROS_DOMAIN_ID"), "phase": "started"}), flush=True)
received = []
qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
node.create_subscription(ActiveTargetTrajectory, "active_target_trajectory", received.append, qos)
deadline = time.monotonic() + 5.0
while not received and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
print(json.dumps({
    "publisher_count": node.count_publishers("active_target_trajectory"),
    "source_kind": received[0].source_kind if received else None,
}), flush=True)
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if received else 1)
