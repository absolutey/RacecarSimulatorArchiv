#!/usr/bin/env python3
from pathlib import Path
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from control_msgs.msg import ActuatorGuardStatus, ActiveTargetTrajectory, CarState, OpponentPrediction, OpponentState, OvertakingPlan
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def readiness_reason(now_ns, receipts, valid, command_publishers, freshness_ns, histories=None):
    for name in ("STATE", "REFERENCE", "OPPONENT_STATE", "PREDICTION", "PLANNER"):
        if name not in receipts or (name != "REFERENCE" and now_ns - receipts[name] > freshness_ns):
            return f"WAITING_FOR_{name}"
        if name != "STATE" and not valid.get(name, False):
            return f"WAITING_FOR_{name}"
    if command_publishers != 1:
        return "WAITING_FOR_CONTROLLER"
    if histories is not None:
        required = {"STATE": 10, "OPPONENT_STATE": 3, "PREDICTION": 3, "PLANNER": 3}
        for name, count in required.items():
            if len(histories.get(name, ())) < count:
                return f"WAITING_FOR_{name}_CADENCE"
        state_times = histories["STATE"][-10:]
        if (state_times[-1] - state_times[0] <= 0 or
                9e9 / (state_times[-1] - state_times[0]) < 30.0):
            return "WAITING_FOR_STATE_CADENCE"
    return "READY"


def opponent_state_ready(message):
    return bool(message.valid or (
        message.lifecycle == "TENTATIVE" and message.hit_count >= 2 and
        message.confidence >= 0.6 and message.measurement_age_s <= 0.5
    ))


def prediction_ready(message):
    return bool(message.valid or (
        message.source_lifecycle == "TENTATIVE" and message.source_hit_count >= 2 and
        message.source_confidence >= 0.6 and message.source_age_s <= 0.5
    ))


class RaceCoordinator(Node):
    def __init__(self):
        super().__init__("race_coordinator")
        share = Path(get_package_share_directory("f1tenth_racing_bringup"))
        profile = yaml.safe_load((share / "config/foundation.yaml").read_text())["race_profile"]
        self.declare_parameter("require_h2h_ready", False)
        self.declare_parameter("require_actuator_guard", True)
        self.declare_parameter("expected_guard_source", "R_B")
        self.declare_parameter("expected_guard_epoch", "")
        self.guard_status = None
        self.guard_steady_ns = None
        self.declare_parameter("readiness_timeout_s", 30.0)
        self.declare_parameter("readiness_freshness_s", 0.75)
        self.require_h2h_ready = self.get_parameter("require_h2h_ready").value
        self.delay_ns = round(profile["synchronized_start_delay_s"] * 1_000_000_000)
        self.timeout_ns = round(self.get_parameter("readiness_timeout_s").value * 1_000_000_000)
        self.freshness_ns = round(self.get_parameter("readiness_freshness_s").value * 1_000_000_000)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Time, "race_start_time", qos)
        self.status_publisher = self.create_publisher(String, "race_readiness", qos)
        self.service = self.create_service(Trigger, "race_start", self.start)
        self.create_subscription(ActuatorGuardStatus, "actuator_guard_status0",
                                 self.receive_guard, qos)
        self.start_message = None
        self.pending_since_ns = None
        self.receipts, self.valid, self.histories = {}, {}, {}
        if self.require_h2h_ready:
            self.create_subscription(CarState, "state0", lambda message: self.received("STATE", True), 1)
            self.create_subscription(ActiveTargetTrajectory, "active_target_trajectory",
                                     lambda message: self.received("REFERENCE", message.active_reference_valid and message.source_kind == "NOMINAL"), qos)
            self.create_subscription(OpponentState, "opponent_state",
                                     lambda message: self.received("OPPONENT_STATE", opponent_state_ready(message)), 10)
            self.create_subscription(OpponentPrediction, "opponent_prediction",
                                     lambda message: self.received("PREDICTION", prediction_ready(message)), 10)
            self.create_subscription(OvertakingPlan, "overtaking_plan",
                                     lambda message: self.received("PLANNER", message.schema_version == 2), 10)
            self.create_timer(0.1, self.advance)

    def receive_guard(self, message):
        self.guard_status=message
        self.guard_steady_ns=time.monotonic_ns()

    def guard_reason(self):
        if not self.get_parameter("require_actuator_guard").value:
            return "GUARD_DISABLED_UNQUALIFIED"
        status=self.guard_status
        if status is None or self.guard_steady_ns is None or time.monotonic_ns()-self.guard_steady_ns>500000000:
            return "WAITING_FOR_GUARD_STATUS"
        if status.run_epoch!=self.get_parameter("expected_guard_epoch").value:
            return "GUARD_EPOCH_MISMATCH"
        if status.source_id!=self.get_parameter("expected_guard_source").value:
            return "GUARD_SOURCE_MISMATCH"
        if not status.ready:
            return "WAITING_FOR_GUARD_READY:"+status.reason
        infos=self.get_publishers_info_by_topic("ackermann_cmd0")
        if len(infos)!=1 or infos[0].node_name!="actuator_guard0":
            return "FINAL_PUBLISHER_NOT_EXCLUSIVE_GUARD"
        return "READY"

    def received(self, name, valid):
        now_ns = self.get_clock().now().nanoseconds
        self.receipts[name] = now_ns
        self.valid[name] = bool(valid)
        self.histories.setdefault(name, []).append(now_ns)
        self.histories[name] = self.histories[name][-20:]

    def status(self):
        guard=self.guard_reason()
        if guard!="READY":return guard
        return readiness_reason(
            self.get_clock().now().nanoseconds, self.receipts, self.valid,
            self.count_publishers("ackermann_cmd0"), self.freshness_ns,
            self.histories,
        )

    def publish_start(self):
        start_ns = self.get_clock().now().nanoseconds + self.delay_ns
        self.start_message = Time(sec=start_ns // 1_000_000_000, nanosec=start_ns % 1_000_000_000)
        self.publisher.publish(self.start_message)
        self.status_publisher.publish(String(data="READY"))

    def advance(self):
        if self.pending_since_ns is None or self.start_message is not None:
            return
        status = self.status()
        self.status_publisher.publish(String(data=status))
        if status == "READY":
            self.publish_start()
        elif self.get_clock().now().nanoseconds - self.pending_since_ns >= self.timeout_ns:
            self.get_logger().error(f"H2H readiness timeout: {status}; race remains stationary")
            self.status_publisher.publish(String(data=f"TIMEOUT:{status}"))
            self.pending_since_ns = None

    def start(self, request, response):
        del request
        if self.start_message is None:
            guard=self.guard_reason()
            if guard!="READY":
                response.success=False
                response.message=guard
                return response
            if self.require_h2h_ready:
                if self.pending_since_ns is None:
                    self.pending_since_ns = self.get_clock().now().nanoseconds
                status = self.status()
                self.status_publisher.publish(String(data=status))
                response.success = True
                response.message = f"PENDING:{status}"
                return response
            self.publish_start()
        response.success = True
        response.message = f"{self.start_message.sec}.{self.start_message.nanosec:09d}"
        return response


def main():
    rclpy.init(); node = RaceCoordinator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__": main()
