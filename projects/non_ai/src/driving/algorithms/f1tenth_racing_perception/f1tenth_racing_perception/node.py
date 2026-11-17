import math
import csv
from collections import deque
import gc
import os
from pathlib import Path
import resource
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import OpponentState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from f1tenth_racing_sim.world.map import OccupancyMap

from .core import CvKalmanTracker, Pose2D, PoseBuffer, TrackCorridor, UkfExtentTracker, extract_measurements
from .expected_scan import StaticExpectedScan


SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class OpponentPerceptionNode(Node):
    def __init__(self):
        super().__init__("opponent_perception")
        perception_share = Path(get_package_share_directory("f1tenth_racing_perception"))
        sim_share = Path(get_package_share_directory("f1tenth_racing_sim"))
        self.declare_parameter("selected_tracker", "")
        self.declare_parameter("environment_file",str(sim_share / "config/competition_environment.yaml"))
        self.declare_parameter("expected_track_id", "")
        self.declare_parameter("expected_map_sha256", "")
        self.c = yaml.safe_load((perception_share / "config/perception.yaml").read_text())
        from hashlib import sha256
        environment_file = Path(self.get_parameter("environment_file").value).resolve(strict=True)
        environment = yaml.safe_load(environment_file.read_text())
        expected_track = self.get_parameter("expected_track_id").value
        expected_map_sha = self.get_parameter("expected_map_sha256").value
        if not isinstance(environment, dict) or (expected_track and
                environment['environment_identity']['active_track'] != expected_track):
            raise ValueError('PERCEPTION_ENVIRONMENT_TRACK_MISMATCH')
        if expected_map_sha and environment['map']['image_sha256'] != expected_map_sha:
            raise ValueError('PERCEPTION_ENVIRONMENT_MAP_MISMATCH')
        selected_map=(sim_share / environment['map']['image_file']).resolve(strict=True)
        selected_centerline=(sim_share / environment['map']['centerline_file']).resolve(strict=True)
        if sha256(selected_map.read_bytes()).hexdigest()!=environment['map']['image_sha256'] or \
                sha256(selected_centerline.read_bytes()).hexdigest()!=environment['map']['centerline_sha256']:
            raise ValueError('PERCEPTION_ASSET_SHA_MISMATCH')
        self.environment = environment
        selected = self.get_parameter("selected_tracker").value or self.c["selected_tracker"]
        if selected not in ("cv", "ukf"):
            raise ValueError("selected_tracker must be cv or ukf")
        self.selected = selected
        self.map = OccupancyMap(selected_map, environment["map"])
        lidar = environment["lidar"]
        self.expected_scan = StaticExpectedScan(
            self.map, lidar["angle_min_rad"], lidar["angle_increment_rad"],
            lidar["beam_count"], lidar["range_min_m"], lidar["range_max_m"],
        )
        mount = environment["vehicle"]["description"]
        self.lidar_x_m, self.lidar_y_m = mount["lidar_x_m"], mount["lidar_y_m"]
        with selected_centerline.open(newline="") as stream:
            self.corridor = TrackCorridor(
                list(csv.DictReader(stream)), 0.5 * mount["chassis_width_m"],
            )
        self.vehicle_radius_m = 0.5 * math.hypot(
            mount["chassis_length_m"], mount["chassis_width_m"],
        )
        self.tracker = (CvKalmanTracker if selected == "cv" else UkfExtentTracker)(
            self.c, self.corridor.geometry,
        )
        self.pose_buffer = PoseBuffer()
        self.pending_scan = None
        self.pending_scan_meta = None
        self.scan_sequence = self.overwritten_scans = 0
        self.previous_process_start_ns = self.previous_process_finish_ns = None
        self.timing_path = os.environ.get("F1TENTH_R_C_TIMING_PATH")
        # Diagnostic runs are explicitly bounded; retain up to 500 s at 10 Hz.
        self.timing_records = deque(maxlen=5000)
        self.publisher = self.create_publisher(OpponentState, "opponent_state", 10)
        self.create_subscription(Odometry, "odom0", self.receive_odom, SENSOR_QOS)
        self.create_subscription(LaserScan, "scan0", self.receive_scan, SENSOR_QOS)

    def receive_scan(self, scan):
        callback_wall_ns = time.perf_counter_ns()
        callback_cpu_ns = time.process_time_ns()
        callback_ros_ns = self.get_clock().now().nanoseconds
        if scan.header.frame_id != self.environment["lidar"]["frame_car0"]:
            self.publish_invalid(scan.header.stamp, "WRONG_SCAN_FRAME")
            return
        self.scan_sequence += 1
        self.overwritten_scans += int(self.pending_scan is not None)
        self.pending_scan = scan
        self.pending_scan_meta = (
            self.scan_sequence, callback_wall_ns, callback_cpu_ns,
            callback_ros_ns, self.overwritten_scans,
        )
        self.try_process()

    def receive_odom(self, odometry):
        if odometry.header.frame_id != self.environment["map"]["frame_id"] or odometry.child_frame_id != "base_link0":
            return
        pose = odometry.pose.pose
        self.pose_buffer.add(Pose2D(
            stamp_s(odometry.header.stamp), pose.position.x, pose.position.y,
            yaw_from_quaternion(pose.orientation),
        ))
        self.try_process()

    def try_process(self):
        if self.pending_scan is None:
            return
        scan_time = stamp_s(self.pending_scan.header.stamp)
        poses = self.pose_buffer.candidates_at_or_before(
            scan_time, self.c["maximum_pose_alignment_s"],
        )
        if not poses:
            return
        scan, self.pending_scan = self.pending_scan, None
        sequence, receive_wall_ns, _, receive_ros_ns, overwritten = self.pending_scan_meta
        self.pending_scan_meta = None
        started_wall_ns, started_cpu_ns = time.perf_counter_ns(), time.process_time_ns()
        candidates = len(poses)
        pose = max(poses, key=lambda candidate: self.static_endpoint_score(scan, candidate))
        pose_wall_ns, pose_cpu_ns = time.perf_counter_ns(), time.process_time_ns()
        sensor_x = pose.x_m + self.lidar_x_m * math.cos(pose.yaw_rad) - self.lidar_y_m * math.sin(pose.yaw_rad)
        sensor_y = pose.y_m + self.lidar_x_m * math.sin(pose.yaw_rad) + self.lidar_y_m * math.cos(pose.yaw_rad)
        transform_wall_ns, transform_cpu_ns = time.perf_counter_ns(), time.process_time_ns()
        expected = self.expected_scan.scan(sensor_x, sensor_y, pose.yaw_rad)
        expected_wall_ns, expected_cpu_ns = time.perf_counter_ns(), time.process_time_ns()
        stage_marks = {} if self.timing_path else None
        measurements, residual_count = extract_measurements(
            scan.ranges, expected, scan.angle_min, scan.angle_increment, pose,
            self.lidar_x_m, self.lidar_y_m, self.c, self.map, self.corridor,
            self.vehicle_radius_m, stage_marks=stage_marks,
        )
        extraction_wall_ns, extraction_cpu_ns = time.perf_counter_ns(), time.process_time_ns()
        estimate = self.tracker.update(measurements, scan_time)
        tracker_wall_ns, tracker_cpu_ns = time.perf_counter_ns(), time.process_time_ns()
        elapsed_us = (tracker_wall_ns - started_wall_ns) / 1e3
        message = OpponentState()
        message.header = scan.header
        message.header.frame_id = "map"
        message.schema_version = 1
        message.track_id = self.environment["environment_identity"]["active_track"]
        message.opponent_id = "opponent0"
        message.valid = estimate.valid
        message.tracker = self.selected.upper()
        message.lifecycle = estimate.lifecycle
        message.status_reason = estimate.reason
        message.track_revision = estimate.revision
        message.x_m, message.y_m, message.vx_mps, message.vy_mps = map(float, estimate.state)
        message.speed_mps = math.hypot(message.vx_mps, message.vy_mps)
        message.heading_rad = math.atan2(message.vy_mps, message.vx_mps) if message.speed_mps > 1e-3 else 0.0
        message.covariance = estimate.covariance.reshape(-1).tolist()
        message.track_age_s = estimate.age_s
        message.measurement_age_s = estimate.measurement_age_s
        message.hit_count = estimate.hits
        message.miss_count = estimate.misses
        message.confidence = estimate.confidence
        message.measured_extent_m = estimate.extent_m
        message.sensor_x_m = sensor_x
        message.sensor_y_m = sensor_y
        message.compute_time_us = elapsed_us
        message.residual_point_count = residual_count
        message.candidate_count = len(measurements)
        construction_wall_ns, construction_cpu_ns = time.perf_counter_ns(), time.process_time_ns()
        self.publisher.publish(message)
        finish_wall_ns, finish_cpu_ns = time.perf_counter_ns(), time.process_time_ns()
        if self.timing_path:
            marks = stage_marks
            residual_wall_ns, residual_cpu_ns = marks["residual"]
            dynamic_wall_ns, dynamic_cpu_ns = marks["dynamic"]
            clustering_wall_ns, clustering_cpu_ns = marks["clustering"]
            candidate_wall_ns, candidate_cpu_ns = marks["candidates"]
            finite_beams = int(np.count_nonzero(np.isfinite(np.asarray(scan.ranges))))
            gc0, gc1, gc2 = gc.get_count()
            self.timing_records.append((
                sequence, scan_time, receive_ros_ns * 1e-9 - scan_time,
                (started_wall_ns - receive_wall_ns) / 1e6,
                0.0 if self.previous_process_start_ns is None else
                (started_wall_ns - self.previous_process_start_ns) / 1e6,
                overwritten, candidates, finite_beams, residual_count, len(measurements),
                (pose_wall_ns - started_wall_ns) / 1e6,
                (transform_wall_ns - pose_wall_ns) / 1e6,
                (expected_wall_ns - transform_wall_ns) / 1e6,
                (residual_wall_ns - expected_wall_ns) / 1e6,
                (dynamic_wall_ns - residual_wall_ns) / 1e6,
                (clustering_wall_ns - dynamic_wall_ns) / 1e6,
                (candidate_wall_ns - clustering_wall_ns) / 1e6,
                (tracker_wall_ns - extraction_wall_ns) / 1e6,
                (construction_wall_ns - tracker_wall_ns) / 1e6,
                (finish_wall_ns - construction_wall_ns) / 1e6,
                (finish_wall_ns - started_wall_ns) / 1e6,
                (finish_cpu_ns - started_cpu_ns) / 1e6,
                resource.getrusage(resource.RUSAGE_SELF).ru_maxrss, gc0, gc1, gc2,
            ))
        self.previous_process_start_ns = started_wall_ns
        self.previous_process_finish_ns = finish_wall_ns

    def write_timing(self):
        if not self.timing_path:
            return
        path = Path(self.timing_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        header = (
            "sequence", "scan_time_s", "input_age_s", "pending_wait_ms",
            "inter_callback_ms", "overwritten_scans", "pose_candidates", "finite_beams",
            "residual_beams", "measurements", "pose_score_ms", "transform_ms",
            "expected_scan_ms", "residual_ms", "dynamic_filter_ms", "clustering_ms",
            "candidate_filter_ms", "tracking_ms", "message_ms", "publish_ms",
            "total_wall_ms", "total_cpu_ms", "max_rss_kib", "gc_gen0", "gc_gen1",
            "gc_gen2",
        )
        with path.open("w", newline="") as stream:
            writer = csv.writer(stream, delimiter="\t")
            writer.writerow(header)
            writer.writerows(self.timing_records)

    def static_endpoint_score(self, scan, pose):
        sensor_x = pose.x_m + self.lidar_x_m * math.cos(pose.yaw_rad) - self.lidar_y_m * math.sin(pose.yaw_rad)
        sensor_y = pose.y_m + self.lidar_x_m * math.sin(pose.yaw_rad) + self.lidar_y_m * math.cos(pose.yaw_rad)
        ranges = np.asarray(scan.ranges)
        indices = np.flatnonzero(np.isfinite(ranges) & (ranges < scan.range_max - 1e-6))
        angles = pose.yaw_rad + scan.angle_min + indices * scan.angle_increment
        xs = sensor_x + ranges[indices] * np.cos(angles)
        ys = sensor_y + ranges[indices] * np.sin(angles)
        return sum(self.map.occupied(float(x), float(y)) for x, y in zip(xs, ys))

    def publish_invalid(self, stamp, reason):
        message = OpponentState()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.schema_version = 1
        message.track_id = self.environment["environment_identity"]["active_track"]
        message.opponent_id = "opponent0"
        message.tracker = self.selected.upper()
        message.lifecycle = "LOST"
        message.status_reason = reason
        message.covariance = (np.eye(4) * 1e6).reshape(-1).tolist()
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = OpponentPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_timing()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
