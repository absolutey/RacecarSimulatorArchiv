import csv
import math
from pathlib import Path
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import OpponentPrediction, OpponentPredictionSample, OpponentState
from f1tenth_racing_geometry import ClosedTrackGeometry
from rclpy.node import Node

from .core import (
    AnalyticImmPredictor,
    OpponentMotion,
    TrackCvPredictor,
    directional_variances,
    prediction_source_is_eligible,
)


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class OpponentPredictionNode(Node):
    def __init__(self):
        super().__init__("opponent_prediction")
        share = Path(get_package_share_directory("f1tenth_racing_prediction"))
        sim_share = Path(get_package_share_directory("f1tenth_racing_sim"))
        self.declare_parameter("selected_predictor", "")
        self.declare_parameter("environment_file",str(sim_share / "config/competition_environment.yaml"))
        self.declare_parameter("centerline_file", "")
        self.declare_parameter("expected_track_id", "")
        self.declare_parameter("expected_map_sha256", "")
        self.c = yaml.safe_load((share / "config/prediction.yaml").read_text())
        self.declare_parameter("horizon_s", float(self.c["horizon_s"]))
        self.declare_parameter("timestep_s", float(self.c["timestep_s"]))
        from hashlib import sha256
        environment=yaml.safe_load(Path(self.get_parameter('environment_file').value).read_text())
        if (self.get_parameter('expected_track_id').value and
                environment['environment_identity']['active_track'] != self.get_parameter('expected_track_id').value):
            raise ValueError('PREDICTION_TRACK_MISMATCH')
        if (self.get_parameter('expected_map_sha256').value and
                environment['map']['image_sha256'] != self.get_parameter('expected_map_sha256').value):
            raise ValueError('PREDICTION_MAP_MISMATCH')
        centerline=Path(self.get_parameter('centerline_file').value or
                        str(sim_share/environment['map']['centerline_file'])).resolve(strict=True)
        if sha256(centerline.read_bytes()).hexdigest()!=environment['map']['centerline_sha256']:
            raise ValueError('PREDICTION_CENTERLINE_SHA_MISMATCH')
        with centerline.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.geometry = ClosedTrackGeometry((float(row["x_m"]), float(row["y_m"])) for row in rows)
        selected = self.get_parameter("selected_predictor").value or self.c["selected_predictor"]
        self.horizon_s = float(self.get_parameter("horizon_s").value)
        self.timestep_s = float(self.get_parameter("timestep_s").value)
        if not np.isfinite(self.horizon_s) or self.horizon_s <= 0.0:
            raise ValueError("horizon_s must be finite and positive")
        if not np.isfinite(self.timestep_s) or self.timestep_s <= 0.0:
            raise ValueError("timestep_s must be finite and positive")
        if self.timestep_s > self.horizon_s + 1e-12:
            raise ValueError("timestep_s must not exceed horizon_s")
        if selected == "track_cv":
            self.predictor = TrackCvPredictor(self.geometry, self.c["process_acceleration_variance_m2_s4"])
        elif selected == "imm":
            self.predictor = AnalyticImmPredictor(
                self.geometry, self.c["imm_transition"], self.c["process_acceleration_variance_m2_s4"],
            )
        else:
            raise ValueError("selected_predictor must be track_cv or imm")
        self.selected = selected
        self.track_id = environment["environment_identity"]["active_track"]
        footprint = environment["collision"]["footprint_relative_to_base_link"]
        self.half_length = 0.5 * (footprint["max_x_m"] - footprint["min_x_m"])
        self.half_width = 0.5 * (footprint["max_y_m"] - footprint["min_y_m"])
        self.publisher = self.create_publisher(OpponentPrediction, "opponent_prediction", 1)
        self.create_subscription(OpponentState, "opponent_state", self.receive, 10)

    def receive(self, state):
        message = OpponentPrediction()
        message.header = state.header
        message.schema_version = 1
        message.source_observation_time = state.header.stamp
        message.source_opponent_id = state.opponent_id
        message.source_track_revision = state.track_revision
        message.source_lifecycle = state.lifecycle
        message.source_confidence = state.confidence
        message.source_hit_count = state.hit_count
        message.source_miss_count = state.miss_count
        message.predictor = self.selected.upper()
        message.track_id = state.track_id
        message.frame_id = state.header.frame_id
        message.horizon_s = self.horizon_s
        message.timestep_s = self.timestep_s
        message.source_age_s = state.measurement_age_s
        source_eligible = prediction_source_is_eligible(
            state.valid,
            state.lifecycle,
            state.hit_count,
            state.confidence,
            state.measurement_age_s,
            maximum_age_s=self.c["maximum_source_measurement_age_s"],
        )
        if (not source_eligible or state.track_id != self.track_id or
                state.header.frame_id != "map"):
            message.status_reason = (
                "INVALID_OR_STALE_SOURCE"
                if source_eligible or state.lifecycle != "TENTATIVE"
                else "TENTATIVE_SOURCE_BELOW_PREDICTION_GATE"
            )
            self.publisher.publish(message)
            return
        started = time.perf_counter()
        motion = OpponentMotion(
            stamp_s(state.header.stamp), state.x_m, state.y_m, state.vx_mps, state.vy_mps,
            np.asarray(state.covariance).reshape(4, 4),
        )
        times = np.arange(
            self.timestep_s,
            self.horizon_s + 0.5 * self.timestep_s,
            self.timestep_s,
        )
        try:
            samples = self.predictor.predict(motion, times)
        except (ValueError, np.linalg.LinAlgError):
            message.status_reason = "INVALID_SOURCE_NUMERICS"
            self.publisher.publish(message)
            return
        sigma = self.c["occupancy_sigma"]
        for sample in samples:
            output = OpponentPredictionSample()
            output.dt_s = sample.dt_s
            output.x_m, output.y_m = sample.x_m, sample.y_m
            output.vx_mps, output.vy_mps = sample.vx_mps, sample.vy_mps
            output.heading_rad, output.s_m, output.d_m = sample.heading_rad, sample.s_m, sample.d_m
            output.covariance_xy = sample.covariance_xy.reshape(-1).tolist()
            along_variance, across_variance = directional_variances(
                sample.covariance_xy, sample.heading_rad,
            )
            output.occupancy_half_length_m = self.half_length + sigma * math.sqrt(max(0.0, along_variance))
            output.occupancy_half_width_m = self.half_width + sigma * math.sqrt(max(0.0, across_variance))
            message.samples.append(output)
        message.valid = True
        message.status_reason = (
            "PREDICTION_VALID_TENTATIVE_SOURCE"
            if not state.valid
            else "PREDICTION_VALID"
        )
        message.compute_time_us = (time.perf_counter() - started) * 1e6
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = OpponentPredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
