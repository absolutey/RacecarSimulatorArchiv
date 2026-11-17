import re
import json
from pathlib import Path
import math

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import (ActiveTargetTrajectory, CarState, ControllerStatus,
                              OpponentPrediction, OpponentState)
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, Int32
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_racing_geometry import ClosedTrackGeometry, interpolate_periodic_scalar
from f1tenth_racing_raceline import load_racing_trajectory
from f1tenth_racing_telemetry.review_validity import LapCollisionReview


PHASES = ("FOUNDATION", "R-A", "R-B", "R-C", "R-D")


def action_for(state, valid):
    if not valid:
        return "INVALID"
    return {"PREPARE": "OVERTAKING", "COMMITTED_PASS": "OVERTAKING",
            "PASSING": "OVERTAKING", "MERGE": "MERGING", "ABORT": "MERGING",
            "BLOCKING_HOLD": "RACING"}.get(state, "RACING")


def format_duration(seconds):
    milliseconds = max(0, round(seconds * 1000))
    minutes, milliseconds = divmod(milliseconds, 60_000)
    whole_seconds, milliseconds = divmod(milliseconds, 1000)
    return f"{minutes:02d}:{whole_seconds:02d}.{milliseconds:03d}"


def board_text(lap, current_lap, best_lap, blue_speed, red_speed, position, gap, passes,
               action, valid, total=0.0):
    return ("F1TENTH H2H RACE\n"
            f"LAP {lap}   CURRENT {current_lap:5.2f}s   BEST {best_lap:5.2f}s "
            f"TOTAL {format_duration(total)}\n"
            f"BLUE {blue_speed:4.1f} m/s   RED {red_speed:4.1f} m/s\n"
            f"POSITION P{position}   GAP {gap:4.1f} m   OVERTAKES {passes}\n"
            f"{action}   RACE {'VALID' if valid else 'INVALID'}")


def closed_nominal_points(trajectory):
    """Close only a verified full-lap offline nominal; never close an override."""
    if trajectory.metadata.get("closed_loop") is not True or len(trajectory.samples) < 3:
        raise ValueError("HUD nominal trajectory is not a valid closed loop")
    points = [(sample.x_m, sample.y_m, 0.085) for sample in trajectory.samples]
    return points + [points[0]]


def nominal_identity_status(active, expected_trajectory_id):
    """Read-only display check, never changes S0-R authority or controller."""
    if active is None:
        return "NO ACTIVE"
    if active.source_kind != "NOMINAL":
        return "OVERRIDE / LOCAL"
    if not active.active_reference_valid:
        return "INVALID ACTIVE"
    return ("MATCH" if active.source_trajectory_id == expected_trajectory_id
            else "MISMATCH")


def active_reference_points(message, z=0.025):
    """Display ONLY; preserve sample order and number, never close local plans."""
    return [(sample.x_m, sample.y_m, z) for sample in message.samples]


def active_reference_color(source_kind):
    """Full nominal owns blue; local nominal is dim gray, overrides orange."""
    return ((1.0, 0.45, 0.0) if source_kind == "OVERTAKE_OVERRIDE"
            else (0.55, 0.55, 0.55))


def compact_hud_status(value):
    """Display-only abbreviations; the full, unmodified status stays on its ROS topic."""
    raw = str(value or "NOT_AVAILABLE")
    aliases = {
        "NOT_AVAILABLE": "N/A",
        "LOCAL_SPEED_CAPPED": "SPEED CAPPED",
        "QUALIFICATION_CAP_ACTIVE": "QUALIF. CAP",
        "NORMAL_TRACKING": "TRACKING",
        "HISTORICAL DISABLED": "DISABLED",
    }
    if raw in aliases:
        return aliases[raw]
    return raw if len(raw) <= 14 else "SEE RAW TOPIC"


def short_reference_id(reference_id):
    return reference_id.rsplit(":", 1)[-1][:8]


def active_target_speed(active, controller):
    if active is None or controller is None or not active.samples:
        return None
    return interpolate_periodic_scalar(
        tuple(sample.s_m for sample in active.samples),
        tuple(sample.target_speed_mps for sample in active.samples),
        controller.target_progress_m,
        active.station_period_m,
    )


def phase_at_least(review_phase, phase):
    return PHASES.index(review_phase) >= PHASES.index(phase)


def opponent_status(review_phase, message):
    if not phase_at_least(review_phase, "R-C"):
        return "INACTIVE"
    if message is None:
        return "NO TARGET"
    if message.valid:
        return (f"TRACKED {message.opponent_id}  conf {message.confidence:.2f}  "
                f"age {message.measurement_age_s:.2f}s")
    return "LOST / INVALID" if message.lifecycle == "LOST" else "INVALID"


def prediction_status(review_phase, message):
    if not phase_at_least(review_phase, "R-D"):
        return "INACTIVE"
    if message is None:
        return "NOT_AVAILABLE"
    if message.valid:
        return f"VALID  {message.predictor}  age {message.source_age_s:.2f}s"
    return "STALE / INVALID"


def opponent_estimate_point(message):
    return message.x_m, message.y_m


def prediction_points(message):
    return [(sample.x_m, sample.y_m, 0.12) for sample in message.samples]


def start_finish_points(sample, half_width_m=1.0):
    dx = -math.sin(sample.heading_rad) * half_width_m
    dy = math.cos(sample.heading_rad) * half_width_m
    return [(sample.x_m - dx, sample.y_m - dy, 0.06),
            (sample.x_m + dx, sample.y_m + dy, 0.06)]


class SpectatorHud(Node):
    """Evaluator-truth spectator output; publishes visualization only."""

    def __init__(self):
        super().__init__("h2h_spectator_hud")
        self.declare_parameter("review_phase", "FOUNDATION")
        self.declare_parameter("solo", False)
        self.declare_parameter("execution_speed_cap_mps", 2.0)
        self.declare_parameter("trajectory_candidate", "baseline")
        self.declare_parameter("nominal_policy_file", "")
        self.declare_parameter("raceline_data_dir", "")
        self.declare_parameter("expected_track_id", "iccas2025")
        self.declare_parameter("expected_map_sha256", "")
        self.review_phase = self.get_parameter("review_phase").value.upper()
        if self.review_phase not in PHASES:
            raise ValueError(f"review_phase must be one of {PHASES}")
        self.solo = self.get_parameter("solo").value
        self.execution_speed_cap_mps = self.get_parameter("execution_speed_cap_mps").value
        candidate_name = self.get_parameter("trajectory_candidate").value
        reference_share = Path(get_package_share_directory("f1tenth_racing_reference"))
        policy_path = (Path(self.get_parameter("nominal_policy_file").value)
            if self.get_parameter("nominal_policy_file").value
            else reference_share / "config/nominal_reference.json")
        policy = json.loads(policy_path.read_text())
        if (policy.get("track_id") != self.get_parameter("expected_track_id").value or
                (self.get_parameter("expected_map_sha256").value and
                 policy.get("map_image_sha256") != self.get_parameter("expected_map_sha256").value)):
            raise ValueError("HUD configured map/track does not match Reference policy")
        if candidate_name not in policy["candidates"]:
            raise ValueError(f"unknown HUD trajectory_candidate: {candidate_name!r}")
        candidate = policy["candidates"][candidate_name]
        share = (Path(self.get_parameter("raceline_data_dir").value)
            if self.get_parameter("raceline_data_dir").value
            else Path(get_package_share_directory("f1tenth_racing_raceline")) / "data")
        self.trajectory = load_racing_trajectory(
            share / candidate["csv"], share / candidate["metadata"],
        )
        if (self.trajectory.trajectory_id != candidate["source_trajectory_id"]
                or self.trajectory.metadata.get("track_id") != policy["track_id"]
                or self.trajectory.metadata.get("frame_id") != policy["frame_id"]
                or self.trajectory.metadata.get("source", {}).get("map_image_sha256")
                != policy["map_image_sha256"]
                or self.trajectory.metadata.get("closed_loop") is not True):
            raise ValueError("HUD nominal candidate provenance/closed-loop mismatch")
        self.trajectory_candidate = candidate_name
        self.expected_nominal_trajectory_id = candidate["source_trajectory_id"]
        self.nominal_loop_points = closed_nominal_points(self.trajectory)
        self.track = ClosedTrackGeometry([(sample.x_m, sample.y_m) for sample in self.trajectory.samples])
        self.publisher = self.create_publisher(MarkerArray, "spectator_hud", 1)
        self.blue = self.red = self.active = None
        self.controller = self.opponent = self.prediction = None
        self.vehicle_collision = None
        self.wall_collisions = [None, None]
        self.review_validity = LapCollisionReview()
        self.prediction_marker_count = 0
        self.laps = [0, 0]
        self.speeds = [0.0, 0.0]
        self.current_lap = self.best_lap = self.total_elapsed = 0.0
        self.elapsed = self.lap_start_elapsed = 0.0
        self.total_start_elapsed = None
        self.valid = True
        self.was_behind = False
        self.passes = 0
        self.create_subscription(CarState, "state0", lambda message: self.state(0, message), 1)
        self.create_subscription(CarState, "state1", lambda message: self.state(1, message), 1)
        for car_id in range(2):
            self.create_subscription(Int32, f"lap_count{car_id}",
                                     lambda message, i=car_id: self.set_lap(i, message.data), 1)
            self.create_subscription(Float64, f"speed{car_id}",
                                     lambda message, i=car_id: self.set_speed(i, message.data), 1)
        self.create_subscription(Float64, "lap_time0", self.lap_time, 10)
        self.create_subscription(Float64, "race_elapsed_time", self.elapsed_time, 1)
        self.create_subscription(Bool, "race_valid", lambda m: setattr(self, "valid", m.data), 1)
        self.create_subscription(Bool, "vehicle_collision", self.receive_collision, 1)
        self.create_subscription(Bool, "collision0", self.receive_wall_level, 1)
        for car_id in range(2):
            self.create_subscription(
                Int32, f"wall_collision_count{car_id}",
                lambda message, i=car_id: self.wall_collisions.__setitem__(i, message.data), 1,
            )
        self.create_subscription(ControllerStatus, "controller_status0",
                                 lambda m: setattr(self, "controller", m), 10)
        self.create_subscription(OpponentState, "opponent_state",
                                 lambda m: setattr(self, "opponent", m), 10)
        self.create_subscription(OpponentPrediction, "opponent_prediction",
                                 lambda m: setattr(self, "prediction", m), 10)
        transient = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ActiveTargetTrajectory, "active_target_trajectory",
            lambda message: setattr(self, "active", message), transient,
        )
        self.create_timer(0.1, self.publish)

    def set_lap(self, car_id, value):
        previous_lap = self.laps[car_id]
        self.laps[car_id] = value
        if car_id == 0:
            self.review_validity.set_lap(value)

        # Blue LAP 0 means the race has started but the start line
        # has not yet been crossed.  The first 0 -> 1 transition
        # starts the first measured lap.
        if car_id == 0 and previous_lap == 0 and value == 1:
            self.lap_start_elapsed = self.elapsed
            self.total_start_elapsed = self.elapsed
            self.current_lap = 0.0
            self.total_elapsed = 0.0

    def set_speed(self, car_id, value):
        self.speeds[car_id] = value

    def lap_time(self, message):
        if self.valid:
            self.best_lap = (
                message.data if self.best_lap == 0.0
                else min(self.best_lap, message.data)
            )
            self.lap_start_elapsed = self.elapsed

    def receive_collision(self, message):
        self.vehicle_collision = message.data
        self.review_validity.update(vehicle=message.data)

    def receive_wall_level(self, message):
        self.review_validity.update(wall=message.data)

    def elapsed_time(self, message):
        self.elapsed = message.data

        # Do not count the short launch-to-start-line approach as a lap.
        if self.laps[0] == 0:
            self.current_lap = 0.0
            self.total_elapsed = self.elapsed
        else:
            self.current_lap = max(
                0.0, self.elapsed - self.lap_start_elapsed
            )
            self.total_elapsed = self.elapsed

    def state(self, car_id, message):
        if car_id == 0:
            self.blue = message
        else:
            self.red = message
        if self.blue is not None and self.red is not None:
            blue_s = self.track.project(self.blue.px, self.blue.py).s
            red_s = self.track.project(self.red.px, self.red.py).s
            delta = self.track.signed_delta_s(red_s, blue_s)
            if delta < -1.0:
                self.was_behind = True
            elif self.was_behind and delta > 1.0:
                self.passes += 1
                self.was_behind = False

    def publish(self):
        if self.blue is None or (self.red is None and not self.solo):
            return
        blue_s = self.track.project(self.blue.px, self.blue.py).s
        red_s = self.track.project(self.red.px, self.red.py).s if self.red else None
        # Seam-safe relative race position.
        # Positive => Blue ahead of Red.
        # Negative => Blue behind Red.
        relative_progress = (
            self.track.signed_delta_s(red_s, blue_s) if red_s is not None else None
        )
        # ============================================================
        # SPECTATOR HUD — SCREEN-ALIGNED CELL GRID
        #
        # Each visible cell is an independent single-line marker.
        # Label/value/unit cells in the same row share one row anchor.
        #
        # Column movement follows the spectator camera's SCREEN-RIGHT
        # direction instead of raw world +X.  This prevents the previous
        # perspective-induced vertical drift between columns.
        # ============================================================

        xs = [sample.x_m for sample in self.trajectory.samples]
        ys = [sample.y_m for sample in self.trajectory.samples]

        track_min_x = min(xs)
        track_max_x = max(xs)
        track_min_y = min(ys)
        track_max_y = max(ys)
        track_width = max(track_max_x - track_min_x, 1.0)

        # ============================================================
        # HUD MANUAL TUNING
        # ============================================================

        HUD_TEXT_SCALE = 0.40

        # Must match h2h_spectator.rviz Current View Yaw.
        HUD_VIEW_YAW_RAD = 4.400000095367432

        # Whole board location.
        HUD_MIN_RIGHT_MARGIN_M = 5.0
        HUD_RIGHT_MARGIN_RATIO = 0.75
        HUD_TABLE_UP_M = 3.25

        # Horizontal screen-space column positions.
        HUD_LABEL_COLUMN_M = 0.0
        HUD_VALUE_COLUMN_M = 3.85
        HUD_UNIT_COLUMN_M = 5.65

        # Vertical row spacing in screen space.
        HUD_ROW_SPACING_M = 0.62

        # Title position relative to table label origin.
        HUD_TITLE_UP_M = 1.55
        HUD_TITLE_RIGHT_M = 1.75

        # ------------------------------------------------------------
        # Convert screen directions into world XY vectors.
        #
        # With XYOrbit:
        #
        # screen-right = (-sin(yaw), cos(yaw))
        # screen-down  = ( cos(yaw), sin(yaw))
        #
        # Therefore moving to another column does NOT also move it
        # vertically on the screen.
        # ------------------------------------------------------------

        screen_right_x = -math.sin(HUD_VIEW_YAW_RAD)
        screen_right_y = math.cos(HUD_VIEW_YAW_RAD)

        screen_down_x = math.cos(HUD_VIEW_YAW_RAD)
        screen_down_y = math.sin(HUD_VIEW_YAW_RAD)

        screen_up_x = -screen_down_x
        screen_up_y = -screen_down_y

        hud_origin_x = (
            track_max_x
            + max(
                HUD_MIN_RIGHT_MARGIN_M,
                track_width * HUD_RIGHT_MARGIN_RATIO,
            )
        )
        hud_origin_y = (track_min_y + track_max_y) * 0.5

        # Move table upward in SCREEN space.
        table_x = hud_origin_x + screen_up_x * HUD_TABLE_UP_M
        table_y = hud_origin_y + screen_up_y * HUD_TABLE_UP_M

        position = None if relative_progress is None else (1 if relative_progress >= 0.0 else 2)
        gap_m = None if relative_progress is None else abs(relative_progress)

        header_template = Marker()
        header_template.header.frame_id = "map"
        header_template.header.stamp = self.get_clock().now().to_msg()

        def make_text_marker(namespace, marker_id, x, y, text):
            output = Marker()
            output.header = header_template.header
            output.ns = namespace
            output.id = marker_id
            output.type = Marker.TEXT_VIEW_FACING
            output.action = Marker.ADD

            output.pose.position.x = x
            output.pose.position.y = y
            output.pose.position.z = 2.5
            output.pose.orientation.w = 1.0

            output.scale.z = HUD_TEXT_SCALE

            output.color.r = 1.0
            output.color.g = 1.0
            output.color.b = 1.0
            output.color.a = 1.0

            output.text = text
            return output

        def screen_point(row_slot, column_m):
            # First move downward to this row.
            row_x = table_x + screen_down_x * (
                row_slot * HUD_ROW_SPACING_M
            )
            row_y = table_y + screen_down_y * (
                row_slot * HUD_ROW_SPACING_M
            )

            # Then move horizontally to the requested column.
            return (
                row_x + screen_right_x * column_m,
                row_y + screen_right_y * column_m,
            )

        hud_markers = []

        # ------------------------------------------------------------
        # Delete HUD layouts used by earlier iterations.
        # ------------------------------------------------------------

        for namespace, marker_id in (
            ("spectator", 0),
            ("spectator_hud_title", 100),
            ("spectator_hud_labels", 101),
            ("spectator_hud_values", 102),
            ("spectator_hud_units", 103),
        ):
            delete = Marker()
            delete.header = header_template.header
            delete.ns = namespace
            delete.id = marker_id
            delete.action = Marker.DELETE
            hud_markers.append(delete)

        # ------------------------------------------------------------
        # Title
        # ------------------------------------------------------------

        title_x = (
            table_x
            + screen_up_x * HUD_TITLE_UP_M
            + screen_right_x * HUD_TITLE_RIGHT_M
        )
        title_y = (
            table_y
            + screen_up_y * HUD_TITLE_UP_M
            + screen_right_y * HUD_TITLE_RIGHT_M
        )

        hud_markers.append(
            make_text_marker(
                "spectator_hud_grid",
                200,
                title_x,
                title_y,
                f"F1TENTH   {self.review_phase}   REVIEW",
            )
        )

        # ------------------------------------------------------------
        # Rows
        #
        # slot numbers intentionally leave visual gaps between groups.
        # ------------------------------------------------------------

        active_source = self.active.source_kind if self.active is not None else "INACTIVE"
        active_identity = (
            f"{short_reference_id(self.active.active_reference_id)} / r{self.active.active_revision}"
            if self.active is not None else "NOT_AVAILABLE"
        )
        controller_mode = self.controller.mode if self.controller is not None else (
            "INACTIVE" if self.review_phase == "FOUNDATION" else "NOT_AVAILABLE"
        )
        lateral_error = (
            f"{self.controller.lateral_error_m:.3f}" if self.controller is not None
            else "NOT_AVAILABLE"
        )
        nominal_target = active_target_speed(self.active, self.controller)
        lap_valid = "VALID" if self.review_validity.lap_valid else "INVALID"
        collision_count = str(self.review_validity.episode_id)
        execution_feasible = (
            None if self.controller is None or
            not math.isfinite(self.controller.execution_envelope_mps)
            else self.controller.execution_envelope_mps
        )
        b1_target = (
            None if self.controller is None else self.controller.target_speed_mps
        )
        slip = None if self.blue is None else self.blue.slip_angle

        timing_rows = [
            # slot, label, value, unit
            (0,  "LAP",       str(self.laps[0]),              ""),
            (1,  "CURRENT",   f"{self.current_lap:.2f}",      "s"),
            (2,  "BEST",      f"{self.best_lap:.2f}",         "s"),
            (3,  "TOTAL",     format_duration(self.total_elapsed), ""),
        ]
        if self.solo and self.review_phase == "R-B":
            rows = timing_rows + [
                (5,  "PHASE",         self.review_phase, ""),
                (6,  "R-A TARGET",    "NOT_AVAILABLE" if nominal_target is None else f"{nominal_target:.2f}", "" if nominal_target is None else "m/s"),
                (7,  "EXEC SPEED", "N/A" if execution_feasible is None else f"{execution_feasible:.2f}", "" if execution_feasible is None else "m/s"),
                (8,  "B1 TARGET",     "NOT_AVAILABLE" if b1_target is None else f"{b1_target:.2f}", "" if b1_target is None else "m/s"),
                (9,  "BLUE ACTUAL",   f"{self.speeds[0]:.2f}", "m/s"),
                (11, "ACTIVE",        active_source, ""),
                (12, "REF",           active_identity, ""),
                (13, "MODE",          compact_hud_status(controller_mode), ""),
                (14, "LIMIT",         compact_hud_status(None if self.controller is None else self.controller.local_speed_limit_reason), ""),
                (15, "e_y",           lateral_error, "" if lateral_error == "NOT_AVAILABLE" else "m"),
                (16, "SLIP",          "NOT_AVAILABLE" if slip is None else f"{slip:.3f}", "" if slip is None else "rad"),
                (17, "NOMINAL ID",    nominal_identity_status(self.active, self.expected_nominal_trajectory_id), ""),
                (18, "R-C",           opponent_status(self.review_phase, self.opponent), ""),
                (19, "R-D",           prediction_status(self.review_phase, self.prediction), ""),
                (20, "R-E",           "HISTORICAL DISABLED", ""),
                (22, "LAP VALID",     lap_valid, ""),
                (23, "COLLISIONS",    collision_count, ""),
            ]
        elif self.solo:
            rows = timing_rows + [
            (5,  "PHASE",      self.review_phase,              ""),
            (6,  "R-A TARGET", "NOT_AVAILABLE" if nominal_target is None else f"{nominal_target:.2f}", "" if nominal_target is None else "m/s"),
            (7,  "EXEC CAP",   f"{self.execution_speed_cap_mps:.1f}", "m/s"),
            (8,  "BLUE ACTUAL", f"{self.speeds[0]:.2f}",      "m/s"),
            (10, "ACTIVE",     active_source,                  ""),
            (11, "REF",        active_identity,                ""),
            (12, "R-B",        controller_mode,                ""),
            (13, "e_y",        lateral_error,                  "" if lateral_error == "NOT_AVAILABLE" else "m"),
            (15, "R-C",        opponent_status(self.review_phase, self.opponent), ""),
            (16, "R-D",        prediction_status(self.review_phase, self.prediction), ""),
            (17, "R-E",        "HISTORICAL DISABLED",         ""),
            (19, "LAP VALID",  lap_valid,                      ""),
            (20, "COLLISIONS", collision_count,                ""),
            ]
        else:
            rows = timing_rows + [
                (5, "TRUTH BLUE", f"{self.speeds[0]:.2f}", "m/s"),
                (6, "TRUTH RED", f"{self.speeds[1]:.2f}", "m/s"),
                (7, "TRUTH POSITION", f"P{position}", ""),
                (8, "TRUTH GAP", f"{gap_m:.1f}", "m"),
                (10, "PHASE", self.review_phase, ""),
                (11, "ACTIVE", active_source, ""),
                (12, "REF", active_identity, ""),
                (13, "R-B", controller_mode, ""),
                (14, "e_y", lateral_error, "" if lateral_error == "NOT_AVAILABLE" else "m"),
                (15, "R-C", opponent_status(self.review_phase, self.opponent), ""),
                (16, "R-D", prediction_status(self.review_phase, self.prediction), ""),
                (17, "R-E", "HISTORICAL DISABLED", ""),
                (19, "LAP VALID", lap_valid, ""),
                (20, "COLLISIONS", collision_count, ""),
            ]

        marker_id = 210

        for row_slot, label, value, unit in rows:
            label_x, label_y = screen_point(
                row_slot,
                HUD_LABEL_COLUMN_M,
            )
            value_x, value_y = screen_point(
                row_slot,
                HUD_VALUE_COLUMN_M,
            )

            hud_markers.append(
                make_text_marker(
                    "spectator_hud_grid",
                    marker_id,
                    label_x,
                    label_y,
                    label,
                )
            )
            marker_id += 1

            hud_markers.append(
                make_text_marker(
                    "spectator_hud_grid",
                    marker_id,
                    value_x,
                    value_y,
                    value,
                )
            )
            marker_id += 1

            if unit:
                unit_x, unit_y = screen_point(
                    row_slot,
                    HUD_UNIT_COLUMN_M,
                )

                hud_markers.append(
                    make_text_marker(
                        "spectator_hud_grid",
                        marker_id,
                        unit_x,
                        unit_y,
                        unit,
                    )
                )
                marker_id += 1

        start_finish = Marker()
        start_finish.header = header_template.header
        start_finish.ns = "start_finish"
        start_finish.id = 2
        start_finish.type = Marker.LINE_STRIP
        start_finish.action = Marker.ADD
        start_finish.pose.orientation.w = 1.0
        start_finish.scale.x = 0.12
        start_finish.color.r = start_finish.color.g = start_finish.color.b = 1.0
        start_finish.color.a = 1.0
        start_finish.points = [Point(x=x, y=y, z=z)
                               for x, y, z in start_finish_points(self.trajectory.samples[0])]
        # The full selected Nominal is closed independently of the local Active
        # Reference. In particular, NEVER append the first point to PASS/ABORT.
        nominal_loop = Marker()
        nominal_loop.header = header_template.header
        nominal_loop.ns = "full_nominal_closed_loop"
        nominal_loop.id = 1
        nominal_loop.type = Marker.LINE_STRIP
        nominal_loop.action = Marker.ADD
        nominal_loop.pose.orientation.w = 1.0
        nominal_loop.scale.x = 0.075
        nominal_loop.color.r = 0.0
        nominal_loop.color.g = 0.8
        nominal_loop.color.b = 1.0
        nominal_loop.color.a = 1.0
        nominal_loop.points = [Point(x=x, y=y, z=z) for x, y, z in self.nominal_loop_points]
        markers = hud_markers + [start_finish, nominal_loop]
        opponent_delete = Marker()
        opponent_delete.header = header_template.header
        opponent_delete.ns = "r_c_estimate"
        opponent_delete.id = 300
        opponent_delete.action = Marker.DELETE
        markers.append(opponent_delete)
        opponent_label_delete = Marker()
        opponent_label_delete.header = header_template.header
        opponent_label_delete.ns = "r_c_estimate"
        opponent_label_delete.id = 301
        opponent_label_delete.action = Marker.DELETE
        markers.append(opponent_label_delete)
        if (phase_at_least(self.review_phase, "R-C") and self.opponent is not None
                and self.opponent.valid):
            estimate = Marker()
            estimate.header = self.opponent.header
            estimate.ns = "r_c_estimate"
            estimate.id = 300
            estimate.type = Marker.SPHERE
            estimate.action = Marker.ADD
            estimate.pose.position.x, estimate.pose.position.y = opponent_estimate_point(
                self.opponent
            )
            estimate.pose.position.z = 0.25
            estimate.pose.orientation.w = 1.0
            diameter = max(0.35, self.opponent.measured_extent_m)
            estimate.scale.x = estimate.scale.y = diameter
            estimate.scale.z = 0.35
            estimate.color.r = 1.0
            estimate.color.g = 0.0
            estimate.color.b = 1.0
            estimate.color.a = 0.85
            label = make_text_marker(
                "r_c_estimate", 301, self.opponent.x_m, self.opponent.y_m,
                f"R-C {self.opponent.opponent_id} {self.opponent.confidence:.2f}",
            )
            label.header = self.opponent.header
            label.pose.position.z = 0.8
            markers.extend((estimate, label))

        for marker_id in range(400, 401 + self.prediction_marker_count):
            delete = Marker()
            delete.header = header_template.header
            delete.ns = "r_d_prediction"
            delete.id = marker_id
            delete.action = Marker.DELETE
            markers.append(delete)
        self.prediction_marker_count = 0
        if (phase_at_least(self.review_phase, "R-D") and self.prediction is not None
                and self.prediction.valid):
            predicted_path = Marker()
            predicted_path.header = self.prediction.header
            predicted_path.ns = "r_d_prediction"
            predicted_path.id = 400
            predicted_path.type = Marker.LINE_STRIP
            predicted_path.action = Marker.ADD
            predicted_path.pose.orientation.w = 1.0
            predicted_path.scale.x = 0.08
            predicted_path.color.r = 1.0
            predicted_path.color.g = 0.85
            predicted_path.color.b = 0.0
            predicted_path.color.a = 1.0
            predicted_path.points = [Point(x=x, y=y, z=z)
                                     for x, y, z in prediction_points(self.prediction)]
            markers.append(predicted_path)
            for index, sample in enumerate(self.prediction.samples, start=1):
                occupancy = Marker()
                occupancy.header = self.prediction.header
                occupancy.ns = "r_d_prediction"
                occupancy.id = 400 + index
                occupancy.type = Marker.CUBE
                occupancy.action = Marker.ADD
                occupancy.pose.position.x = sample.x_m
                occupancy.pose.position.y = sample.y_m
                occupancy.pose.position.z = 0.08
                occupancy.pose.orientation.z = math.sin(sample.heading_rad * 0.5)
                occupancy.pose.orientation.w = math.cos(sample.heading_rad * 0.5)
                occupancy.scale.x = 2.0 * sample.occupancy_half_length_m
                occupancy.scale.y = 2.0 * sample.occupancy_half_width_m
                occupancy.scale.z = 0.08
                occupancy.color.r = 1.0
                occupancy.color.g = 0.85
                occupancy.color.b = 0.0
                occupancy.color.a = 0.12
                markers.append(occupancy)
            self.prediction_marker_count = len(self.prediction.samples)
        if self.active is not None:
            path = Marker()
            path.header = header_template.header
            path.ns = "active_reference"
            path.id = 1
            path.type = Marker.LINE_STRIP
            path.action = Marker.ADD
            path.pose.orientation.w = 1.0
            is_override = self.active.source_kind == "OVERTAKE_OVERRIDE"
            path.scale.x = 0.075 if is_override else 0.022
            path.color.r, path.color.g, path.color.b = active_reference_color(
                self.active.source_kind
            )
            path.color.a = 1.0 if is_override else 0.35
            path.points = [Point(x=x, y=y, z=z) for x, y, z in active_reference_points(
                self.active, z=0.12 if is_override else 0.025
            )]
            markers.append(path)
        self.publisher.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = SpectatorHud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
