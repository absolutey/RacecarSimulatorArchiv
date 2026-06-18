#include "race_driver/race_driver_node.hpp"

#include <chrono>
#include <cmath>
#include <string>

namespace race_driver
{

using namespace std::chrono_literals;

RaceDriverNode::RaceDriverNode()
: Node("race_driver_node")
{
  const double control_rate_hz = this->declare_parameter<double>("control_rate_hz", 40.0);
  const double planning_horizon_m = this->declare_parameter<double>("planning_horizon_m", 14.0);
  const double lookahead_distance = this->declare_parameter<double>("lookahead_distance", 2.0);
  const double obstacle_margin = this->declare_parameter<double>("obstacle_margin", 0.20);
  const double shift_pre_margin_m = this->declare_parameter<double>("shift_pre_margin_m", 2.6);
  const double shift_post_margin_m = this->declare_parameter<double>("shift_post_margin_m", 3.2);
  const double max_peak_shift_m = this->declare_parameter<double>("max_peak_shift_m", 0.55);
  const double return_blend_length_m = this->declare_parameter<double>("return_blend_length_m", 1.8);
  const double left_right_cost_bias = this->declare_parameter<double>("left_right_cost_bias", 0.0);
  const bool use_racing_reference =
    this->declare_parameter<bool>("use_racing_reference", false);
  const std::string racing_reference_csv_path =
    this->declare_parameter<std::string>("racing_reference_csv_path", "");
  const double racing_reference_max_offset =
    this->declare_parameter<double>("racing_reference_max_offset", 0.45);
  const double racing_reference_blend =
    this->declare_parameter<double>("racing_reference_blend", 0.70);
  const double racing_reference_boundary_margin =
    this->declare_parameter<double>("racing_reference_boundary_margin", 0.25);

  const double max_speed = this->declare_parameter<double>("max_speed", 5.0);
  const double min_speed = this->declare_parameter<double>("min_speed", 2.0);
  const double curvature_speed_gain = this->declare_parameter<double>("curvature_speed_gain", 0.15);
  const double obstacle_slowdown_gain = this->declare_parameter<double>("obstacle_slowdown_gain", 0.10);
  const double brake_distance = this->declare_parameter<double>("brake_distance", 1.2);
  const double stop_distance = this->declare_parameter<double>("stop_distance", 0.28);
  const double avoid_speed_scale = this->declare_parameter<double>("avoid_speed_scale", 0.97);
  const double narrow_corridor_speed_scale = this->declare_parameter<double>("narrow_corridor_speed_scale", 0.88);
  const double max_lateral_accel = this->declare_parameter<double>("max_lateral_accel", 8.0);

  const double front_roi_min_x = this->declare_parameter<double>("front_roi_min_x", 0.0);
  const double front_roi_max_x = this->declare_parameter<double>("front_roi_max_x", 7.0);
  const double corridor_half_width = this->declare_parameter<double>("corridor_half_width", 0.62);
  const double cluster_distance_threshold = this->declare_parameter<double>("cluster_distance_threshold", 0.30);
  const int min_cluster_points = this->declare_parameter<int>("min_cluster_points", 3);
  const double wall_length_threshold = this->declare_parameter<double>("wall_length_threshold", 0.80);
  const double obstacle_width_margin = this->declare_parameter<double>("obstacle_width_margin", 0.14);
  const double corridor_block_threshold = this->declare_parameter<double>("corridor_block_threshold", 0.08);
  const double obstacle_forward_horizon_m = this->declare_parameter<double>("obstacle_forward_horizon_m", 6.5);

  const double speed_kp = this->declare_parameter<double>("speed_kp", 1.7);
  const double max_steer = this->declare_parameter<double>("max_steer", 0.4);
  const double max_accel = this->declare_parameter<double>("max_accel", 2.8);
  const double wheelbase = this->declare_parameter<double>("wheelbase", 0.33);
  const double steer_rate_limit = this->declare_parameter<double>("steer_rate_limit", 3.2);
  const double low_speed_deadband = this->declare_parameter<double>("low_speed_deadband", 0.15);

  odom_timeout_sec_ = this->declare_parameter<double>("odom_timeout_sec", 0.30);
  scan_timeout_sec_ = this->declare_parameter<double>("scan_timeout_sec", 0.30);
  track_timeout_sec_ = this->declare_parameter<double>("track_timeout_sec", 1.00);
  stop_on_invalid_path_ = this->declare_parameter<bool>("stop_on_invalid_path", false);
  debug_logging_ = this->declare_parameter<bool>("debug_logging", true);
  use_map_safety_check_ =
    this->declare_parameter<bool>("use_map_safety_check", false);
  map_vehicle_radius_m_ =
    this->declare_parameter<double>("map_vehicle_radius_m", 0.22);
  map_safety_speed_cap_ =
    this->declare_parameter<double>("map_safety_speed_cap", 3.2);

  const double map_rollout_horizon_m =
    this->declare_parameter<double>("map_rollout_horizon_m", 7.5);
  const double map_rollout_step_m =
    this->declare_parameter<double>("map_rollout_step_m", 0.25);
  const double map_rollout_vehicle_radius_m =
    this->declare_parameter<double>("map_rollout_vehicle_radius_m", 0.12);
  const double map_rollout_max_steer =
    this->declare_parameter<double>("map_rollout_max_steer", 0.40);
  const double map_rollout_wheelbase =
    this->declare_parameter<double>("map_rollout_wheelbase", 0.33);
  const double map_rollout_max_lateral_accel =
    this->declare_parameter<double>("map_rollout_max_lateral_accel", 9.5);
  const int map_rollout_steer_samples =
    this->declare_parameter<int>("map_rollout_steer_samples", 15);
  map_rollout_fallback_to_boundary_ =
    this->declare_parameter<bool>("map_rollout_fallback_to_boundary", true);
  planner_mode_ = this->declare_parameter<std::string>("planner_mode", "boundary_corridor");

  const double boundary_wall_margin_m =
    this->declare_parameter<double>("boundary_wall_margin_m", 0.28);
  const double boundary_obstacle_margin_m =
    this->declare_parameter<double>("boundary_obstacle_margin_m", 0.32);
  const double boundary_max_lateral_accel =
    this->declare_parameter<double>("boundary_max_lateral_accel", max_lateral_accel);

  const double v5_min_boundary_margin =
    this->declare_parameter<double>("v5_min_boundary_margin", 0.22);
  const double v5_smoothing_gain =
    this->declare_parameter<double>("v5_smoothing_gain", 0.35);
  const double v5_planning_horizon_m =
    this->declare_parameter<double>("v5_planning_horizon_m", 8.0);


  const double control_dt = 1.0 / std::max(1.0, control_rate_hz);

  obstacle_detector_.setParameters(
    front_roi_min_x, front_roi_max_x, corridor_half_width,
    cluster_distance_threshold, min_cluster_points,
    wall_length_threshold, obstacle_width_margin,
    corridor_block_threshold, obstacle_forward_horizon_m);
  boundary_corridor_planner_.setParameters(
    planning_horizon_m,
    boundary_wall_margin_m,
    boundary_obstacle_margin_m,
    boundary_max_lateral_accel);
  map_rollout_planner_.setParameters(
    map_rollout_horizon_m,
    map_rollout_step_m,
    map_rollout_vehicle_radius_m,
    map_rollout_max_steer,
    map_rollout_wheelbase,
    map_rollout_max_lateral_accel,
    map_rollout_steer_samples);
  raceline_planner_.setParameters(
    planning_horizon_m, obstacle_margin, shift_pre_margin_m,
    shift_post_margin_m, max_peak_shift_m,
    return_blend_length_m, left_right_cost_bias);

  raceline_planner_.setRacingReferenceOptions(
    racing_reference_max_offset,
    racing_reference_blend,
    racing_reference_boundary_margin);

  raceline_planner_.setUseRacingReference(use_racing_reference);

  if (use_racing_reference) {
    const bool loaded =
      raceline_planner_.loadRacingReferenceFromCsv(racing_reference_csv_path);

    if (loaded) {
      RCLCPP_INFO(
        this->get_logger(),
        "Racing reference offset hint loaded: %s",
        racing_reference_csv_path.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to load racing reference offset hint: %s. Falling back to /center_path.",
        racing_reference_csv_path.c_str());
      raceline_planner_.setUseRacingReference(false);
    }
  }

  v5_racing_planner_.configure(
    min_speed,
    max_speed,
    max_lateral_accel,
    max_accel,
    5.0,
    v5_min_boundary_margin,
    v5_smoothing_gain,
    v5_planning_horizon_m);

  speed_planner_.setParameters(
    max_speed, min_speed, curvature_speed_gain,
    obstacle_slowdown_gain, brake_distance, stop_distance,
    avoid_speed_scale, narrow_corridor_speed_scale, max_lateral_accel);
  controller_.setParameters(
    lookahead_distance, speed_kp, max_steer, max_accel,
    wheelbase, steer_rate_limit, control_dt, low_speed_deadband);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom0", 10,
    std::bind(&RaceDriverNode::odomCallback, this, std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan0", 10,
    std::bind(&RaceDriverNode::scanCallback, this, std::placeholders::_1));

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&RaceDriverNode::mapCallback, this, std::placeholders::_1));

  center_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/center_path", 10,
    std::bind(&RaceDriverNode::centerPathCallback, this, std::placeholders::_1));

  left_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/left_boundary", 10,
    std::bind(&RaceDriverNode::leftBoundaryCallback, this, std::placeholders::_1));

  right_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/right_boundary", 10,
    std::bind(&RaceDriverNode::rightBoundaryCallback, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    "/ackermann_cmd0", 10);

  local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/race_driver/local_path", 10);

  obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/race_driver/obstacle_markers", 10);

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&RaceDriverNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "race_driver_node started.");
}

void RaceDriverNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
  has_odom_ = true;
  last_odom_time_ = this->now();
  state_manager_.updateFromOdom(*msg);
}

void RaceDriverNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = *msg;
  has_scan_ = true;
  last_scan_time_ = this->now();
}

void RaceDriverNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  occupancy_grid_manager_.updateMap(*msg);
}

void RaceDriverNode::centerPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  track_manager_.updateCenterline(*msg);
  last_track_time_ = this->now();
}

void RaceDriverNode::leftBoundaryCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  track_manager_.updateLeftBoundary(*msg);
  last_track_time_ = this->now();
}

void RaceDriverNode::rightBoundaryCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  track_manager_.updateRightBoundary(*msg);
  last_track_time_ = this->now();
}

bool RaceDriverNode::inputsFresh() const
{
  const auto now = this->now();
  if (!has_odom_ || !has_scan_ || !track_manager_.isReady()) {
    return false;
  }
  if ((now - last_odom_time_).seconds() > odom_timeout_sec_) {
    return false;
  }
  if ((now - last_scan_time_).seconds() > scan_timeout_sec_) {
    return false;
  }
  if ((now - last_track_time_).seconds() > track_timeout_sec_) {
    return false;
  }
  return true;
}

void RaceDriverNode::publishZeroCommand()
{
  ackermann_msgs::msg::AckermannDriveStamped cmd_msg;
  cmd_msg.header.stamp = this->now();
  cmd_msg.drive.steering_angle = 0.0;
  cmd_msg.drive.speed = 0.0;
  cmd_msg.drive.acceleration = 0.0;
  cmd_pub_->publish(cmd_msg);
}

void RaceDriverNode::publishLocalPath(const LocalPath & local_path)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "map";

  if (local_path.valid) {
    path_msg.poses.reserve(local_path.points.size());

    for (const auto & p : local_path.points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.position.z = 0.05;

      const double half_yaw = 0.5 * p.yaw;
      pose.pose.orientation.z = std::sin(half_yaw);
      pose.pose.orientation.w = std::cos(half_yaw);

      path_msg.poses.push_back(pose);
    }
  }

  local_path_pub_->publish(path_msg);
}

void RaceDriverNode::publishObstacleMarkers(const ObstacleList & obstacles)
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.stamp = this->now();
  clear_marker.header.frame_id = "map";
  clear_marker.ns = "race_driver_obstacles";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  int id = 1;

  if (obstacles.valid) {
    for (const auto & obs : obstacles.obstacles) {
      if (!obs.valid) {
        continue;
      }

      visualization_msgs::msg::Marker marker;
      marker.header.stamp = this->now();
      marker.header.frame_id = "map";
      marker.ns = "race_driver_obstacles";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = obs.world_x;
      marker.pose.position.y = obs.world_y;
      marker.pose.position.z = 0.18;
      marker.pose.orientation.w = 1.0;

      const double size = std::max(0.18, obs.width);
      marker.scale.x = size;
      marker.scale.y = size;
      marker.scale.z = 0.25;

      if (obs.blocks_center_corridor) {
        marker.color.r = 1.0f;
        marker.color.g = 0.15f;
        marker.color.b = 0.05f;
        marker.color.a = 0.90f;
      } else {
        marker.color.r = 1.0f;
        marker.color.g = 0.85f;
        marker.color.b = 0.05f;
        marker.color.a = 0.55f;
      }

      marker.lifetime = rclcpp::Duration::from_seconds(0.25);
      marker_array.markers.push_back(marker);
    }
  }

  obstacle_marker_pub_->publish(marker_array);
}

void RaceDriverNode::controlLoop()
{
  if (!inputsFresh()) {
    if (debug_logging_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Input stale or track not ready. Publishing zero command.");
    }
    publishZeroCommand();
    return;
  }

  const EgoState ego_state = state_manager_.getState();
  const TrackModel track_model = track_manager_.getTrackModel();
  const ObstacleList obstacles = obstacle_detector_.detect(latest_scan_, ego_state, track_model);

  LocalPath local_path;

  if (planner_mode_ == "v5_raceline") {
    local_path = v5_racing_planner_.plan(ego_state, track_model);

    if (!local_path.valid) {
      local_path = map_rollout_planner_.plan(
        ego_state,
        occupancy_grid_manager_,
        obstacles);

      if (local_path.valid) {
        local_path.decision.reason = "v5_fallback_map_rollout";
      }
    }

    if (!local_path.valid && map_rollout_fallback_to_boundary_) {
      local_path = boundary_corridor_planner_.plan(ego_state, track_model, obstacles);

      if (local_path.valid) {
        local_path.decision.reason = "v5_fallback_boundary";
      }
    }
  } else if (planner_mode_ == "map_rollout") {
    local_path = map_rollout_planner_.plan(
      ego_state,
      occupancy_grid_manager_,
      obstacles);

    if (!local_path.valid && map_rollout_fallback_to_boundary_) {
      local_path = boundary_corridor_planner_.plan(ego_state, track_model, obstacles);

      if (local_path.valid) {
        local_path.decision.reason = "map_rollout_fallback_boundary";
      }
    }
  } else if (planner_mode_ == "boundary_corridor") {
    local_path = boundary_corridor_planner_.plan(ego_state, track_model, obstacles);
  } else {
    local_path = raceline_planner_.plan(ego_state, track_model, obstacles);
  }

  publishLocalPath(local_path);
  publishObstacleMarkers(obstacles);

  bool map_path_unsafe = false;

  if (
    use_map_safety_check_ &&
    occupancy_grid_manager_.valid() &&
    local_path.valid &&
    !occupancy_grid_manager_.isPathSafe(local_path, map_vehicle_radius_m_))
  {
    map_path_unsafe = true;
  }

  if (!local_path.valid) {
    if (debug_logging_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Local path invalid: %s", local_path.decision.reason.c_str());
    }
    if (stop_on_invalid_path_) {
      publishZeroCommand();
      return;
    }
  }

  double target_speed = speed_planner_.computeTargetSpeed(ego_state, local_path, obstacles);

  if (map_path_unsafe) {
    target_speed = std::min(target_speed, map_safety_speed_cap_);

    if (debug_logging_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Map safety soft-guard active. Speed capped to %.2f", target_speed);
    }
  }

  const ControlCommand control = controller_.compute(ego_state, local_path, target_speed);

  if (debug_logging_) {
    if (local_path.decision.blocking_obstacle.valid) {
      const auto & obs = local_path.decision.blocking_obstacle;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "decision=%s obs(s=%.2f,d=%.2f) free(L=%.2f,R=%.2f) peak_shift=%.2f v=%.2f kmax=%.3f",
        local_path.decision.reason.c_str(), obs.s, obs.d, obs.left_free, obs.right_free,
        local_path.decision.peak_shift, target_speed, local_path.max_curvature);
    } else {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "decision=%s v=%.2f kmax=%.3f",
        local_path.decision.reason.c_str(), target_speed, local_path.max_curvature);
    }
  }

  ackermann_msgs::msg::AckermannDriveStamped cmd_msg;
  cmd_msg.header.stamp = this->now();
  cmd_msg.drive.steering_angle = control.steering;
  cmd_msg.drive.speed = control.target_speed;
  cmd_msg.drive.acceleration = control.acceleration;
  cmd_pub_->publish(cmd_msg);
}

}  // namespace race_driver
