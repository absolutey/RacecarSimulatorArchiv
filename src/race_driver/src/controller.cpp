#include "race_driver/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace race_driver
{

void Controller::setParameters(
  double lookahead_distance,
  double speed_kp,
  double max_steer,
  double max_accel,
  double wheelbase,
  double steer_rate_limit,
  double control_dt,
  double low_speed_deadband)
{
  lookahead_distance_ = lookahead_distance;
  speed_kp_ = speed_kp;
  max_steer_ = max_steer;
  max_accel_ = max_accel;
  wheelbase_ = wheelbase;
  steer_rate_limit_ = steer_rate_limit;
  control_dt_ = control_dt;
  low_speed_deadband_ = low_speed_deadband;
}

double Controller::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double Controller::clamp(double value, double min_value, double max_value) const
{
  return std::max(min_value, std::min(max_value, value));
}

bool Controller::findTrackingTarget(
  const EgoState & ego_state,
  const LocalPath & local_path,
  double lookahead,
  LocalPathPoint & target) const
{
  if (!local_path.valid || local_path.points.empty()) {
    return false;
  }

  double best_dist = std::numeric_limits<double>::max();
  std::size_t nearest_idx = 0;

  for (std::size_t i = 0; i < local_path.points.size(); ++i) {
    const auto & p = local_path.points[i];
    const double dx = p.x - ego_state.x;
    const double dy = p.y - ego_state.y;
    const double dist = std::hypot(dx, dy);

    if (dist < best_dist) {
      best_dist = dist;
      nearest_idx = i;
    }
  }

  for (std::size_t i = nearest_idx; i < local_path.points.size(); ++i) {
    const auto & p = local_path.points[i];
    const double dx = p.x - ego_state.x;
    const double dy = p.y - ego_state.y;
    const double dist = std::hypot(dx, dy);

    if (dist >= lookahead) {
      target = p;
      return true;
    }
  }

  target = local_path.points.back();
  return true;
}

bool Controller::findNearestPathPoint(
  const EgoState & ego_state,
  const LocalPath & local_path,
  LocalPathPoint & nearest,
  double & signed_cross_track_error,
  double & heading_error) const
{
  if (!local_path.valid || local_path.points.empty()) {
    return false;
  }

  double best_dist = std::numeric_limits<double>::max();
  std::size_t best_idx = 0;

  for (std::size_t i = 0; i < local_path.points.size(); ++i) {
    const auto & p = local_path.points[i];
    const double dx = ego_state.x - p.x;
    const double dy = ego_state.y - p.y;
    const double dist = std::hypot(dx, dy);

    if (dist < best_dist) {
      best_dist = dist;
      best_idx = i;
    }
  }

  nearest = local_path.points[best_idx];

  const double dx = ego_state.x - nearest.x;
  const double dy = ego_state.y - nearest.y;

  const double nx = -std::sin(nearest.yaw);
  const double ny = std::cos(nearest.yaw);

  signed_cross_track_error = dx * nx + dy * ny;
  heading_error = normalizeAngle(nearest.yaw - ego_state.yaw);

  return true;
}

double Controller::computePurePursuitSteer(
  const EgoState & ego_state,
  const LocalPathPoint & target,
  double lookahead) const
{
  const double dx = target.x - ego_state.x;
  const double dy = target.y - ego_state.y;

  const double cos_yaw = std::cos(ego_state.yaw);
  const double sin_yaw = std::sin(ego_state.yaw);

  const double x_local = cos_yaw * dx + sin_yaw * dy;
  const double y_local = -sin_yaw * dx + cos_yaw * dy;

  const double ld = std::max(0.2, lookahead);

  if (x_local < -0.1) {
    return 0.0;
  }

  const double curvature = 2.0 * y_local / (ld * ld);
  return std::atan(wheelbase_ * curvature);
}

double Controller::computeStanleySteer(
  double signed_cross_track_error,
  double heading_error,
  double speed) const
{
  const double v = std::max(0.1, speed);

  const double cte_term = std::atan2(
    stanley_gain_ * signed_cross_track_error,
    v + stanley_softening_speed_);

  return heading_error - cte_term;
}

ControlCommand Controller::compute(
  const EgoState & ego_state,
  const LocalPath & local_path,
  double target_speed)
{
  ControlCommand cmd;

  if (!ego_state.valid || !local_path.valid || local_path.points.empty()) {
    last_steer_ = 0.0;
    return cmd;
  }

  const double curvature_factor = clamp(local_path.max_curvature, 0.0, 1.2);
  const double speed_factor = clamp(ego_state.speed / 5.5, 0.0, 1.0);

  double dynamic_lookahead =
    lookahead_distance_ +
    0.30 * ego_state.speed -
    0.20 * curvature_factor;

  dynamic_lookahead = clamp(dynamic_lookahead, 1.70, 3.90);

  LocalPathPoint target;
  if (!findTrackingTarget(ego_state, local_path, dynamic_lookahead, target)) {
    last_steer_ = 0.0;
    return cmd;
  }

  LocalPathPoint nearest;
  double signed_cross_track_error = 0.0;
  double heading_error = 0.0;

  if (!findNearestPathPoint(
      ego_state,
      local_path,
      nearest,
      signed_cross_track_error,
      heading_error))
  {
    last_steer_ = 0.0;
    return cmd;
  }

  const double pp_steer =
    computePurePursuitSteer(ego_state, target, dynamic_lookahead);

  const double stanley_steer =
    computeStanleySteer(
      signed_cross_track_error,
      heading_error,
      ego_state.speed);

  const double heading_abs = std::abs(heading_error);

  const double stanley_weight = clamp(
    min_stanley_weight_ +
    0.12 * speed_factor +
    0.25 * clamp(heading_abs / 0.45, 0.0, 1.0),
    min_stanley_weight_,
    max_stanley_weight_);

  double steer_cmd =
    (1.0 - stanley_weight) * pp_steer +
    stanley_weight * stanley_steer;

  steer_cmd = clamp(steer_cmd, -max_steer_, max_steer_);

  const double max_delta = steer_rate_limit_ * std::max(1e-3, control_dt_);
  steer_cmd = clamp(
    steer_cmd,
    last_steer_ - max_delta,
    last_steer_ + max_delta);

  last_steer_ = steer_cmd;

  double accel_cmd = speed_kp_ * (target_speed - ego_state.speed);
  accel_cmd = clamp(accel_cmd, -max_accel_, max_accel_);

  if (target_speed < low_speed_deadband_) {
    accel_cmd = std::min(accel_cmd, 0.0);
  }

  cmd.steering = steer_cmd;
  cmd.acceleration = accel_cmd;
  cmd.target_speed = target_speed;
  cmd.valid = true;

  return cmd;
}

}  // namespace race_driver
