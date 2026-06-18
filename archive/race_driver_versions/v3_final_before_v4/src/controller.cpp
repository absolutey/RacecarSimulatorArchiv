#include "race_driver/controller.hpp"

#include <algorithm>
#include <cmath>

namespace race_driver
{

namespace
{
std::size_t findLookaheadIndex(const LocalPath & local_path, double lookahead_distance)
{
  if (local_path.points.empty()) {
    return 0;
  }
  double accum_dist = 0.0;
  for (std::size_t i = 1; i < local_path.points.size(); ++i) {
    const double dx = local_path.points[i].x - local_path.points[i - 1].x;
    const double dy = local_path.points[i].y - local_path.points[i - 1].y;
    accum_dist += std::sqrt(dx * dx + dy * dy);
    if (accum_dist >= lookahead_distance) {
      return i;
    }
  }
  return local_path.points.size() - 1;
}
}  // namespace

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

ControlCommand Controller::compute(
  const EgoState & ego_state,
  const LocalPath & local_path,
  double target_speed)
{
  ControlCommand cmd;
  cmd.target_speed = target_speed;
  if (!ego_state.valid || !local_path.valid || local_path.points.size() < 2) {
    prev_steering_ = 0.0;
    return cmd;
  }

  const double dynamic_lookahead = std::clamp(
    lookahead_distance_ + 0.35 * std::max(0.0, ego_state.speed),
    1.0, 4.5);
  const std::size_t target_idx = findLookaheadIndex(local_path, dynamic_lookahead);
  const auto & target = local_path.points[target_idx];

  const double dx = target.x - ego_state.x;
  const double dy = target.y - ego_state.y;
  const double cos_yaw = std::cos(ego_state.yaw);
  const double sin_yaw = std::sin(ego_state.yaw);

  const double x_local = cos_yaw * dx + sin_yaw * dy;
  const double y_local = -sin_yaw * dx + cos_yaw * dy;
  const double ld = std::max(1e-3, std::sqrt(x_local * x_local + y_local * y_local));

  double steer_cmd = 0.0;
  if (std::abs(target_speed) > low_speed_deadband_ || ego_state.speed > low_speed_deadband_) {
    const double curvature = 2.0 * y_local / (ld * ld);
    steer_cmd = std::atan(wheelbase_ * curvature);
  }

  steer_cmd = std::clamp(steer_cmd, -max_steer_, max_steer_);
  const double max_delta = steer_rate_limit_ * std::max(1e-3, control_dt_);
  const double lower = prev_steering_ - max_delta;
  const double upper = prev_steering_ + max_delta;
  cmd.steering = std::clamp(steer_cmd, lower, upper);
  prev_steering_ = cmd.steering;

  const double accel_cmd = speed_kp_ * (target_speed - ego_state.speed);
  cmd.acceleration = std::clamp(accel_cmd, -max_accel_, max_accel_);
  if (target_speed < low_speed_deadband_ && ego_state.speed < low_speed_deadband_) {
    cmd.acceleration = std::min(0.0, cmd.acceleration);
  }

  cmd.valid = true;
  return cmd;
}

}  // namespace race_driver
