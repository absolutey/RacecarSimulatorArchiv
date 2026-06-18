#include "race_driver/mpc_lite_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace race_driver
{

void MpcLiteController::setParameters(
  int horizon_steps,
  double dt,
  double wheelbase,
  double max_steer,
  double max_accel,
  double steer_rate_limit,
  int steer_samples,
  double speed_delta,
  double path_weight,
  double heading_weight,
  double steer_weight,
  double steer_rate_weight,
  double speed_reward_weight,
  double clearance_weight,
  double clearance_margin)
{
  horizon_steps_ = std::max(3, horizon_steps);
  dt_ = std::max(0.01, dt);
  wheelbase_ = std::max(0.05, wheelbase);
  max_steer_ = std::max(0.05, max_steer);
  max_accel_ = std::max(0.1, max_accel);
  steer_rate_limit_ = std::max(0.1, steer_rate_limit);
  steer_samples_ = std::max(3, steer_samples);
  if (steer_samples_ % 2 == 0) {
    steer_samples_ += 1;
  }

  speed_delta_ = std::max(0.0, speed_delta);

  path_weight_ = std::max(0.0, path_weight);
  heading_weight_ = std::max(0.0, heading_weight);
  steer_weight_ = std::max(0.0, steer_weight);
  steer_rate_weight_ = std::max(0.0, steer_rate_weight);
  speed_reward_weight_ = std::max(0.0, speed_reward_weight);
  clearance_weight_ = std::max(0.0, clearance_weight);
  clearance_margin_ = std::max(0.0, clearance_margin);
}

double MpcLiteController::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double MpcLiteController::clamp(double value, double min_value, double max_value) const
{
  return std::max(min_value, std::min(max_value, value));
}

MpcLiteController::NearestPathInfo MpcLiteController::findNearestPathPoint(
  double x,
  double y,
  double yaw,
  const LocalPath & local_path) const
{
  NearestPathInfo info;

  if (!local_path.valid || local_path.points.empty()) {
    return info;
  }

  double best_dist = std::numeric_limits<double>::max();
  std::size_t best_idx = 0;

  for (std::size_t i = 0; i < local_path.points.size(); ++i) {
    const auto & p = local_path.points[i];
    const double d = std::hypot(x - p.x, y - p.y);

    if (d < best_dist) {
      best_dist = d;
      best_idx = i;
    }
  }

  const auto & p = local_path.points[best_idx];

  info.valid = true;
  info.point = p;
  info.distance = best_dist;
  info.heading_error = normalizeAngle(p.yaw - yaw);

  if (p.left_clearance > 0.01 && p.right_clearance > 0.01) {
    info.clearance = std::min(p.left_clearance, p.right_clearance);
  }

  return info;
}

double MpcLiteController::evaluateCandidate(
  const EgoState & ego_state,
  const LocalPath & local_path,
  double steer,
  double candidate_speed) const
{
  SimState s;
  s.x = ego_state.x;
  s.y = ego_state.y;
  s.yaw = ego_state.yaw;
  s.speed = std::max(0.0, candidate_speed);

  double cost = 0.0;

  const double steer_rate_proxy = std::abs(steer - last_steer_);

  for (int i = 0; i < horizon_steps_; ++i) {
    s.x += s.speed * std::cos(s.yaw) * dt_;
    s.y += s.speed * std::sin(s.yaw) * dt_;
    s.yaw = normalizeAngle(
      s.yaw + s.speed / wheelbase_ * std::tan(steer) * dt_);

    const auto nearest = findNearestPathPoint(s.x, s.y, s.yaw, local_path);

    if (!nearest.valid) {
      return std::numeric_limits<double>::max();
    }

    const double step_scale = 1.0 + 0.04 * static_cast<double>(i);

    cost += step_scale * path_weight_ * nearest.distance * nearest.distance;
    cost += step_scale * heading_weight_ *
      nearest.heading_error * nearest.heading_error;

    if (nearest.clearance < clearance_margin_) {
      const double lack = clearance_margin_ - nearest.clearance;
      cost += step_scale * clearance_weight_ * lack * lack * 8.0;
    }
  }

  cost += steer_weight_ * steer * steer;
  cost += steer_rate_weight_ * steer_rate_proxy * steer_rate_proxy;

  // 빠른 후보는 보상한다. 단, path/heading/clearance 비용이 더 우선이다.
  cost -= speed_reward_weight_ * candidate_speed;

  return cost;
}

ControlCommand MpcLiteController::compute(
  const EgoState & ego_state,
  const LocalPath & local_path,
  double target_speed)
{
  ControlCommand cmd;

  if (!ego_state.valid || !local_path.valid || local_path.points.size() < 4) {
    last_steer_ = 0.0;
    return cmd;
  }

  const double max_delta = steer_rate_limit_ * std::max(1e-3, dt_);

  double best_cost = std::numeric_limits<double>::max();
  double best_steer = 0.0;
  double best_speed = target_speed;

  std::vector<double> speed_candidates;
  speed_candidates.push_back(std::max(0.0, target_speed - speed_delta_));
  speed_candidates.push_back(std::max(0.0, target_speed));
  speed_candidates.push_back(std::max(0.0, target_speed + 0.5 * speed_delta_));

  for (int i = 0; i < steer_samples_; ++i) {
    const double ratio = steer_samples_ == 1
      ? 0.0
      : static_cast<double>(i) / static_cast<double>(steer_samples_ - 1);

    double steer = -max_steer_ + 2.0 * max_steer_ * ratio;

    // 실제 한 주기에 가능한 조향 변화량 제한
    steer = clamp(steer, last_steer_ - max_delta, last_steer_ + max_delta);
    steer = clamp(steer, -max_steer_, max_steer_);

    for (const double v : speed_candidates) {
      const double cost = evaluateCandidate(ego_state, local_path, steer, v);

      if (cost < best_cost) {
        best_cost = cost;
        best_steer = steer;
        best_speed = v;
      }
    }
  }

  if (!std::isfinite(best_cost)) {
    last_steer_ = 0.0;
    return cmd;
  }

  double accel_cmd = 2.1 * (best_speed - ego_state.speed);
  accel_cmd = clamp(accel_cmd, -max_accel_, max_accel_);

  last_steer_ = best_steer;

  cmd.steering = best_steer;
  cmd.acceleration = accel_cmd;
  cmd.target_speed = best_speed;
  cmd.valid = true;

  return cmd;
}

}  // namespace race_driver
