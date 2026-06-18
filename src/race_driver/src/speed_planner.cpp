#include "race_driver/speed_planner.hpp"

#include <algorithm>
#include <cmath>

namespace race_driver
{

void SpeedPlanner::setParameters(
  double max_speed,
  double min_speed,
  double curvature_speed_gain,
  double obstacle_slowdown_gain,
  double brake_distance,
  double stop_distance,
  double avoid_speed_scale,
  double narrow_corridor_speed_scale,
  double max_lateral_accel)
{
  max_speed_ = max_speed;
  min_speed_ = min_speed;
  curvature_speed_gain_ = curvature_speed_gain;
  obstacle_slowdown_gain_ = obstacle_slowdown_gain;
  brake_distance_ = brake_distance;
  stop_distance_ = stop_distance;
  avoid_speed_scale_ = avoid_speed_scale;
  narrow_corridor_speed_scale_ = narrow_corridor_speed_scale;
  max_lateral_accel_ = max_lateral_accel;
}

double SpeedPlanner::computeTargetSpeed(
  const EgoState & ego_state,
  const LocalPath & local_path,
  const ObstacleList & obstacles) const
{
  if (!local_path.valid || local_path.points.size() < 2) {
    return 0.0;
  }

  /*
   * v3-1 principle:
   * - 기본은 빠르게 간다.
   * - 곡률이 큰 구간에서만 필요한 만큼 감속한다.
   * - 회피 중이라는 이유만으로 과도하게 감속하지 않는다.
   */

  const double max_curvature = std::max(0.0, local_path.max_curvature);

  double curvature_limit_speed = max_speed_;

  if (max_curvature > 1e-5) {
    curvature_limit_speed = std::sqrt(
      std::max(0.1, max_lateral_accel_) / std::max(1e-5, max_curvature));

    curvature_limit_speed =
      curvature_limit_speed / (1.0 + 0.25 * curvature_speed_gain_ * max_curvature);

    curvature_limit_speed = std::clamp(
      curvature_limit_speed,
      min_speed_,
      max_speed_);
  }

  double target_speed = curvature_limit_speed;

  const bool avoiding =
    local_path.decision.reason == "avoid_left" ||
    local_path.decision.reason == "avoid_right" ||
    std::abs(local_path.decision.peak_shift) > 1e-3;

  if (avoiding) {
    const auto & obs = local_path.decision.blocking_obstacle;

    if (obs.valid && obs.blocks_center_corridor) {
      const double ego_s = local_path.points.front().s;
      const double obstacle_distance_along_path = std::max(0.0, obs.s - ego_s);

      if (obstacle_distance_along_path <= stop_distance_) {
        return 0.0;
      }

      if (obstacle_distance_along_path <= brake_distance_) {
        const double ratio = std::clamp(
          (obstacle_distance_along_path - stop_distance_) /
          std::max(0.1, brake_distance_ - stop_distance_),
          0.0,
          1.0);

        const double brake_speed =
          min_speed_ + ratio * (target_speed - min_speed_);

        target_speed = std::min(target_speed, brake_speed);
      } else {
        target_speed *= std::clamp(avoid_speed_scale_, 0.75, 1.0);
      }

      const double corridor_width = std::max(
        0.05,
        std::min(obs.left_free, obs.right_free));

      if (corridor_width < 0.35) {
        target_speed *= std::clamp(narrow_corridor_speed_scale_, 0.70, 1.0);
      }
    }
  }

  (void)ego_state;
  (void)obstacles;
  (void)obstacle_slowdown_gain_;

  return std::clamp(target_speed, 0.0, max_speed_);
}

}  // namespace race_driver
