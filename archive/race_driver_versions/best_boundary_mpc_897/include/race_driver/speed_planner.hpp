#pragma once

#include "race_driver/types.hpp"

namespace race_driver
{

class SpeedPlanner
{
public:
  void setParameters(
    double max_speed,
    double min_speed,
    double curvature_speed_gain,
    double obstacle_slowdown_gain,
    double brake_distance,
    double stop_distance,
    double avoid_speed_scale,
    double narrow_corridor_speed_scale,
    double max_lateral_accel);

  double computeTargetSpeed(
    const EgoState & ego_state,
    const LocalPath & local_path,
    const ObstacleList & obstacles) const;

private:
  double max_speed_{5.0};
  double min_speed_{1.0};
  double curvature_speed_gain_{2.0};
  double obstacle_slowdown_gain_{1.5};
  double brake_distance_{2.5};
  double stop_distance_{1.0};
  double avoid_speed_scale_{0.8};
  double narrow_corridor_speed_scale_{0.7};
  double max_lateral_accel_{2.5};
};

}  // namespace race_driver
