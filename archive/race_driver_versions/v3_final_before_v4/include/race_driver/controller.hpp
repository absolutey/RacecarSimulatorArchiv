#pragma once

#include "race_driver/types.hpp"

namespace race_driver
{

class Controller
{
public:
  void setParameters(
    double lookahead_distance,
    double speed_kp,
    double max_steer,
    double max_accel,
    double wheelbase,
    double steer_rate_limit,
    double control_dt,
    double low_speed_deadband);

  ControlCommand compute(
    const EgoState & ego_state,
    const LocalPath & local_path,
    double target_speed);

private:
  double lookahead_distance_{1.5};
  double speed_kp_{1.0};
  double max_steer_{0.4};
  double max_accel_{2.0};
  double wheelbase_{0.33};
  double steer_rate_limit_{1.5};
  double control_dt_{0.05};
  double low_speed_deadband_{0.15};
  double prev_steering_{0.0};
};

}  // namespace race_driver
