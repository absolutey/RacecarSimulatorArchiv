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
  double normalizeAngle(double angle) const;
  double clamp(double value, double min_value, double max_value) const;

  bool findTrackingTarget(
    const EgoState & ego_state,
    const LocalPath & local_path,
    double lookahead,
    LocalPathPoint & target) const;

  bool findNearestPathPoint(
    const EgoState & ego_state,
    const LocalPath & local_path,
    LocalPathPoint & nearest,
    double & signed_cross_track_error,
    double & heading_error) const;

  double computePurePursuitSteer(
    const EgoState & ego_state,
    const LocalPathPoint & target,
    double lookahead) const;

  double computeStanleySteer(
    double signed_cross_track_error,
    double heading_error,
    double speed) const;

  double lookahead_distance_{2.0};
  double speed_kp_{1.7};
  double max_steer_{0.40};
  double max_accel_{2.8};
  double wheelbase_{0.33};
  double steer_rate_limit_{3.2};
  double control_dt_{0.025};
  double low_speed_deadband_{0.15};

  double last_steer_{0.0};

  double stanley_gain_{0.45};
  double stanley_softening_speed_{2.0};
  double min_stanley_weight_{0.00};
  double max_stanley_weight_{0.25};
};

}  // namespace race_driver
