#pragma once

#include "race_driver/types.hpp"

namespace race_driver
{

class MpcLiteController
{
public:
  MpcLiteController() = default;

  void setParameters(
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
    double clearance_margin);

  ControlCommand compute(
    const EgoState & ego_state,
    const LocalPath & local_path,
    double target_speed);

private:
  struct NearestPathInfo
  {
    bool valid{false};
    LocalPathPoint point;
    double distance{0.0};
    double heading_error{0.0};
    double clearance{999.0};
  };

  struct SimState
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double speed{0.0};
  };

  double normalizeAngle(double angle) const;
  double clamp(double value, double min_value, double max_value) const;

  NearestPathInfo findNearestPathPoint(
    double x,
    double y,
    double yaw,
    const LocalPath & local_path) const;

  double evaluateCandidate(
    const EgoState & ego_state,
    const LocalPath & local_path,
    double steer,
    double candidate_speed) const;

  int horizon_steps_{12};
  double dt_{0.05};
  double wheelbase_{0.33};
  double max_steer_{0.30};
  double max_accel_{3.7};
  double steer_rate_limit_{3.8};
  int steer_samples_{11};
  double speed_delta_{0.25};

  double path_weight_{2.4};
  double heading_weight_{1.2};
  double steer_weight_{0.18};
  double steer_rate_weight_{0.35};
  double speed_reward_weight_{0.22};
  double clearance_weight_{1.0};
  double clearance_margin_{0.31};

  double last_steer_{0.0};
};

}  // namespace race_driver
