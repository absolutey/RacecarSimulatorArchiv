#pragma once

#include <string>
#include <vector>

#include "race_driver/occupancy_grid_manager.hpp"
#include "race_driver/types.hpp"

namespace race_driver
{

class MapRolloutPlanner
{
public:
  void setParameters(
    double horizon_m,
    double step_m,
    double vehicle_radius_m,
    double max_steer,
    double wheelbase,
    double max_lateral_accel,
    int steer_samples);

  LocalPath plan(
    const EgoState & ego_state,
    const OccupancyGridManager & grid,
    const ObstacleList & obstacles) const;

private:
  struct Candidate
  {
    LocalPath path;
    double cost{0.0};
    double steer{0.0};
    bool valid{false};
    std::string reason{"unset"};
  };

  struct CorridorClearance
  {
    bool collision_free{false};
    double min_clearance{0.0};
    double avg_clearance{0.0};
    double avg_balance_error{0.0};
  };

  static double normalizeAngle(double angle);
  static double clamp(double value, double min_value, double max_value);

  Candidate rolloutCandidate(
    const EgoState & ego_state,
    const OccupancyGridManager & grid,
    const ObstacleList & obstacles,
    double steer) const;

  void recomputeGeometry(LocalPath & path) const;

  double raycastClearance(
    const OccupancyGridManager & grid,
    double x,
    double y,
    double nx,
    double ny) const;

  CorridorClearance evaluateMapCorridor(
    LocalPath & path,
    const OccupancyGridManager & grid) const;

  double obstacleCost(
    const LocalPath & path,
    const ObstacleList & obstacles,
    bool & collision_free) const;

  double evaluateCost(
    LocalPath & path,
    const OccupancyGridManager & grid,
    const ObstacleList & obstacles,
    const EgoState & ego_state,
    double steer,
    bool & collision_free) const;

  double horizon_m_{4.8};
  double step_m_{0.18};
  double vehicle_radius_m_{0.22};
  double max_steer_{0.28};
  double wheelbase_{0.33};
  double max_lateral_accel_{7.2};
  int steer_samples_{13};

  double clearance_probe_step_m_{0.05};
  double clearance_probe_max_m_{1.20};
  double min_required_clearance_m_{0.24};
};

}  // namespace race_driver
