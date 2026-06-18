#pragma once

#include <string>
#include <vector>

#include "race_driver/types.hpp"

namespace race_driver
{

class BoundaryCorridorPlanner
{
public:
  void setParameters(
    double planning_horizon_m,
    double wall_margin_m,
    double obstacle_margin_m,
    double max_lateral_accel);

  LocalPath plan(
    const EgoState & ego_state,
    const TrackModel & track_model,
    const ObstacleList & obstacles) const;

private:
  struct CorridorPoint
  {
    double left_x{0.0};
    double left_y{0.0};
    double right_x{0.0};
    double right_y{0.0};
    double mid_x{0.0};
    double mid_y{0.0};
    double s{0.0};
    double yaw{0.0};
    double width{0.0};
  };

  struct Candidate
  {
    LocalPath path;
    double cost{0.0};
    std::string reason{"unset"};
  };

  static double normalizeAngle(double angle);
  static double clamp(double value, double min_value, double max_value);

  std::vector<CorridorPoint> buildCorridor(
    const TrackModel & track_model) const;

  int findNearestCorridorIndex(
    const EgoState & ego_state,
    const std::vector<CorridorPoint> & corridor) const;

  std::vector<CorridorPoint> cropHorizon(
    const std::vector<CorridorPoint> & corridor,
    int start_idx) const;

  Candidate makeConstantRatioCandidate(
    const std::vector<CorridorPoint> & segment,
    double ratio,
    const ObstacleList & obstacles) const;

  Candidate makeApexCandidate(
    const std::vector<CorridorPoint> & segment,
    bool late_apex,
    const ObstacleList & obstacles) const;

  void recomputeGeometry(LocalPath & path) const;

  double obstacleDistanceCost(
    const LocalPath & path,
    const ObstacleList & obstacles,
    bool & collision_free) const;

  double evaluateCost(
    const LocalPath & path,
    const ObstacleList & obstacles,
    bool & collision_free) const;

  double planning_horizon_m_{14.0};
  double wall_margin_m_{0.28};
  double obstacle_margin_m_{0.32};
  double max_lateral_accel_{9.0};
};

}  // namespace race_driver
