#pragma once

#include <vector>

#include "race_driver/types.hpp"

namespace race_driver
{

class CandidateGenerator
{
public:
  void setParameters(
    double planning_horizon_m,
    double obstacle_margin,
    double shift_pre_margin_m,
    double shift_post_margin_m,
    double max_peak_shift_m,
    double return_blend_length_m);

  std::vector<PathCandidate> generate(
    const EgoState & ego_state,
    const TrackModel & track_model,
    const ObstacleList & obstacles) const;

private:
  static std::size_t findNearestIndex(
    const EgoState & ego_state,
    const std::vector<TrackPoint> & centerline);

  std::vector<LocalPathPoint> buildBasePath(
    const EgoState & ego_state,
    const TrackModel & track_model,
    double & start_s) const;

  const Obstacle * selectPrimaryObstacle(
    double start_s,
    const ObstacleList & obstacles) const;

  PathCandidate makeCandidate(
    CandidateSide side,
    double peak_shift,
    const std::vector<LocalPathPoint> & base_path,
    const Obstacle * obstacle) const;

  PathCandidate makeApexCandidate(
    const std::vector<LocalPathPoint> & base_path) const;

  static void recomputeCandidateGeometry(PathCandidate & candidate);

  double planning_horizon_m_{8.0};
  double obstacle_margin_{0.35};
  double shift_pre_margin_m_{1.8};
  double shift_post_margin_m_{2.4};
  double max_peak_shift_m_{1.0};
  double return_blend_length_m_{0.8};
};

}  // namespace race_driver
