#pragma once

#include <vector>

#include "race_driver/types.hpp"

namespace race_driver
{

class CostEvaluator
{
public:
  void setWeights(
    double collision_weight,
    double boundary_weight,
    double curvature_weight,
    double deviation_weight,
    double smoothness_weight,
    double obstacle_weight);

  std::vector<PathCandidate> evaluate(
    const std::vector<PathCandidate> & candidates,
    const ObstacleList & obstacles) const;

  PathCandidate selectBest(
    const std::vector<PathCandidate> & candidates) const;

private:
  CostBreakdown evaluateOne(
    PathCandidate & candidate,
    const ObstacleList & obstacles) const;

  double collision_weight_{1000000.0};
  double boundary_weight_{50.0};
  double curvature_weight_{8.0};
  double deviation_weight_{1.2};
  double smoothness_weight_{3.0};
  double obstacle_weight_{5.0};
};

}  // namespace race_driver
