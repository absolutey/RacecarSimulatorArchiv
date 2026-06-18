#pragma once

#include <vector>

namespace race_driver
{

struct LapTimeEvaluation
{
  bool valid{false};
  double estimated_time_sec{0.0};
  double total_distance_m{0.0};
  double average_speed_mps{0.0};
};

class LapTimeEvaluator
{
public:
  LapTimeEvaluator() = default;

  LapTimeEvaluation evaluate(
    const std::vector<double> & segment_lengths,
    const std::vector<double> & speed_profile) const;
};

}  // namespace race_driver
