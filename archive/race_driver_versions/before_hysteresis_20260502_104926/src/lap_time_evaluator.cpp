#include "race_driver/lap_time_evaluator.hpp"

#include <algorithm>
#include <cmath>

namespace race_driver
{

LapTimeEvaluation LapTimeEvaluator::evaluate(
  const std::vector<double> & segment_lengths,
  const std::vector<double> & speed_profile) const
{
  LapTimeEvaluation result;

  if (segment_lengths.empty() || speed_profile.empty()) {
    return result;
  }

  const std::size_t n = std::min(segment_lengths.size(), speed_profile.size());

  double total_time = 0.0;
  double total_distance = 0.0;

  for (std::size_t i = 0; i < n; ++i) {
    const double ds = std::max(0.0, segment_lengths[i]);
    const double v = std::max(0.05, speed_profile[i]);

    total_distance += ds;
    total_time += ds / v;
  }

  if (total_time <= 0.0 || total_distance <= 0.0) {
    return result;
  }

  result.valid = true;
  result.estimated_time_sec = total_time;
  result.total_distance_m = total_distance;
  result.average_speed_mps = total_distance / total_time;

  return result;
}

}  // namespace race_driver
