#pragma once

#include <vector>

namespace race_driver
{

class SpeedProfileOptimizer
{
public:
  SpeedProfileOptimizer() = default;

  void configure(
    double min_speed,
    double max_speed,
    double max_lateral_accel,
    double max_accel,
    double max_decel);

  std::vector<double> computeSpeedProfile(
    const std::vector<double> & curvature,
    const std::vector<double> & segment_lengths) const;

private:
  double min_speed_{1.0};
  double max_speed_{4.5};
  double max_lateral_accel_{8.0};
  double max_accel_{3.0};
  double max_decel_{5.0};
};

}  // namespace race_driver
