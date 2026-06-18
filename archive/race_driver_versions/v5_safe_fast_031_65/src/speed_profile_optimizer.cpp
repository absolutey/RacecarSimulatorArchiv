#include "race_driver/speed_profile_optimizer.hpp"

#include <algorithm>
#include <cmath>

namespace race_driver
{

void SpeedProfileOptimizer::configure(
  double min_speed,
  double max_speed,
  double max_lateral_accel,
  double max_accel,
  double max_decel)
{
  min_speed_ = std::max(0.05, min_speed);
  max_speed_ = std::max(min_speed_, max_speed);
  max_lateral_accel_ = std::max(0.1, max_lateral_accel);
  max_accel_ = std::max(0.1, max_accel);
  max_decel_ = std::max(0.1, max_decel);
}

std::vector<double> SpeedProfileOptimizer::computeSpeedProfile(
  const std::vector<double> & curvature,
  const std::vector<double> & segment_lengths) const
{
  const std::size_t n = curvature.size();
  std::vector<double> speed(n, max_speed_);

  if (n == 0) {
    return speed;
  }

  // 1) curvature limit: v <= sqrt(a_lat / kappa)
  for (std::size_t i = 0; i < n; ++i) {
    const double kappa = std::abs(curvature[i]);

    if (kappa > 1e-4) {
      const double v_curve = std::sqrt(max_lateral_accel_ / kappa);
      speed[i] = std::clamp(v_curve, min_speed_, max_speed_);
    } else {
      speed[i] = max_speed_;
    }
  }

  // 2) forward pass: acceleration limit
  for (std::size_t i = 1; i < n; ++i) {
    const double ds = segment_lengths.empty()
      ? 0.1
      : std::max(0.01, segment_lengths[std::min(i - 1, segment_lengths.size() - 1)]);

    const double reachable = std::sqrt(std::max(0.0, speed[i - 1] * speed[i - 1] + 2.0 * max_accel_ * ds));
    speed[i] = std::min(speed[i], reachable);
  }

  // 3) backward pass: deceleration limit
  for (std::size_t idx = n - 1; idx > 0; --idx) {
    const std::size_t i = idx - 1;

    const double ds = segment_lengths.empty()
      ? 0.1
      : std::max(0.01, segment_lengths[std::min(i, segment_lengths.size() - 1)]);

    const double reachable = std::sqrt(std::max(0.0, speed[i + 1] * speed[i + 1] + 2.0 * max_decel_ * ds));
    speed[i] = std::min(speed[i], reachable);
  }

  for (double & v : speed) {
    v = std::clamp(v, min_speed_, max_speed_);
  }

  return speed;
}

}  // namespace race_driver
