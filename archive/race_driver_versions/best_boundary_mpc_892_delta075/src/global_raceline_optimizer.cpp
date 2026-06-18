#include "race_driver/global_raceline_optimizer.hpp"

#include <algorithm>
#include <cmath>

namespace race_driver
{

void GlobalRacelineOptimizer::configure(
  double min_boundary_margin,
  double smoothing_gain)
{
  min_boundary_margin_ = std::max(0.0, min_boundary_margin);
  smoothing_gain_ = std::clamp(smoothing_gain, 0.0, 1.0);
}

GlobalRaceline GlobalRacelineOptimizer::buildCenterlineSeed(
  const std::vector<geometry_msgs::msg::Point> & center_points) const
{
  GlobalRaceline line;

  if (center_points.size() < 3) {
    return line;
  }

  line.points = center_points;
  line.segment_lengths.resize(line.points.size(), 0.0);
  line.curvature.resize(line.points.size(), 0.0);

  for (std::size_t i = 1; i < line.points.size(); ++i) {
    line.segment_lengths[i - 1] = distance2D(line.points[i - 1], line.points[i]);
  }

  line.segment_lengths.back() = line.segment_lengths.size() >= 2
    ? line.segment_lengths[line.segment_lengths.size() - 2]
    : 0.1;

  for (std::size_t i = 1; i + 1 < line.points.size(); ++i) {
    const double h1 = headingBetween(line.points[i - 1], line.points[i]);
    const double h2 = headingBetween(line.points[i], line.points[i + 1]);

    double dh = h2 - h1;
    while (dh > M_PI) {
      dh -= 2.0 * M_PI;
    }
    while (dh < -M_PI) {
      dh += 2.0 * M_PI;
    }

    const double ds = std::max(0.05, 0.5 * (line.segment_lengths[i - 1] + line.segment_lengths[i]));
    line.curvature[i] = dh / ds;
  }

  if (line.curvature.size() >= 2) {
    line.curvature.front() = line.curvature[1];
    line.curvature.back() = line.curvature[line.curvature.size() - 2];
  }

  line.valid = true;
  return line;
}

double GlobalRacelineOptimizer::distance2D(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

double GlobalRacelineOptimizer::headingBetween(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  return std::atan2(b.y - a.y, b.x - a.x);
}

}  // namespace race_driver
