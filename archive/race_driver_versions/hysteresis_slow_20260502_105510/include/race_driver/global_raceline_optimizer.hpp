#pragma once

#include <vector>

#include <geometry_msgs/msg/point.hpp>

namespace race_driver
{

struct GlobalRaceline
{
  bool valid{false};
  std::vector<geometry_msgs::msg::Point> points;
  std::vector<double> curvature;
  std::vector<double> segment_lengths;
  std::vector<double> speed_profile;
  double estimated_lap_time_sec{0.0};
};

class GlobalRacelineOptimizer
{
public:
  GlobalRacelineOptimizer() = default;

  void configure(
    double min_boundary_margin,
    double smoothing_gain);

  GlobalRaceline buildCenterlineSeed(
    const std::vector<geometry_msgs::msg::Point> & center_points) const;

private:
  double min_boundary_margin_{0.22};
  double smoothing_gain_{0.35};

  static double distance2D(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b);

  static double headingBetween(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b);
};

}  // namespace race_driver
