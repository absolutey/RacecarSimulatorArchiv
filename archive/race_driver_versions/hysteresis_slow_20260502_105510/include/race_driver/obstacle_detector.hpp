#pragma once

#include "sensor_msgs/msg/laser_scan.hpp"
#include "race_driver/types.hpp"

namespace race_driver
{

class ObstacleDetector
{
public:
  void setParameters(
    double front_roi_min_x,
    double front_roi_max_x,
    double corridor_half_width,
    double cluster_distance_threshold,
    int min_cluster_points,
    double wall_length_threshold,
    double obstacle_width_margin,
    double corridor_block_threshold,
    double obstacle_forward_horizon_m);

  ObstacleList detect(
    const sensor_msgs::msg::LaserScan & scan,
    const EgoState & ego_state,
    const TrackModel & track_model) const;

private:
  double front_roi_min_x_{0.0};
  double front_roi_max_x_{6.0};
  double corridor_half_width_{1.5};
  double cluster_distance_threshold_{0.30};
  int min_cluster_points_{3};
  double wall_length_threshold_{1.0};
  double obstacle_width_margin_{0.25};
  double corridor_block_threshold_{0.15};
  double obstacle_forward_horizon_m_{10.0};
};

}  // namespace race_driver
