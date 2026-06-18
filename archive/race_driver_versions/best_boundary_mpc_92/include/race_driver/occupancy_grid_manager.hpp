#pragma once

#include <cstdint>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "race_driver/types.hpp"

namespace race_driver
{

class OccupancyGridManager
{
public:
  void updateMap(const nav_msgs::msg::OccupancyGrid & msg);

  bool valid() const;

  bool worldToGrid(
    double x,
    double y,
    int & gx,
    int & gy) const;

  bool isCellOccupied(
    int gx,
    int gy) const;

  bool isWorldPointSafe(
    double x,
    double y,
    double radius_m) const;

  bool isPathSafe(
    const LocalPath & path,
    double vehicle_radius_m) const;

private:
  bool inBounds(int gx, int gy) const;
  int index(int gx, int gy) const;

  double origin_x_{0.0};
  double origin_y_{0.0};
  double resolution_{0.05};

  int width_{0};
  int height_{0};

  std::vector<int8_t> data_;

  bool valid_{false};
};

}  // namespace race_driver
