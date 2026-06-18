#include "race_driver/occupancy_grid_manager.hpp"

#include <algorithm>
#include <cmath>

namespace race_driver
{

void OccupancyGridManager::updateMap(const nav_msgs::msg::OccupancyGrid & msg)
{
  origin_x_ = msg.info.origin.position.x;
  origin_y_ = msg.info.origin.position.y;
  resolution_ = msg.info.resolution;

  width_ = static_cast<int>(msg.info.width);
  height_ = static_cast<int>(msg.info.height);

  data_ = msg.data;

  valid_ =
    resolution_ > 1e-6 &&
    width_ > 0 &&
    height_ > 0 &&
    static_cast<int>(data_.size()) == width_ * height_;
}

bool OccupancyGridManager::valid() const
{
  return valid_;
}

bool OccupancyGridManager::inBounds(int gx, int gy) const
{
  return gx >= 0 && gy >= 0 && gx < width_ && gy < height_;
}

int OccupancyGridManager::index(int gx, int gy) const
{
  return gy * width_ + gx;
}

bool OccupancyGridManager::worldToGrid(
  double x,
  double y,
  int & gx,
  int & gy) const
{
  if (!valid_) {
    return false;
  }

  gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));

  return inBounds(gx, gy);
}

bool OccupancyGridManager::isCellOccupied(int gx, int gy) const
{
  if (!valid_ || !inBounds(gx, gy)) {
    return true;
  }

  const int8_t value = data_[index(gx, gy)];

  // OccupancyGrid convention:
  // -1 unknown, 0 free, 100 occupied.
  // For racing safety, unknown is treated as occupied.
  if (value < 0) {
    return true;
  }

  return value >= 50;
}

bool OccupancyGridManager::isWorldPointSafe(
  double x,
  double y,
  double radius_m) const
{
  if (!valid_) {
    return true;
  }

  int cx = 0;
  int cy = 0;

  if (!worldToGrid(x, y, cx, cy)) {
    return false;
  }

  const int radius_cells = std::max(
    1,
    static_cast<int>(std::ceil(radius_m / resolution_)));

  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double dist_cells = std::hypot(
        static_cast<double>(dx),
        static_cast<double>(dy));

      if (dist_cells > static_cast<double>(radius_cells)) {
        continue;
      }

      const int gx = cx + dx;
      const int gy = cy + dy;

      if (isCellOccupied(gx, gy)) {
        return false;
      }
    }
  }

  return true;
}

bool OccupancyGridManager::isPathSafe(
  const LocalPath & path,
  double vehicle_radius_m) const
{
  if (!valid_) {
    return true;
  }

  if (!path.valid || path.points.empty()) {
    return false;
  }

  for (const auto & p : path.points) {
    if (!isWorldPointSafe(p.x, p.y, vehicle_radius_m)) {
      return false;
    }
  }

  return true;
}

}  // namespace race_driver
