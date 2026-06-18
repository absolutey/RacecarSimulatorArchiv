#include "race_driver/obstacle_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace race_driver
{

namespace
{
struct ScanPoint
{
  double x{0.0};
  double y{0.0};
};

double pointDistance(const ScanPoint & a, const ScanPoint & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

int nearestCenterIndex(const TrackModel & track_model, double x, double y)
{
  int best_idx = -1;
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < track_model.centerline.size(); ++i) {
    const double dx = track_model.centerline[i].x - x;
    const double dy = track_model.centerline[i].y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

Obstacle buildObstacle(
  const std::vector<ScanPoint> & cluster,
  const EgoState & ego_state,
  const TrackModel & track_model,
  double corridor_half_width,
  double obstacle_width_margin,
  double corridor_block_threshold,
  double ego_s,
  double obstacle_forward_horizon_m,
  double wall_length_threshold)
{
  Obstacle obs;
  if (cluster.empty() || track_model.centerline.empty()) {
    return obs;
  }

  double min_x = cluster.front().x;
  double max_x = cluster.front().x;
  double min_y = cluster.front().y;
  double max_y = cluster.front().y;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double polyline_length = 0.0;

  for (std::size_t i = 0; i < cluster.size(); ++i) {
    const auto & p = cluster[i];
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
    sum_x += p.x;
    sum_y += p.y;
    if (i > 0) {
      polyline_length += pointDistance(cluster[i - 1], cluster[i]);
    }
  }

  obs.x = sum_x / static_cast<double>(cluster.size());
  obs.y = sum_y / static_cast<double>(cluster.size());
  obs.world_x = ego_state.x + std::cos(ego_state.yaw) * obs.x - std::sin(ego_state.yaw) * obs.y;
  obs.world_y = ego_state.y + std::sin(ego_state.yaw) * obs.x + std::cos(ego_state.yaw) * obs.y;
  obs.lateral_offset = obs.y;
  obs.distance = std::sqrt(obs.x * obs.x + obs.y * obs.y);
  obs.width = std::max(0.05, max_y - min_y);
  obs.length = std::max(polyline_length, max_x - min_x);
  obs.point_count = static_cast<int>(cluster.size());

  const int center_idx = nearestCenterIndex(track_model, obs.world_x, obs.world_y);
  if (center_idx < 0) {
    return obs;
  }

  const auto & center_pt = track_model.centerline[static_cast<std::size_t>(center_idx)];
  const double nx = -std::sin(center_pt.yaw);
  const double ny = std::cos(center_pt.yaw);
  obs.s = center_pt.s;
  obs.d = (obs.world_x - center_pt.x) * nx + (obs.world_y - center_pt.y) * ny;
  obs.s_min = obs.s - 0.5 * std::max(obs.length, obs.width);
  obs.s_max = obs.s + 0.5 * std::max(obs.length, obs.width);

  const double occ_half = 0.5 * obs.width + obstacle_width_margin;
  obs.left_free = center_pt.left_clearance - std::max(0.0, obs.d + occ_half);
  obs.right_free = center_pt.right_clearance - std::max(0.0, -obs.d + occ_half);
  obs.passable_left = obs.left_free > corridor_block_threshold;
  obs.passable_right = obs.right_free > corridor_block_threshold;
  obs.blocks_center_corridor = std::abs(obs.d) <= (corridor_half_width + corridor_block_threshold);

  const bool ahead = (obs.s >= ego_s - 0.2) && ((obs.s - ego_s) <= obstacle_forward_horizon_m);
  if (!ahead) {
    return obs;
  }

  const double heading_local = normalizeAngle(std::atan2(max_y - min_y, max_x - min_x) + ego_state.yaw - center_pt.yaw);
  const bool wall_like = (obs.length > wall_length_threshold) &&
    (std::abs(std::cos(heading_local)) > 0.85) &&
    (std::abs(std::abs(obs.d) - std::max(center_pt.left_clearance, center_pt.right_clearance)) < 0.75);
  if (wall_like) {
    return obs;
  }

  const double forward_score = std::max(0.0, obstacle_forward_horizon_m - (obs.s - ego_s));
  const double center_block_score = obs.blocks_center_corridor ? 3.0 : 0.0;
  const double width_score = std::min(2.0, obs.width);
  obs.risk_score = center_block_score + width_score + 1.0 / std::max(0.5, obs.distance);
  obs.priority_score = forward_score + 2.0 * obs.risk_score + (obs.blocks_center_corridor ? 5.0 : 0.0);
  obs.valid = true;
  return obs;
}
}  // namespace

void ObstacleDetector::setParameters(
  double front_roi_min_x,
  double front_roi_max_x,
  double corridor_half_width,
  double cluster_distance_threshold,
  int min_cluster_points,
  double wall_length_threshold,
  double obstacle_width_margin,
  double corridor_block_threshold,
  double obstacle_forward_horizon_m)
{
  front_roi_min_x_ = front_roi_min_x;
  front_roi_max_x_ = front_roi_max_x;
  corridor_half_width_ = corridor_half_width;
  cluster_distance_threshold_ = cluster_distance_threshold;
  min_cluster_points_ = std::max(1, min_cluster_points);
  wall_length_threshold_ = wall_length_threshold;
  obstacle_width_margin_ = obstacle_width_margin;
  corridor_block_threshold_ = corridor_block_threshold;
  obstacle_forward_horizon_m_ = obstacle_forward_horizon_m;
}

ObstacleList ObstacleDetector::detect(
  const sensor_msgs::msg::LaserScan & scan,
  const EgoState & ego_state,
  const TrackModel & track_model) const
{
  ObstacleList out;
  out.valid = true;
  if (!ego_state.valid || !track_model.valid || track_model.centerline.empty()) {
    return out;
  }

  const int ego_center_idx = nearestCenterIndex(track_model, ego_state.x, ego_state.y);
  const double ego_s = ego_center_idx >= 0 ?
    track_model.centerline[static_cast<std::size_t>(ego_center_idx)].s : 0.0;

  std::vector<ScanPoint> points;
  points.reserve(scan.ranges.size());

  double angle = scan.angle_min;
  for (const auto & r : scan.ranges) {
    if (std::isfinite(r)) {
      const double x = r * std::cos(angle);
      const double y = r * std::sin(angle);
      if (x >= front_roi_min_x_ && x <= front_roi_max_x_ && std::abs(y) <= corridor_half_width_ + 2.0) {
        points.push_back({x, y});
      }
    }
    angle += scan.angle_increment;
  }

  if (points.empty()) {
    return out;
  }

  std::vector<ScanPoint> cluster;
  cluster.push_back(points.front());
  for (std::size_t i = 1; i < points.size(); ++i) {
    if (pointDistance(points[i - 1], points[i]) <= cluster_distance_threshold_) {
      cluster.push_back(points[i]);
    } else {
      if (static_cast<int>(cluster.size()) >= min_cluster_points_) {
        auto obs = buildObstacle(
          cluster, ego_state, track_model, corridor_half_width_, obstacle_width_margin_,
          corridor_block_threshold_, ego_s, obstacle_forward_horizon_m_, wall_length_threshold_);
        if (obs.valid) {
          out.obstacles.push_back(obs);
        }
      }
      cluster.clear();
      cluster.push_back(points[i]);
    }
  }
  if (static_cast<int>(cluster.size()) >= min_cluster_points_) {
    auto obs = buildObstacle(
      cluster, ego_state, track_model, corridor_half_width_, obstacle_width_margin_,
      corridor_block_threshold_, ego_s, obstacle_forward_horizon_m_, wall_length_threshold_);
    if (obs.valid) {
      out.obstacles.push_back(obs);
    }
  }

  std::sort(out.obstacles.begin(), out.obstacles.end(), [](const Obstacle & a, const Obstacle & b) {
    return a.priority_score > b.priority_score;
  });

  return out;
}

}  // namespace race_driver
