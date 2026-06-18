#include "race_driver/map_rollout_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace race_driver
{

void MapRolloutPlanner::setParameters(
  double horizon_m,
  double step_m,
  double vehicle_radius_m,
  double max_steer,
  double wheelbase,
  double max_lateral_accel,
  int steer_samples)
{
  horizon_m_ = std::max(2.0, horizon_m);
  step_m_ = std::max(0.05, step_m);
  vehicle_radius_m_ = std::max(0.03, vehicle_radius_m);
  max_steer_ = std::max(0.05, max_steer);
  wheelbase_ = std::max(0.05, wheelbase);
  max_lateral_accel_ = std::max(0.5, max_lateral_accel);
  steer_samples_ = std::max(3, steer_samples);

  if (steer_samples_ % 2 == 0) {
    steer_samples_ += 1;
  }

  // Map-only planner는 boundary topic이 없기 때문에 map clearance를 더 보수적으로 본다.
  min_required_clearance_m_ = std::max(0.18, vehicle_radius_m_);
}

double MapRolloutPlanner::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double MapRolloutPlanner::clamp(
  double value,
  double min_value,
  double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

void MapRolloutPlanner::recomputeGeometry(LocalPath & path) const
{
  auto & pts = path.points;

  if (pts.size() < 2) {
    path.valid = false;
    return;
  }

  pts[0].s = 0.0;

  for (std::size_t i = 1; i < pts.size(); ++i) {
    const double dx = pts[i].x - pts[i - 1].x;
    const double dy = pts[i].y - pts[i - 1].y;
    pts[i].s = pts[i - 1].s + std::hypot(dx, dy);
  }

  for (std::size_t i = 0; i < pts.size(); ++i) {
    const std::size_t prev_i = i == 0 ? 0 : i - 1;
    const std::size_t next_i = std::min(pts.size() - 1, i + 1);

    const double dx = pts[next_i].x - pts[prev_i].x;
    const double dy = pts[next_i].y - pts[prev_i].y;

    pts[i].yaw = std::atan2(dy, dx);
    pts[i].curvature = 0.0;
  }

  std::vector<double> abs_curvatures;
  abs_curvatures.reserve(pts.size());

  for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
    const double yaw_prev = pts[i - 1].yaw;
    const double yaw_next = pts[i + 1].yaw;
    const double ds = std::max(1e-6, pts[i + 1].s - pts[i - 1].s);

    pts[i].curvature = normalizeAngle(yaw_next - yaw_prev) / ds;
    abs_curvatures.push_back(std::abs(pts[i].curvature));
  }

  if (pts.size() >= 3) {
    pts.front().curvature = pts[1].curvature;
    pts.back().curvature = pts[pts.size() - 2].curvature;
  }

  if (abs_curvatures.empty()) {
    path.max_curvature = 0.0;
  } else {
    std::sort(abs_curvatures.begin(), abs_curvatures.end());

    const std::size_t idx = std::min(
      abs_curvatures.size() - 1,
      static_cast<std::size_t>(0.85 * static_cast<double>(abs_curvatures.size() - 1)));

    path.max_curvature = abs_curvatures[idx];
  }

  path.valid = true;
}

double MapRolloutPlanner::raycastClearance(
  const OccupancyGridManager & grid,
  double x,
  double y,
  double nx,
  double ny) const
{
  double last_safe = 0.0;

  for (double r = clearance_probe_step_m_; r <= clearance_probe_max_m_; r += clearance_probe_step_m_) {
    const double px = x + nx * r;
    const double py = y + ny * r;

    if (!grid.isWorldPointSafe(px, py, vehicle_radius_m_)) {
      break;
    }

    last_safe = r;
  }

  return last_safe;
}

MapRolloutPlanner::CorridorClearance MapRolloutPlanner::evaluateMapCorridor(
  LocalPath & path,
  const OccupancyGridManager & grid) const
{
  CorridorClearance result;

  if (!path.valid || path.points.empty() || !grid.valid()) {
    return result;
  }

  double min_clearance = std::numeric_limits<double>::max();
  double sum_clearance = 0.0;
  double sum_balance = 0.0;
  int count = 0;

  for (auto & p : path.points) {
    if (!grid.isWorldPointSafe(p.x, p.y, vehicle_radius_m_)) {
      result.collision_free = false;
      result.min_clearance = 0.0;
      result.avg_clearance = 0.0;
      result.avg_balance_error = 1.0;
      return result;
    }

    const double nx_left = -std::sin(p.yaw);
    const double ny_left = std::cos(p.yaw);

    const double left_clearance =
      raycastClearance(grid, p.x, p.y, nx_left, ny_left);

    const double right_clearance =
      raycastClearance(grid, p.x, p.y, -nx_left, -ny_left);

    p.left_clearance = left_clearance;
    p.right_clearance = right_clearance;

    const double side_min = std::min(left_clearance, right_clearance);
    const double side_sum = std::max(1e-6, left_clearance + right_clearance);
    const double balance_error = std::abs(left_clearance - right_clearance) / side_sum;

    min_clearance = std::min(min_clearance, side_min);
    sum_clearance += side_min;
    sum_balance += balance_error;
    ++count;
  }

  if (count <= 0) {
    return result;
  }

  result.collision_free = min_clearance >= min_required_clearance_m_;
  result.min_clearance = min_clearance;
  result.avg_clearance = sum_clearance / static_cast<double>(count);
  result.avg_balance_error = sum_balance / static_cast<double>(count);

  return result;
}

double MapRolloutPlanner::obstacleCost(
  const LocalPath & path,
  const ObstacleList & obstacles,
  bool & collision_free) const
{
  collision_free = true;

  if (!obstacles.valid || obstacles.obstacles.empty()) {
    return 0.0;
  }

  double min_dist = std::numeric_limits<double>::max();

  for (const auto & obs : obstacles.obstacles) {
    if (!obs.valid || obs.wall_like) {
      continue;
    }

    for (const auto & p : path.points) {
      const double dx = p.x - obs.world_x;
      const double dy = p.y - obs.world_y;
      const double d = std::hypot(dx, dy);

      min_dist = std::min(min_dist, d);

      if (d < vehicle_radius_m_ + 0.18) {
        collision_free = false;
      }
    }
  }

  if (!std::isfinite(min_dist) || min_dist == std::numeric_limits<double>::max()) {
    return 0.0;
  }

  return 3.0 / std::max(0.12, min_dist);
}

double MapRolloutPlanner::evaluateCost(
  LocalPath & path,
  const OccupancyGridManager & grid,
  const ObstacleList & obstacles,
  const EgoState & ego_state,
  double steer,
  bool & collision_free) const
{
  collision_free = true;

  if (!path.valid || path.points.size() < 2) {
    collision_free = false;
    return std::numeric_limits<double>::infinity();
  }

  CorridorClearance corridor = evaluateMapCorridor(path, grid);

  if (!corridor.collision_free) {
    collision_free = false;
    return std::numeric_limits<double>::infinity();
  }

  bool obstacle_clear = true;
  const double obs_cost = obstacleCost(path, obstacles, obstacle_clear);

  if (!obstacle_clear) {
    collision_free = false;
    return std::numeric_limits<double>::infinity();
  }

  const double path_length = path.points.back().s;
  const double effective_curvature = std::max(0.016, path.max_curvature);
  const double feasible_speed = std::sqrt(max_lateral_accel_ / effective_curvature);
  const double expected_time = path_length / std::max(0.5, feasible_speed);

  const double heading_x = std::cos(ego_state.yaw);
  const double heading_y = std::sin(ego_state.yaw);

  const auto & first = path.points.front();
  const auto & last = path.points.back();

  const double dx = last.x - first.x;
  const double dy = last.y - first.y;
  const double forward_progress = dx * heading_x + dy * heading_y;

  if (forward_progress < 1.15) {
    collision_free = false;
    return std::numeric_limits<double>::infinity();
  }

  const double yaw_change = std::abs(normalizeAngle(last.yaw - ego_state.yaw));

  const double clearance_cost = 1.35 / std::max(0.05, corridor.min_clearance);
  const double avg_clearance_cost = 0.45 / std::max(0.05, corridor.avg_clearance);
  const double balance_cost = 0.80 * corridor.avg_balance_error;
  const double curvature_cost = 1.25 * path.max_curvature;
  const double steer_cost = 0.50 * std::abs(steer);
  const double yaw_change_cost = 0.25 * yaw_change;

  return
    2.55 * expected_time +
    clearance_cost +
    avg_clearance_cost +
    balance_cost +
    curvature_cost +
    steer_cost +
    yaw_change_cost +
    obs_cost -
    0.18 * forward_progress;
}

MapRolloutPlanner::Candidate MapRolloutPlanner::rolloutCandidate(
  const EgoState & ego_state,
  const OccupancyGridManager & grid,
  const ObstacleList & obstacles,
  double steer) const
{
  Candidate candidate;
  candidate.steer = steer;
  candidate.reason = "map_corridor_rollout";

  if (!ego_state.valid || !grid.valid()) {
    candidate.valid = false;
    candidate.cost = std::numeric_limits<double>::infinity();
    return candidate;
  }

  double x = ego_state.x;
  double y = ego_state.y;
  double yaw = ego_state.yaw;

  candidate.path.points.reserve(
    static_cast<std::size_t>(std::ceil(horizon_m_ / step_m_)) + 1);

  const double curvature = std::tan(steer) / wheelbase_;

  for (double s = 0.0; s <= horizon_m_; s += step_m_) {
    LocalPathPoint p;
    p.x = x;
    p.y = y;
    p.yaw = yaw;
    p.s = s;
    p.curvature = curvature;
    p.left_clearance = 0.0;
    p.right_clearance = 0.0;
    p.d = steer;

    if (!grid.isWorldPointSafe(p.x, p.y, vehicle_radius_m_)) {
      candidate.valid = false;
      candidate.cost = std::numeric_limits<double>::infinity();
      candidate.reason = "map_corridor_collision";
      return candidate;
    }

    candidate.path.points.push_back(p);

    x += step_m_ * std::cos(yaw);
    y += step_m_ * std::sin(yaw);
    yaw = normalizeAngle(yaw + step_m_ * curvature);
  }

  recomputeGeometry(candidate.path);

  bool collision_free = true;
  candidate.cost = evaluateCost(
    candidate.path,
    grid,
    obstacles,
    ego_state,
    steer,
    collision_free);

  candidate.valid = candidate.path.valid && collision_free;
  candidate.path.valid = candidate.valid;

  candidate.path.decision.valid = candidate.valid;
  candidate.path.decision.reason = candidate.reason;
  candidate.path.decision.peak_shift = steer;
  candidate.path.decision.shift_direction = steer >= 0.0 ? 1.0 : -1.0;
  candidate.path.decision.chosen_left = steer >= 0.0;

  return candidate;
}

LocalPath MapRolloutPlanner::plan(
  const EgoState & ego_state,
  const OccupancyGridManager & grid,
  const ObstacleList & obstacles) const
{
  LocalPath empty;

  if (!ego_state.valid || !grid.valid()) {
    empty.valid = false;
    empty.decision.valid = false;
    empty.decision.reason = "map corridor rollout invalid input";
    return empty;
  }

  Candidate best;
  best.valid = false;
  best.cost = std::numeric_limits<double>::infinity();

  const int half = steer_samples_ / 2;

  for (int i = -half; i <= half; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(half);
    const double steer = ratio * max_steer_;

    Candidate candidate = rolloutCandidate(
      ego_state,
      grid,
      obstacles,
      steer);

    if (!candidate.valid) {
      continue;
    }

    if (candidate.cost < best.cost) {
      best = candidate;
    }
  }

  if (!best.valid) {
    empty.valid = false;
    empty.decision.valid = false;
    empty.decision.reason = "map corridor rollout no valid candidate";
    return empty;
  }

  best.path.decision.valid = true;
  best.path.decision.reason =
    "map_corridor_rollout_steer_" + std::to_string(best.steer);

  return best.path;
}

}  // namespace race_driver
