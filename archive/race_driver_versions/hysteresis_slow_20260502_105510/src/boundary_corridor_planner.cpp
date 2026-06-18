#include "race_driver/boundary_corridor_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace race_driver
{

void BoundaryCorridorPlanner::setParameters(
  double planning_horizon_m,
  double wall_margin_m,
  double obstacle_margin_m,
  double max_lateral_accel)
{
  planning_horizon_m_ = std::max(2.0, planning_horizon_m);
  wall_margin_m_ = std::max(0.05, wall_margin_m);
  obstacle_margin_m_ = std::max(0.05, obstacle_margin_m);
  max_lateral_accel_ = std::max(0.5, max_lateral_accel);
}

double BoundaryCorridorPlanner::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double BoundaryCorridorPlanner::clamp(
  double value,
  double min_value,
  double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

std::vector<BoundaryCorridorPlanner::CorridorPoint>
BoundaryCorridorPlanner::buildCorridor(const TrackModel & track_model) const
{
  std::vector<CorridorPoint> corridor;

  const std::size_t n = std::min(
    track_model.left_boundary.size(),
    track_model.right_boundary.size());

  if (n < 4) {
    return corridor;
  }

  corridor.reserve(n);

  for (std::size_t i = 0; i < n; ++i) {
    const auto & left = track_model.left_boundary[i];
    const auto & right = track_model.right_boundary[i];

    CorridorPoint c;
    c.left_x = left.x;
    c.left_y = left.y;
    c.right_x = right.x;
    c.right_y = right.y;

    c.mid_x = 0.5 * (left.x + right.x);
    c.mid_y = 0.5 * (left.y + right.y);

    const double wx = left.x - right.x;
    const double wy = left.y - right.y;
    c.width = std::hypot(wx, wy);

    if (c.width > 0.2 && c.width < 8.0) {
      corridor.push_back(c);
    }
  }

  if (corridor.size() < 4) {
    corridor.clear();
    return corridor;
  }

  corridor[0].s = 0.0;

  for (std::size_t i = 1; i < corridor.size(); ++i) {
    const double dx = corridor[i].mid_x - corridor[i - 1].mid_x;
    const double dy = corridor[i].mid_y - corridor[i - 1].mid_y;
    corridor[i].s = corridor[i - 1].s + std::hypot(dx, dy);
  }

  for (std::size_t i = 0; i < corridor.size(); ++i) {
    const std::size_t prev_i = i == 0 ? 0 : i - 1;
    const std::size_t next_i = std::min(corridor.size() - 1, i + 1);

    const double dx = corridor[next_i].mid_x - corridor[prev_i].mid_x;
    const double dy = corridor[next_i].mid_y - corridor[prev_i].mid_y;
    corridor[i].yaw = std::atan2(dy, dx);
  }

  return corridor;
}

int BoundaryCorridorPlanner::findNearestCorridorIndex(
  const EgoState & ego_state,
  const std::vector<CorridorPoint> & corridor) const
{
  if (corridor.empty()) {
    return -1;
  }

  int best_idx = 0;
  double best_dist2 = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < corridor.size(); ++i) {
    const double dx = corridor[i].mid_x - ego_state.x;
    const double dy = corridor[i].mid_y - ego_state.y;
    const double dist2 = dx * dx + dy * dy;

    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      best_idx = static_cast<int>(i);
    }
  }

  return best_idx;
}

std::vector<BoundaryCorridorPlanner::CorridorPoint>
BoundaryCorridorPlanner::cropHorizon(
  const std::vector<CorridorPoint> & corridor,
  int start_idx) const
{
  std::vector<CorridorPoint> segment;

  if (start_idx < 0 || static_cast<std::size_t>(start_idx) >= corridor.size()) {
    return segment;
  }

  if (corridor.size() < 4) {
    return segment;
  }

  const std::size_t n = corridor.size();
  const std::size_t start = static_cast<std::size_t>(start_idx);

  segment.reserve(n);

  double accumulated = 0.0;

  CorridorPoint prev = corridor[start];
  prev.s = 0.0;
  segment.push_back(prev);

  for (std::size_t step = 1; step < n; ++step) {
    const std::size_t idx = (start + step) % n;
    CorridorPoint cur = corridor[idx];

    const double dx = cur.mid_x - prev.mid_x;
    const double dy = cur.mid_y - prev.mid_y;

    accumulated += std::hypot(dx, dy);

    cur.s = accumulated;
    segment.push_back(cur);

    prev = cur;

    if (accumulated >= planning_horizon_m_) {
      break;
    }
  }

  return segment;
}

void BoundaryCorridorPlanner::recomputeGeometry(LocalPath & path) const
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

BoundaryCorridorPlanner::Candidate
BoundaryCorridorPlanner::makeConstantRatioCandidate(
  const std::vector<CorridorPoint> & segment,
  double ratio,
  const ObstacleList & obstacles) const
{
  Candidate candidate;
  candidate.reason = "boundary_ratio_" + std::to_string(static_cast<int>(ratio * 100.0));

  if (segment.size() < 3) {
    candidate.path.valid = false;
    candidate.cost = std::numeric_limits<double>::infinity();
    return candidate;
  }

  ratio = clamp(ratio, 0.05, 0.95);

  candidate.path.points.reserve(segment.size());

  for (const auto & c : segment) {
    LocalPathPoint p;

    p.x = c.right_x + ratio * (c.left_x - c.right_x);
    p.y = c.right_y + ratio * (c.left_y - c.right_y);
    p.d = ratio - 0.5;

    p.right_clearance = ratio * c.width;
    p.left_clearance = (1.0 - ratio) * c.width;

    candidate.path.points.push_back(p);
  }

  recomputeGeometry(candidate.path);

  bool collision_free = true;
  candidate.cost = evaluateCost(candidate.path, obstacles, collision_free);
  candidate.path.valid = candidate.path.valid && collision_free;

  candidate.path.decision.valid = candidate.path.valid;
  candidate.path.decision.reason = candidate.reason;
  candidate.path.decision.peak_shift = ratio - 0.5;
  candidate.path.decision.shift_direction = ratio >= 0.5 ? 1.0 : -1.0;
  candidate.path.decision.chosen_left = ratio >= 0.5;

  return candidate;
}

BoundaryCorridorPlanner::Candidate
BoundaryCorridorPlanner::makeApexCandidate(
  const std::vector<CorridorPoint> & segment,
  bool late_apex,
  const ObstacleList & obstacles) const
{
  Candidate candidate;
  candidate.reason = late_apex ? "boundary_late_apex" : "boundary_apex";

  if (segment.size() < 5) {
    candidate.path.valid = false;
    candidate.cost = std::numeric_limits<double>::infinity();
    return candidate;
  }

  double weighted_curvature = 0.0;
  double weight_sum = 0.0;

  for (std::size_t i = 1; i + 1 < segment.size(); ++i) {
    const double yaw_prev = segment[i - 1].yaw;
    const double yaw_next = segment[i + 1].yaw;
    const double ds = std::max(1e-6, segment[i + 1].s - segment[i - 1].s);
    const double k = normalizeAngle(yaw_next - yaw_prev) / ds;

    const double t = static_cast<double>(i) / static_cast<double>(segment.size() - 1);
    const double w = std::sin(M_PI * t);

    weighted_curvature += k * w;
    weight_sum += w;
  }

  if (weight_sum <= 1e-6) {
    candidate.path.valid = false;
    candidate.cost = std::numeric_limits<double>::infinity();
    return candidate;
  }

  const double avg_k = weighted_curvature / weight_sum;

  if (std::abs(avg_k) < 0.015) {
    candidate.path.valid = false;
    candidate.cost = std::numeric_limits<double>::infinity();
    return candidate;
  }

  const double corner_sign = avg_k >= 0.0 ? 1.0 : -1.0;
  const double apex_t = late_apex ? 0.58 : 0.50;

  candidate.path.points.reserve(segment.size());

  for (std::size_t i = 0; i < segment.size(); ++i) {
    const auto & c = segment[i];
    const double t = static_cast<double>(i) / static_cast<double>(segment.size() - 1);

    // ratio: 0.0 right boundary, 1.0 left boundary
    // left turn(+): outside right at entry/exit, inside left at apex
    // right turn(-): outside left at entry/exit, inside right at apex
    const double outside_ratio = corner_sign > 0.0 ? 0.27 : 0.73;
    const double inside_ratio = corner_sign > 0.0 ? 0.73 : 0.27;

    const double sigma = 0.21;
    const double g = std::exp(-0.5 * std::pow((t - apex_t) / sigma, 2.0));

    double ratio = outside_ratio + g * (inside_ratio - outside_ratio);

    const double min_ratio = wall_margin_m_ / std::max(0.1, c.width);
    const double max_ratio = 1.0 - min_ratio;

    ratio = clamp(ratio, min_ratio, max_ratio);

    LocalPathPoint p;
    p.x = c.right_x + ratio * (c.left_x - c.right_x);
    p.y = c.right_y + ratio * (c.left_y - c.right_y);
    p.d = ratio - 0.5;
    p.right_clearance = ratio * c.width;
    p.left_clearance = (1.0 - ratio) * c.width;

    candidate.path.points.push_back(p);
  }

  recomputeGeometry(candidate.path);

  bool collision_free = true;
  candidate.cost = evaluateCost(candidate.path, obstacles, collision_free);
  candidate.path.valid = candidate.path.valid && collision_free;

  candidate.path.decision.valid = candidate.path.valid;
  candidate.path.decision.reason = candidate.reason;
  candidate.path.decision.peak_shift = 0.5;
  candidate.path.decision.shift_direction = corner_sign;
  candidate.path.decision.chosen_left = corner_sign > 0.0;

  return candidate;
}

double BoundaryCorridorPlanner::obstacleDistanceCost(
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

      if (d < obstacle_margin_m_) {
        collision_free = false;
      }
    }
  }

  if (!std::isfinite(min_dist) || min_dist == std::numeric_limits<double>::max()) {
    return 0.0;
  }

  return 4.0 / std::max(0.15, min_dist);
}

double BoundaryCorridorPlanner::evaluateCost(
  const LocalPath & path,
  const ObstacleList & obstacles,
  bool & collision_free) const
{
  collision_free = true;

  if (!path.valid || path.points.size() < 2) {
    collision_free = false;
    return std::numeric_limits<double>::infinity();
  }

  double path_length = 0.0;
  double min_clearance = std::numeric_limits<double>::max();
  double smoothness = 0.0;

  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const double dx = path.points[i].x - path.points[i - 1].x;
    const double dy = path.points[i].y - path.points[i - 1].y;
    path_length += std::hypot(dx, dy);
  }

  for (const auto & p : path.points) {
    min_clearance = std::min(min_clearance, std::min(p.left_clearance, p.right_clearance));
  }

  for (std::size_t i = 1; i < path.points.size(); ++i) {
    smoothness += std::abs(path.points[i].curvature - path.points[i - 1].curvature);
  }

  if (min_clearance < wall_margin_m_) {
    collision_free = false;
    return 1000000.0 + 1000.0 * (wall_margin_m_ - min_clearance);
  }

  bool obstacle_clear = true;
  const double obstacle_cost = obstacleDistanceCost(path, obstacles, obstacle_clear);

  if (!obstacle_clear) {
    collision_free = false;
    return 1000000.0 + obstacle_cost;
  }

  const double effective_curvature = std::max(0.012, path.max_curvature);
  const double feasible_speed = std::sqrt(max_lateral_accel_ / effective_curvature);
  const double expected_time = path_length / std::max(0.5, feasible_speed);

  const double clearance_cost = 0.75 / std::max(0.1, min_clearance);
  const double curvature_cost = 1.35 * path.max_curvature;
  const double smoothness_cost = 0.12 * smoothness;

  return
    2.45 * expected_time +
    clearance_cost +
    curvature_cost +
    smoothness_cost +
    obstacle_cost;
}

LocalPath BoundaryCorridorPlanner::plan(
  const EgoState & ego_state,
  const TrackModel & track_model,
  const ObstacleList & obstacles) const
{
  LocalPath empty;

  if (!ego_state.valid || track_model.left_boundary.size() < 4 || track_model.right_boundary.size() < 4) {
    empty.valid = false;
    empty.decision.valid = false;
    empty.decision.reason = "boundary corridor invalid input";
    return empty;
  }

  const auto corridor = buildCorridor(track_model);

  if (corridor.size() < 4) {
    empty.valid = false;
    empty.decision.valid = false;
    empty.decision.reason = "boundary corridor build failed";
    return empty;
  }

  const int start_idx = findNearestCorridorIndex(ego_state, corridor);
  const auto segment = cropHorizon(corridor, start_idx);

  if (segment.size() < 4) {
    empty.valid = false;
    empty.decision.valid = false;
    empty.decision.reason = "boundary corridor short horizon";
    return empty;
  }

  std::vector<Candidate> candidates;

  for (const double ratio : {0.24, 0.32, 0.40, 0.50, 0.60, 0.68, 0.76}) {
    candidates.push_back(makeConstantRatioCandidate(segment, ratio, obstacles));
  }

  candidates.push_back(makeApexCandidate(segment, false, obstacles));
  candidates.push_back(makeApexCandidate(segment, true, obstacles));

  Candidate best;
  best.cost = std::numeric_limits<double>::infinity();
  best.path.valid = false;

  for (const auto & candidate : candidates) {
    if (!candidate.path.valid) {
      continue;
    }

    if (candidate.cost < best.cost) {
      best = candidate;
    }
  }

  if (!best.path.valid) {
    empty.valid = false;
    empty.decision.valid = false;
    empty.decision.reason = "boundary corridor no valid candidate";
    last_selected_reason_.clear();
    last_selected_cost_ = std::numeric_limits<double>::infinity();
    last_selected_count_ = 0;
    return empty;
  }

  // Hysteresis:
  // If the previous candidate is still valid and only slightly worse,
  // keep it instead of switching every tick.
  if (!last_selected_reason_.empty() && last_selected_count_ >= hysteresis_min_count_) {
    for (const auto & candidate : candidates) {
      if (!candidate.path.valid) {
        continue;
      }

      if (candidate.reason != last_selected_reason_) {
        continue;
      }

      const double keep_limit = best.cost * hysteresis_keep_ratio_;

      if (candidate.cost <= keep_limit) {
        best = candidate;
      }

      break;
    }
  }

  if (best.reason == last_selected_reason_) {
    last_selected_count_++;
  } else {
    last_selected_reason_ = best.reason;
    last_selected_count_ = 1;
  }

  last_selected_cost_ = best.cost;

  best.path.decision.valid = true;
  best.path.decision.reason = best.reason + "_stable";

  return best.path;
}

}  // namespace race_driver
