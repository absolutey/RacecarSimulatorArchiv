#include "race_driver/cost_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "race_driver/math_utils.hpp"

namespace race_driver
{

void CostEvaluator::setWeights(
  double collision_weight,
  double boundary_weight,
  double curvature_weight,
  double deviation_weight,
  double smoothness_weight,
  double obstacle_weight)
{
  collision_weight_ = collision_weight;
  boundary_weight_ = boundary_weight;
  curvature_weight_ = curvature_weight;
  deviation_weight_ = deviation_weight;
  smoothness_weight_ = smoothness_weight;
  obstacle_weight_ = obstacle_weight;
}

CostBreakdown CostEvaluator::evaluateOne(
  PathCandidate & candidate,
  const ObstacleList & obstacles) const
{
  CostBreakdown cost;

  if (!candidate.valid || candidate.points.size() < 2) {
    candidate.collision_free = false;
    cost.collision = collision_weight_;
    cost.total = cost.collision;
    return cost;
  }

  double deviation_sum = 0.0;
  double smoothness_sum = 0.0;
  double path_length = 0.0;
  double min_boundary_margin = std::numeric_limits<double>::max();
  double min_obstacle_distance = std::numeric_limits<double>::max();
  bool has_reliable_boundary = false;
  bool has_blocking_obstacle = false;

  for (std::size_t i = 0; i < candidate.points.size(); ++i) {
    const auto & p = candidate.points[i];
    deviation_sum += std::abs(p.d);

    if (i > 0) {
      const double dx = p.x - candidate.points[i - 1].x;
      const double dy = p.y - candidate.points[i - 1].y;
      path_length += std::hypot(dx, dy);
      smoothness_sum += std::abs(p.d - candidate.points[i - 1].d);
    }

    if (p.left_clearance > 0.05 && p.right_clearance > 0.05) {
      has_reliable_boundary = true;
      const double left_margin = p.left_clearance - std::max(0.0, p.d);
      const double right_margin = p.right_clearance - std::max(0.0, -p.d);
      min_boundary_margin = std::min(min_boundary_margin, std::min(left_margin, right_margin));
    }

    if (obstacles.valid) {
      for (const auto & obs : obstacles.obstacles) {
        if (!obs.valid || !obs.blocks_center_corridor) {
          continue;
        }
        has_blocking_obstacle = true;

        if (std::abs(obs.s - p.s) > 1.3) {
          continue;
        }

        const double dist = distance2D(p.x, p.y, obs.world_x, obs.world_y);
        min_obstacle_distance = std::min(min_obstacle_distance, dist);

        const double safety_radius = 0.28 + 0.5 * std::max(obs.width, 0.18);
        if (dist < safety_radius) {
          candidate.collision_free = false;
        }
      }
    }
  }

  if (!std::isfinite(min_obstacle_distance)) {
    min_obstacle_distance = 9999.0;
  }
  candidate.min_obstacle_distance = min_obstacle_distance;

  if (!candidate.collision_free) {
    cost.collision = collision_weight_;
  }

  if (has_reliable_boundary) {
    if (min_boundary_margin < -0.05) {
      cost.boundary = boundary_weight_ * 100.0;
      candidate.collision_free = false;
    } else if (min_boundary_margin < 0.12) {
      cost.boundary = boundary_weight_ * (0.12 - min_boundary_margin + 0.5);
    } else {
      cost.boundary = 0.20 * boundary_weight_ / std::max(0.12, min_boundary_margin + 0.20);
    }
  } else {
    cost.boundary = 0.5;
  }

  // v3-2: prefer fast, low-curvature paths over conservative center hugging.
  cost.curvature = curvature_weight_ * candidate.max_curvature;
  cost.deviation = 0.35 * deviation_weight_ * deviation_sum /
    static_cast<double>(candidate.points.size());
  cost.smoothness = smoothness_weight_ * smoothness_sum;

  if (min_obstacle_distance < 999.0) {
    cost.obstacle = obstacle_weight_ / std::max(0.22, min_obstacle_distance);
  }

  if (candidate.side == CandidateSide::CENTER && has_blocking_obstacle) {
    cost.obstacle += 8.0;
  }

  const double progress = std::max(0.0, candidate.points.back().s - candidate.points.front().s);
  const double excess_length = std::max(0.0, path_length - progress);

  // Use the existing speed slot as lap-time cost: shorter and smoother candidates win.
  cost.speed = 0.35 * excess_length - 1.0 * progress;

  cost.total =
    cost.collision +
    cost.boundary +
    cost.curvature +
    cost.deviation +
    cost.smoothness +
    cost.obstacle +
    cost.speed;

  // v3-4: If the apex racing-line candidate is safe, give it a small lap-time reward.
  // This prevents centerline hugging from always winning on clear corners.
  if (
    candidate.collision_free &&
    (candidate.reason == "apex_left" || candidate.reason == "apex_right"))
  {
    cost.total -= 2.40;
  }

  return cost;
}

std::vector<PathCandidate> CostEvaluator::evaluate(
  const std::vector<PathCandidate> & candidates,
  const ObstacleList & obstacles) const
{
  std::vector<PathCandidate> evaluated = candidates;

  for (auto & candidate : evaluated) {
    candidate.cost = evaluateOne(candidate, obstacles);
  }

  std::sort(
    evaluated.begin(),
    evaluated.end(),
    [](const PathCandidate & a, const PathCandidate & b) {
      return a.cost.total < b.cost.total;
    });

  return evaluated;
}

PathCandidate CostEvaluator::selectBest(
  const std::vector<PathCandidate> & candidates) const
{
  PathCandidate fallback;
  fallback.valid = false;
  fallback.reason = "no candidate";

  if (candidates.empty()) {
    return fallback;
  }

  for (const auto & candidate : candidates) {
    if (candidate.valid && candidate.collision_free) {
      return candidate;
    }
  }

  for (const auto & candidate : candidates) {
    if (candidate.valid && candidate.points.size() >= 2) {
      fallback = candidate;
      fallback.valid = true;
      fallback.collision_free = true;
      fallback.reason = "fallback best valid candidate";
      return fallback;
    }
  }

  fallback = candidates.front();
  fallback.valid = false;
  fallback.reason = "all candidates rejected";
  return fallback;
}

}  // namespace race_driver
