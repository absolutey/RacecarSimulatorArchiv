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
  double min_boundary_margin = std::numeric_limits<double>::max();
  double min_obstacle_distance = std::numeric_limits<double>::max();

  bool has_reliable_boundary = false;

  for (std::size_t i = 0; i < candidate.points.size(); ++i) {
    const auto & p = candidate.points[i];

    deviation_sum += std::abs(p.d);

    if (i > 0) {
      smoothness_sum += std::abs(candidate.points[i].d - candidate.points[i - 1].d);
    }

    // Boundary clearance가 0 또는 음수면 아직 신뢰 가능한 boundary 정보가 아니라고 본다.
    // v2 초기 주행 단계에서는 이 값 때문에 모든 candidate가 reject되는 것을 막는다.
    if (p.left_clearance > 0.05 && p.right_clearance > 0.05) {
      has_reliable_boundary = true;

      const double left_margin = p.left_clearance - std::max(0.0, p.d);
      const double right_margin = p.right_clearance - std::max(0.0, -p.d);
      min_boundary_margin = std::min(min_boundary_margin, std::min(left_margin, right_margin));
    }

    if (obstacles.valid) {
      for (const auto & obs : obstacles.obstacles) {
        if (!obs.valid) {
          continue;
        }

        // s가 가까운 장애물만 평가한다.
        if (std::abs(obs.s - p.s) > 1.5) {
          continue;
        }

        const double dist = distance2D(p.x, p.y, obs.world_x, obs.world_y);
        min_obstacle_distance = std::min(min_obstacle_distance, dist);

        const double safety_radius = 0.35 + 0.5 * std::max(obs.width, 0.20);

        // 진짜 중심 corridor를 막는 장애물과 너무 가까울 때만 collision 처리
        if (obs.blocks_center_corridor && dist < safety_radius) {
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

  // Boundary cost는 주행 불가 판정이 아니라 비용으로만 반영한다.
  // 단, 신뢰 가능한 boundary가 있고 정말 바깥으로 나간 경우에만 reject한다.
  if (has_reliable_boundary) {
    if (min_boundary_margin < -0.10) {
      cost.boundary = boundary_weight_ * 100.0;
      candidate.collision_free = false;
    } else if (min_boundary_margin < 0.20) {
      cost.boundary = boundary_weight_ * (0.20 - min_boundary_margin + 1.0);
    } else {
      cost.boundary = boundary_weight_ / std::max(0.20, min_boundary_margin + 0.20);
    }
  } else {
    // boundary 정보가 아직 부정확하면 reject하지 않고 약한 비용만 준다.
    cost.boundary = 1.0;
  }

  cost.curvature = curvature_weight_ * candidate.max_curvature;

  cost.deviation =
    deviation_weight_ * deviation_sum / static_cast<double>(candidate.points.size());

  cost.smoothness = smoothness_weight_ * smoothness_sum;

  if (min_obstacle_distance < 999.0) {
    cost.obstacle = obstacle_weight_ / std::max(0.20, min_obstacle_distance);
  }

  // 장애물이 있는데 CENTER를 고집하면 약간 penalty.
  // 단, 이 penalty만으로 후보를 reject하지는 않는다.
  if (candidate.side == CandidateSide::CENTER && obstacles.valid && !obstacles.obstacles.empty()) {
    cost.obstacle += 10.0;
  }

  cost.total =
    cost.collision +
    cost.boundary +
    cost.curvature +
    cost.deviation +
    cost.smoothness +
    cost.obstacle +
    cost.speed;

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

  // 모든 후보가 reject되더라도, 완전 정지로 빠지기 전에
  // 가장 비용이 낮은 valid 후보를 보수 모드로 반환한다.
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
