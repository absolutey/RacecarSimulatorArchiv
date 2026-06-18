#include "race_driver/raceline_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace race_driver
{

void RacelinePlanner::setParameters(
  double planning_horizon_m,
  double obstacle_margin,
  double shift_pre_margin_m,
  double shift_post_margin_m,
  double max_peak_shift_m,
  double return_blend_length_m,
  double left_right_cost_bias)
{
  left_right_cost_bias_ = left_right_cost_bias;

  candidate_generator_.setParameters(
    planning_horizon_m,
    obstacle_margin,
    shift_pre_margin_m,
    shift_post_margin_m,
    max_peak_shift_m,
    return_blend_length_m);

  cost_evaluator_.setWeights(
    1000000.0,
    45.0,
    10.0,
    1.0,
    3.0,
    6.0);
}

LocalPath RacelinePlanner::toLocalPath(
  const PathCandidate & candidate,
  const std::vector<PathCandidate> & all_candidates,
  const ObstacleList & obstacles)
{
  LocalPath path;

  path.points = candidate.points;
  path.max_curvature = candidate.max_curvature;
  path.valid = candidate.valid && candidate.collision_free && path.points.size() >= 2;

  path.decision.valid = path.valid;
  path.decision.peak_shift = candidate.peak_shift;
  path.decision.shift_start_s = candidate.shift_start_s;
  path.decision.shift_peak_s = candidate.shift_peak_s;
  path.decision.shift_end_s = candidate.shift_end_s;

  if (candidate.side == CandidateSide::LEFT) {
    path.decision.chosen_left = true;
    path.decision.shift_direction = 1.0;
    path.decision.reason = candidate.reason.empty() ? "left" : candidate.reason;
  } else if (candidate.side == CandidateSide::RIGHT) {
    path.decision.chosen_left = false;
    path.decision.shift_direction = -1.0;
    path.decision.reason = candidate.reason.empty() ? "right" : candidate.reason;
  } else {
    path.decision.chosen_left = false;
    path.decision.shift_direction = 0.0;
    path.decision.reason = candidate.reason.empty() ? "cruise" : candidate.reason;
  }

  path.decision.left_cost = std::numeric_limits<double>::infinity();
  path.decision.right_cost = std::numeric_limits<double>::infinity();

  for (const auto & c : all_candidates) {
    if (c.side == CandidateSide::LEFT) {
      path.decision.left_cost = std::min(path.decision.left_cost, c.cost.total);
    } else if (c.side == CandidateSide::RIGHT) {
      path.decision.right_cost = std::min(path.decision.right_cost, c.cost.total);
    }
  }

  if (!std::isfinite(path.decision.left_cost)) {
    path.decision.left_cost = 0.0;
  }

  if (!std::isfinite(path.decision.right_cost)) {
    path.decision.right_cost = 0.0;
  }

  if (obstacles.valid) {
    for (const auto & obs : obstacles.obstacles) {
      if (obs.valid && obs.blocks_center_corridor) {
        path.decision.blocking_obstacle = obs;
        break;
      }
    }
  }

  if (!path.valid) {
    path.decision.valid = false;
    path.decision.reason = candidate.reason;

    if (path.decision.reason.empty() || path.decision.reason == "unset") {
      path.decision.reason = "no valid candidate";
    }
  }

  return path;
}

LocalPath RacelinePlanner::plan(
  const EgoState & ego_state,
  const TrackModel & track_model,
  const ObstacleList & obstacles) const
{
  LocalPath empty_path;

  if (!ego_state.valid || !track_model.valid || track_model.centerline.size() < 2) {
    empty_path.valid = false;
    empty_path.decision.valid = false;
    empty_path.decision.reason = "invalid input";
    return empty_path;
  }

  auto candidates = candidate_generator_.generate(
    ego_state,
    track_model,
    obstacles);

  if (candidates.empty()) {
    empty_path.valid = false;
    empty_path.decision.valid = false;
    empty_path.decision.reason = "no candidate generated";
    return empty_path;
  }

  auto evaluated = cost_evaluator_.evaluate(
    candidates,
    obstacles);

  if (left_right_cost_bias_ != 0.0) {
    for (auto & candidate : evaluated) {
      if (candidate.side == CandidateSide::LEFT) {
        candidate.cost.total += left_right_cost_bias_;
      } else if (candidate.side == CandidateSide::RIGHT) {
        candidate.cost.total -= left_right_cost_bias_;
      }
    }

    std::sort(
      evaluated.begin(),
      evaluated.end(),
      [](const PathCandidate & a, const PathCandidate & b) {
        return a.cost.total < b.cost.total;
      });
  }

  const PathCandidate best = cost_evaluator_.selectBest(evaluated);

  return toLocalPath(
    best,
    evaluated,
    obstacles);
}

}  // namespace race_driver
