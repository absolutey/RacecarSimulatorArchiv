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

bool RacelinePlanner::loadRacingReferenceFromCsv(const std::string & csv_path)
{
  return racing_reference_.loadFromCsv(csv_path);
}

void RacelinePlanner::setUseRacingReference(bool enabled)
{
  use_racing_reference_ = enabled;
}

void RacelinePlanner::setRacingReferenceOptions(
  double max_offset,
  double blend,
  double boundary_margin)
{
  racing_reference_max_offset_ = std::max(0.0, max_offset);
  racing_reference_blend_ = clamp(blend, 0.0, 1.0);
  racing_reference_boundary_margin_ = std::max(0.0, boundary_margin);
}

bool RacelinePlanner::hasRacingReference() const
{
  return racing_reference_.valid();
}

double RacelinePlanner::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double RacelinePlanner::clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

void RacelinePlanner::recomputeCenterlineGeometry(std::vector<TrackPoint> & points) const
{
  if (points.size() < 2) {
    return;
  }

  points[0].s = 0.0;

  for (std::size_t i = 1; i < points.size(); ++i) {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    points[i].s = points[i - 1].s + std::hypot(dx, dy);
  }

  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::size_t prev_i = i == 0 ? 0 : i - 1;
    const std::size_t next_i = std::min(points.size() - 1, i + 1);

    const double dx = points[next_i].x - points[prev_i].x;
    const double dy = points[next_i].y - points[prev_i].y;

    points[i].yaw = std::atan2(dy, dx);
    points[i].curvature = 0.0;
  }

  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const double yaw_prev = points[i - 1].yaw;
    const double yaw_next = points[i + 1].yaw;
    const double ds = std::max(1e-6, points[i + 1].s - points[i - 1].s);

    points[i].curvature = normalizeAngle(yaw_next - yaw_prev) / ds;
  }

  points.front().curvature = points[1].curvature;
  points.back().curvature = points[points.size() - 2].curvature;
}

TrackModel RacelinePlanner::buildRacingPlanningModel(const TrackModel & track_model) const
{
  TrackModel planning_model = track_model;

  if (
    !use_racing_reference_ ||
    !racing_reference_.valid() ||
    track_model.centerline.size() < 2)
  {
    return planning_model;
  }

  planning_model.centerline = track_model.centerline;

  for (auto & p : planning_model.centerline) {
    double hint_d = 0.0;

    if (!racing_reference_.lateralOffsetHint(p, hint_d)) {
      continue;
    }

    double desired_d = hint_d * racing_reference_blend_;

    const double left_allowed = std::max(
      0.0,
      p.left_clearance - racing_reference_boundary_margin_);

    const double right_allowed = std::max(
      0.0,
      p.right_clearance - racing_reference_boundary_margin_);

    const double max_left = std::min(racing_reference_max_offset_, left_allowed);
    const double max_right = std::min(racing_reference_max_offset_, right_allowed);

    desired_d = clamp(desired_d, -max_right, max_left);

    const double nx = -std::sin(p.yaw);
    const double ny = std::cos(p.yaw);

    p.x += desired_d * nx;
    p.y += desired_d * ny;

    if (desired_d >= 0.0) {
      p.left_clearance = std::max(0.0, p.left_clearance - desired_d);
      p.right_clearance += desired_d;
    } else {
      p.right_clearance = std::max(0.0, p.right_clearance + desired_d);
      p.left_clearance -= desired_d;
    }
  }

  recomputeCenterlineGeometry(planning_model.centerline);
  planning_model.valid = planning_model.centerline.size() >= 2;

  return planning_model;
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

  const TrackModel planning_model = buildRacingPlanningModel(track_model);

  auto candidates = candidate_generator_.generate(
    ego_state,
    planning_model,
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
