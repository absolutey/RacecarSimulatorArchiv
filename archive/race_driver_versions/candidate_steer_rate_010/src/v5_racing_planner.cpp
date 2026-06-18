#include "race_driver/v5_racing_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace race_driver
{

void V5RacingPlanner::configure(
  double min_speed,
  double max_speed,
  double max_lateral_accel,
  double max_accel,
  double max_decel,
  double min_boundary_margin,
  double smoothing_gain,
  double planning_horizon_m)
{
  min_boundary_margin_ = std::max(0.0, min_boundary_margin);
  planning_horizon_m_ = std::max(1.0, planning_horizon_m);

  raceline_optimizer_.configure(min_boundary_margin, smoothing_gain);
  speed_profile_optimizer_.configure(
    min_speed,
    max_speed,
    max_lateral_accel,
    max_accel,
    max_decel);
}

bool V5RacingPlanner::loadReferenceFromCsv(const std::string & csv_path)
{
  if (csv_path.empty()) {
    return false;
  }

  return racing_reference_.loadFromCsv(csv_path);
}

bool V5RacingPlanner::hasReference() const
{
  return racing_reference_.valid();
}

GlobalRaceline V5RacingPlanner::buildInitialRaceline(
  const std::vector<geometry_msgs::msg::Point> & center_points)
{
  GlobalRaceline line = raceline_optimizer_.buildCenterlineSeed(center_points);

  if (!line.valid) {
    return line;
  }

  line.speed_profile = speed_profile_optimizer_.computeSpeedProfile(
    line.curvature,
    line.segment_lengths);

  const auto lap_eval = lap_time_evaluator_.evaluate(
    line.segment_lengths,
    line.speed_profile);

  if (lap_eval.valid) {
    line.estimated_lap_time_sec = lap_eval.estimated_time_sec;
  }

  return line;
}

LocalPath V5RacingPlanner::plan(
  const EgoState & ego_state,
  const TrackModel & track_model)
{
  if (hasReference()) {
    LocalPath ref_path = planFromReference(ego_state, track_model);

    if (ref_path.valid) {
      return ref_path;
    }
  }

  return planFromCenterline(ego_state, track_model);
}

LocalPath V5RacingPlanner::planFromReference(
  const EgoState & ego_state,
  const TrackModel & track_model)
{
  LocalPath path;
  path.decision.reason = "v5_ref_unset";

  if (!ego_state.valid || !track_model.valid || !racing_reference_.valid()) {
    path.decision.reason = "v5_ref_input_invalid";
    return path;
  }

  const auto & ref = racing_reference_.points();

  if (ref.size() < 4) {
    path.decision.reason = "v5_ref_too_short";
    return path;
  }

  int nearest_idx = 0;
  double best_dist = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < ref.size(); ++i) {
    const double d = distance2D(ego_state.x, ego_state.y, ref[i].x, ref[i].y);
    if (d < best_dist) {
      best_dist = d;
      nearest_idx = static_cast<int>(i);
    }
  }

  const double start_s = ref[nearest_idx].s;
  const double end_s = start_s + planning_horizon_m_;

  path.points.reserve(ref.size());

  double min_corridor_clearance = std::numeric_limits<double>::max();
  int clearance_sample_count = 0;

  for (std::size_t i = static_cast<std::size_t>(nearest_idx); i < ref.size(); ++i) {
    const auto & tp = ref[i];

    if (tp.s < start_s) {
      continue;
    }
    if (tp.s > end_s) {
      break;
    }

    LocalPathPoint p;
    p.x = tp.x;
    p.y = tp.y;
    p.s = tp.s;
    p.d = 0.0;
    p.yaw = tp.yaw;
    p.curvature = tp.curvature;
    p.left_clearance = tp.left_clearance;
    p.right_clearance = tp.right_clearance;

    if (p.left_clearance > 0.01 && p.right_clearance > 0.01) {
      const double c = std::min(p.left_clearance, p.right_clearance);
      min_corridor_clearance = std::min(min_corridor_clearance, c);
      clearance_sample_count++;
    }

    path.max_curvature = std::max(path.max_curvature, std::abs(p.curvature));
    path.points.push_back(p);
  }

  if (path.points.size() < 4) {
    path.decision.reason = "v5_ref_too_few_points";
    return path;
  }

  if (clearance_sample_count > 0 && min_corridor_clearance < min_boundary_margin_) {
    path.valid = false;
    path.decision.valid = false;
    path.decision.reason = "v5_ref_clearance_guard_fallback";
    return path;
  }

  path.valid = true;
  path.decision.valid = true;
  path.decision.reason = "v5_reference_seed_guarded";

  return path;
}

LocalPath V5RacingPlanner::planFromCenterline(
  const EgoState & ego_state,
  const TrackModel & track_model)
{
  LocalPath path;
  path.decision.reason = "v5_unset";

  if (!ego_state.valid || !track_model.valid || track_model.centerline.size() < 3) {
    path.decision.reason = "v5_input_invalid";
    return path;
  }

  const auto & center = track_model.centerline;

  int nearest_idx = 0;
  double best_dist = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < center.size(); ++i) {
    const double d = distance2D(ego_state.x, ego_state.y, center[i].x, center[i].y);
    if (d < best_dist) {
      best_dist = d;
      nearest_idx = static_cast<int>(i);
    }
  }

  const double start_s = center[nearest_idx].s;
  const double end_s = start_s + planning_horizon_m_;

  path.points.reserve(center.size());

  double min_corridor_clearance = std::numeric_limits<double>::max();
  int clearance_sample_count = 0;

  for (std::size_t i = static_cast<std::size_t>(nearest_idx); i < center.size(); ++i) {
    const auto & tp = center[i];

    if (tp.s < start_s) {
      continue;
    }
    if (tp.s > end_s) {
      break;
    }

    LocalPathPoint p;
    p.x = tp.x;
    p.y = tp.y;
    p.s = tp.s;
    p.d = 0.0;
    p.yaw = tp.yaw;
    p.curvature = tp.curvature;
    p.left_clearance = tp.left_clearance;
    p.right_clearance = tp.right_clearance;

    if (p.left_clearance > 0.01 && p.right_clearance > 0.01) {
      const double c = std::min(p.left_clearance, p.right_clearance);
      min_corridor_clearance = std::min(min_corridor_clearance, c);
      clearance_sample_count++;
    }

    path.max_curvature = std::max(path.max_curvature, std::abs(p.curvature));
    path.points.push_back(p);
  }

  if (path.points.size() < 4) {
    path.decision.reason = "v5_too_few_points";
    return path;
  }

  if (clearance_sample_count > 0 && min_corridor_clearance < min_boundary_margin_) {
    path.valid = false;
    path.decision.valid = false;
    path.decision.reason = "v5_clearance_guard_fallback";
    return path;
  }

  path.valid = true;
  path.decision.valid = true;
  path.decision.reason = "v5_centerline_seed_guarded";

  return path;
}

double V5RacingPlanner::normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

double V5RacingPlanner::distance2D(double ax, double ay, double bx, double by)
{
  return std::hypot(bx - ax, by - ay);
}

}  // namespace race_driver
