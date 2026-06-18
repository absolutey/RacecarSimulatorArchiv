#include "race_driver/candidate_generator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "race_driver/math_utils.hpp"

namespace race_driver
{

namespace
{
constexpr double kBoundaryMargin = 0.12;

bool hasUsefulClearance(const LocalPathPoint & p, double shift)
{
  if (shift > 0.0) {
    return p.left_clearance > std::abs(shift) + kBoundaryMargin;
  }
  if (shift < 0.0) {
    return p.right_clearance > std::abs(shift) + kBoundaryMargin;
  }
  return true;
}
}

void CandidateGenerator::setParameters(
  double planning_horizon_m,
  double obstacle_margin,
  double shift_pre_margin_m,
  double shift_post_margin_m,
  double max_peak_shift_m,
  double return_blend_length_m)
{
  planning_horizon_m_ = planning_horizon_m;
  obstacle_margin_ = obstacle_margin;
  shift_pre_margin_m_ = shift_pre_margin_m;
  shift_post_margin_m_ = shift_post_margin_m;
  max_peak_shift_m_ = max_peak_shift_m;
  return_blend_length_m_ = return_blend_length_m;
}

std::size_t CandidateGenerator::findNearestIndex(
  const EgoState & ego_state,
  const std::vector<TrackPoint> & centerline)
{
  std::size_t best_idx = 0;
  double best_dist = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < centerline.size(); ++i) {
    const double d2 =
      sqr(centerline[i].x - ego_state.x) +
      sqr(centerline[i].y - ego_state.y);

    if (d2 < best_dist) {
      best_dist = d2;
      best_idx = i;
    }
  }

  return best_idx;
}

std::vector<LocalPathPoint> CandidateGenerator::buildBasePath(
  const EgoState & ego_state,
  const TrackModel & track_model,
  double & start_s) const
{
  std::vector<LocalPathPoint> path;
  start_s = 0.0;

  if (!ego_state.valid || !track_model.valid || track_model.centerline.size() < 2) {
    return path;
  }

  const auto & cl = track_model.centerline;
  const std::size_t start_idx = findNearestIndex(ego_state, cl);
  start_s = cl[start_idx].s;

  const double total_length = track_model.total_length > 1e-6 ?
    track_model.total_length : cl.back().s;

  double prev_cont_s = start_s;
  for (std::size_t step = 0; step < cl.size(); ++step) {
    const std::size_t idx = (start_idx + step) % cl.size();
    const auto & src = cl[idx];

    double cont_s = src.s;
    if (step > 0 && total_length > 1e-6 && cont_s + 1e-6 < start_s) {
      cont_s += total_length;
    }
    if (step > 0 && cont_s + 1e-6 < prev_cont_s) {
      break;
    }
    prev_cont_s = cont_s;

    const double forward = cont_s - start_s;
    if (forward < -0.1) {
      continue;
    }
    if (forward > planning_horizon_m_) {
      break;
    }

    LocalPathPoint p;
    p.x = src.x;
    p.y = src.y;
    p.s = cont_s;
    p.d = 0.0;
    p.yaw = src.yaw;
    p.curvature = src.curvature;
    p.left_clearance = src.left_clearance;
    p.right_clearance = src.right_clearance;
    path.push_back(p);
  }

  return path;
}

const Obstacle * CandidateGenerator::selectPrimaryObstacle(
  double start_s,
  const ObstacleList & obstacles) const
{
  if (!obstacles.valid || obstacles.obstacles.empty()) {
    return nullptr;
  }

  const Obstacle * best = nullptr;
  double best_forward = std::numeric_limits<double>::max();

  for (const auto & obs : obstacles.obstacles) {
    if (!obs.valid || !obs.blocks_center_corridor) {
      continue;
    }

    const double forward = obs.s - start_s;

    if (forward < -0.2 || forward > planning_horizon_m_ + 0.5) {
      continue;
    }

    if (forward < best_forward) {
      best_forward = forward;
      best = &obs;
    }
  }

  return best;
}

PathCandidate CandidateGenerator::makeCandidate(
  CandidateSide side,
  double peak_shift,
  const std::vector<LocalPathPoint> & base_path,
  const Obstacle * obstacle) const
{
  PathCandidate candidate;
  candidate.side = side;
  candidate.peak_shift = std::max(0.0, peak_shift);
  candidate.valid = base_path.size() >= 2;
  candidate.collision_free = true;

  if (!candidate.valid) {
    candidate.reason = "base path too short";
    return candidate;
  }

  candidate.points = base_path;

  if (side == CandidateSide::CENTER || candidate.peak_shift <= 1e-6) {
    candidate.target_d = 0.0;
    candidate.reason = "cruise";
    recomputeCandidateGeometry(candidate);
    return candidate;
  }

  const double direction = side == CandidateSide::LEFT ? 1.0 : -1.0;
  candidate.target_d = direction * candidate.peak_shift;

  if (obstacle != nullptr) {
    candidate.shift_start_s = obstacle->s - shift_pre_margin_m_;
    candidate.shift_peak_s = obstacle->s;
    candidate.shift_end_s = obstacle->s + shift_post_margin_m_;
    candidate.reason = side == CandidateSide::LEFT ? "avoid_left" : "avoid_right";
  } else {
    // v3-2: even without obstacles, create mild racing-line offset candidates.
    candidate.shift_start_s = candidate.points.front().s;
    candidate.shift_peak_s = candidate.points.front().s + 0.5 * planning_horizon_m_;
    candidate.shift_end_s = candidate.points.front().s + planning_horizon_m_;
    candidate.reason = side == CandidateSide::LEFT ? "race_left" : "race_right";
  }

  const double plateau_half = obstacle != nullptr ?
    std::max(0.20, return_blend_length_m_ * 0.5) :
    0.5 * planning_horizon_m_;
  const double plateau_start_s = candidate.shift_peak_s - plateau_half;
  const double plateau_end_s = candidate.shift_peak_s + plateau_half;

  for (auto & p : candidate.points) {
    double weight = 1.0;

    if (obstacle != nullptr) {
      if (p.s < candidate.shift_start_s || p.s > candidate.shift_end_s) {
        continue;
      }

      if (p.s <= plateau_start_s) {
        const double t = (p.s - candidate.shift_start_s) /
          std::max(1e-6, plateau_start_s - candidate.shift_start_s);
        weight = cosineBlend01(t);
      } else if (p.s <= plateau_end_s) {
        weight = 1.0;
      } else {
        const double t = (p.s - plateau_end_s) /
          std::max(1e-6, candidate.shift_end_s - plateau_end_s);
        weight = 1.0 - cosineBlend01(t);
      }
    }

    const double nominal_shift = direction * candidate.peak_shift * weight;
    const double side_clearance = direction > 0.0 ? p.left_clearance : p.right_clearance;
    const double max_allowed = side_clearance > 0.05 ?
      std::max(0.0, side_clearance - kBoundaryMargin) :
      std::abs(nominal_shift);

    const double applied_abs = clamp(std::abs(nominal_shift), 0.0, max_allowed);
    const double applied_shift = direction * applied_abs;

    if (!hasUsefulClearance(p, applied_shift)) {
      continue;
    }

    const double nx = -std::sin(p.yaw);
    const double ny = std::cos(p.yaw);

    p.x += applied_shift * nx;
    p.y += applied_shift * ny;
    p.d = applied_shift;
  }

  recomputeCandidateGeometry(candidate);
  return candidate;
}

PathCandidate CandidateGenerator::makeApexCandidate(
  const std::vector<LocalPathPoint> & base_path) const
{
  PathCandidate candidate;
  candidate.valid = base_path.size() >= 4;
  candidate.collision_free = true;
  candidate.reason = "apex_racing_line";

  if (!candidate.valid) {
    candidate.reason = "base path too short for apex";
    return candidate;
  }

  candidate.points = base_path;

  const double s0 = candidate.points.front().s;
  const double s1 = candidate.points.back().s;
  const double horizon = std::max(1e-6, s1 - s0);

  double weighted_curvature_sum = 0.0;
  double weight_sum = 0.0;

  for (const auto & p : candidate.points) {
    const double t = std::clamp((p.s - s0) / horizon, 0.0, 1.0);

    // 코너 중반부를 더 중요하게 보고 코너 방향을 추정한다.
    const double mid_weight = std::sin(M_PI * t);
    const double w = std::max(0.0, mid_weight);

    weighted_curvature_sum += p.curvature * w;
    weight_sum += w;
  }

  if (weight_sum < 1e-6) {
    candidate.valid = false;
    candidate.reason = "no curvature weight";
    return candidate;
  }

  const double avg_curvature = weighted_curvature_sum / weight_sum;

  // 거의 직선이면 apex 후보는 의미가 없다.
  if (std::abs(avg_curvature) < 0.025) {
    candidate.valid = false;
    candidate.reason = "straight segment no apex";
    return candidate;
  }

  const double corner_sign = avg_curvature >= 0.0 ? 1.0 : -1.0;

  // left turn(+): entry right(-), apex left(+), exit right(-)
  // right turn(-): entry left(+), apex right(-), exit left(+)
  const double apex_shift = std::min(max_peak_shift_m_, 0.48);

  candidate.side = corner_sign > 0.0 ? CandidateSide::LEFT : CandidateSide::RIGHT;
  candidate.peak_shift = apex_shift;
  candidate.target_d = corner_sign * apex_shift;
  candidate.shift_start_s = s0;
  candidate.shift_peak_s = s0 + 0.50 * horizon;
  candidate.shift_end_s = s1;
  candidate.reason = corner_sign > 0.0 ? "apex_left" : "apex_right";

  for (auto & p : candidate.points) {
    const double t = std::clamp((p.s - s0) / horizon, 0.0, 1.0);

    // -cos(2*pi*t):
    // t=0   -> outside
    // t=0.5 -> inside apex
    // t=1   -> outside
    const double profile = -std::cos(2.0 * M_PI * t);
    const double nominal_shift = corner_sign * apex_shift * profile;

    const double side_clearance = nominal_shift >= 0.0 ?
      p.left_clearance :
      p.right_clearance;

    const double max_allowed = side_clearance > 0.05 ?
      std::max(0.0, side_clearance - kBoundaryMargin) :
      std::abs(nominal_shift);

    const double applied_abs = clamp(std::abs(nominal_shift), 0.0, max_allowed);
    const double applied_shift = nominal_shift >= 0.0 ? applied_abs : -applied_abs;

    if (!hasUsefulClearance(p, applied_shift)) {
      continue;
    }

    const double nx = -std::sin(p.yaw);
    const double ny = std::cos(p.yaw);

    p.x += applied_shift * nx;
    p.y += applied_shift * ny;
    p.d = applied_shift;
  }

  recomputeCandidateGeometry(candidate);
  return candidate;
}


void CandidateGenerator::recomputeCandidateGeometry(PathCandidate & candidate)
{
  if (candidate.points.size() < 2) {
    candidate.valid = false;
    candidate.reason = "candidate too short";
    return;
  }

  std::vector<double> curvatures;
  curvatures.reserve(candidate.points.size());

  candidate.max_curvature = 0.0;
  candidate.min_left_clearance = std::numeric_limits<double>::max();
  candidate.min_right_clearance = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < candidate.points.size(); ++i) {
    const std::size_t prev_i = (i == 0) ? 0 : i - 1;
    const std::size_t next_i =
      (i + 1 < candidate.points.size()) ? i + 1 : candidate.points.size() - 1;

    const double dx = candidate.points[next_i].x - candidate.points[prev_i].x;
    const double dy = candidate.points[next_i].y - candidate.points[prev_i].y;

    candidate.points[i].yaw = std::atan2(dy, dx);
    candidate.points[i].curvature = 0.0;

    if (i > 0 && i + 1 < candidate.points.size()) {
      const double raw_curvature = curvatureFromThreePoints(
        candidate.points[i - 1].x, candidate.points[i - 1].y,
        candidate.points[i].x, candidate.points[i].y,
        candidate.points[i + 1].x, candidate.points[i + 1].y);

      candidate.points[i].curvature = raw_curvature;
      curvatures.push_back(std::abs(raw_curvature));
    }

    const double left_margin =
      candidate.points[i].left_clearance - std::max(0.0, candidate.points[i].d);
    const double right_margin =
      candidate.points[i].right_clearance - std::max(0.0, -candidate.points[i].d);

    candidate.min_left_clearance = std::min(candidate.min_left_clearance, left_margin);
    candidate.min_right_clearance = std::min(candidate.min_right_clearance, right_margin);
  }

  if (!curvatures.empty()) {
    std::sort(curvatures.begin(), curvatures.end());
    const std::size_t idx75 = static_cast<std::size_t>(0.75 * static_cast<double>(curvatures.size() - 1));
    const std::size_t idx90 = static_cast<std::size_t>(0.90 * static_cast<double>(curvatures.size() - 1));
    const double c75 = curvatures[idx75];
    const double c90 = curvatures[idx90];
    const double cmax = curvatures.back();

    // v3-2 uses effective curvature, not a single spike. This is the key to racing speed.
    candidate.max_curvature = 0.70 * c75 + 0.25 * c90 + 0.05 * cmax;
  }

  if (!std::isfinite(candidate.min_left_clearance)) {
    candidate.min_left_clearance = 0.0;
  }

  if (!std::isfinite(candidate.min_right_clearance)) {
    candidate.min_right_clearance = 0.0;
  }
}

std::vector<PathCandidate> CandidateGenerator::generate(
  const EgoState & ego_state,
  const TrackModel & track_model,
  const ObstacleList & obstacles) const
{
  std::vector<PathCandidate> candidates;

  double start_s = 0.0;
  const auto base_path = buildBasePath(ego_state, track_model, start_s);

  if (base_path.size() < 2) {
    return candidates;
  }

  const Obstacle * primary = selectPrimaryObstacle(start_s, obstacles);

  candidates.push_back(makeCandidate(CandidateSide::CENTER, 0.0, base_path, primary));

  // v3-4: Add outside-inside-outside apex candidate for true racing-line behavior.
  {
    const auto apex_candidate = makeApexCandidate(base_path);
    if (apex_candidate.valid) {
      candidates.push_back(apex_candidate);
    }
  }

  // v3-2: Always generate mild minimum-path offset candidates.
  const double free_left = base_path.front().left_clearance;
  const double free_right = base_path.front().right_clearance;
  const double small_shift = clamp(0.24, 0.15, max_peak_shift_m_);
  const double medium_shift = clamp(0.42, 0.20, max_peak_shift_m_);

  if (free_left > small_shift + kBoundaryMargin) {
    candidates.push_back(makeCandidate(CandidateSide::LEFT, small_shift, base_path, nullptr));
  }
  if (free_left > medium_shift + kBoundaryMargin && std::abs(medium_shift - small_shift) > 1e-3) {
    candidates.push_back(makeCandidate(CandidateSide::LEFT, medium_shift, base_path, nullptr));
  }
  if (free_right > small_shift + kBoundaryMargin) {
    candidates.push_back(makeCandidate(CandidateSide::RIGHT, small_shift, base_path, nullptr));
  }
  if (free_right > medium_shift + kBoundaryMargin && std::abs(medium_shift - small_shift) > 1e-3) {
    candidates.push_back(makeCandidate(CandidateSide::RIGHT, medium_shift, base_path, nullptr));
  }

  if (primary == nullptr) {
    return candidates;
  }

  const double required_left = std::max(
    0.22,
    obstacle_margin_ + 0.5 * primary->width - primary->d);

  const double required_right = std::max(
    0.22,
    obstacle_margin_ + 0.5 * primary->width + primary->d);

  if (primary->passable_left) {
    const double peak1 = clamp(required_left, 0.22, max_peak_shift_m_);
    const double peak2 = clamp(required_left + 0.14, 0.22, max_peak_shift_m_);

    candidates.push_back(makeCandidate(CandidateSide::LEFT, peak1, base_path, primary));
    if (std::abs(peak2 - peak1) > 1e-3) {
      candidates.push_back(makeCandidate(CandidateSide::LEFT, peak2, base_path, primary));
    }
  }

  if (primary->passable_right) {
    const double peak1 = clamp(required_right, 0.22, max_peak_shift_m_);
    const double peak2 = clamp(required_right + 0.14, 0.22, max_peak_shift_m_);

    candidates.push_back(makeCandidate(CandidateSide::RIGHT, peak1, base_path, primary));
    if (std::abs(peak2 - peak1) > 1e-3) {
      candidates.push_back(makeCandidate(CandidateSide::RIGHT, peak2, base_path, primary));
    }
  }

  return candidates;
}

}  // namespace race_driver
