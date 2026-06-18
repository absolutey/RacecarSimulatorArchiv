#include "race_driver/track_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace race_driver
{

namespace
{
double normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

TrackPoint interpolateTrackPoint(const TrackPoint & a, const TrackPoint & b, double t)
{
  TrackPoint out;
  out.x = a.x + t * (b.x - a.x);
  out.y = a.y + t * (b.y - a.y);
  out.s = a.s + t * (b.s - a.s);
  const double dyaw = normalizeAngle(b.yaw - a.yaw);
  out.yaw = normalizeAngle(a.yaw + t * dyaw);
  out.curvature = a.curvature + t * (b.curvature - a.curvature);
  out.left_clearance = a.left_clearance + t * (b.left_clearance - a.left_clearance);
  out.right_clearance = a.right_clearance + t * (b.right_clearance - a.right_clearance);
  return out;
}

void computeYawAndCurvature(std::vector<TrackPoint> & pts)
{
  if (pts.size() < 2) {
    return;
  }

  pts[0].s = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const double dx = pts[i].x - pts[i - 1].x;
    const double dy = pts[i].y - pts[i - 1].y;
    pts[i].s = pts[i - 1].s + std::sqrt(dx * dx + dy * dy);
  }

  for (std::size_t i = 0; i < pts.size(); ++i) {
    const std::size_t prev_i = (i == 0) ? 0 : i - 1;
    const std::size_t next_i = (i + 1 < pts.size()) ? i + 1 : pts.size() - 1;
    const double dx = pts[next_i].x - pts[prev_i].x;
    const double dy = pts[next_i].y - pts[prev_i].y;
    pts[i].yaw = std::atan2(dy, dx);
    pts[i].curvature = 0.0;

    if (i > 0 && i + 1 < pts.size()) {
      const double yaw_prev = std::atan2(pts[i].y - pts[i - 1].y, pts[i].x - pts[i - 1].x);
      const double yaw_next = std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
      const double ds = std::max(1e-3, pts[i + 1].s - pts[i - 1].s);
      pts[i].curvature = std::abs(normalizeAngle(yaw_next - yaw_prev)) / ds;
    }
  }
}

double signedDistanceToBoundary(const TrackPoint & center, const std::vector<TrackPoint> & boundary)
{
  if (boundary.empty()) {
    return 0.0;
  }

  const double nx = -std::sin(center.yaw);
  const double ny = std::cos(center.yaw);

  double best_abs_proj = std::numeric_limits<double>::max();
  double best_proj = 0.0;
  for (const auto & b : boundary) {
    const double dx = b.x - center.x;
    const double dy = b.y - center.y;
    const double proj = dx * nx + dy * ny;
    const double lateral_err = std::abs(proj);
    if (lateral_err < best_abs_proj) {
      best_abs_proj = lateral_err;
      best_proj = proj;
    }
  }
  return best_proj;
}
}  // namespace

std::vector<TrackPoint> TrackManager::convertPath(const nav_msgs::msg::Path & msg)
{
  std::vector<TrackPoint> out;
  out.reserve(msg.poses.size());

  for (const auto & pose_stamped : msg.poses) {
    TrackPoint p;
    p.x = pose_stamped.pose.position.x;
    p.y = pose_stamped.pose.position.y;
    out.push_back(p);
  }

  computeYawAndCurvature(out);
  return out;
}

double TrackManager::distance(const TrackPoint & a, const TrackPoint & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

void TrackManager::updateDerivedFields()
{
  auto & cl = track_model_.centerline;
  if (cl.size() < 2 || track_model_.left_boundary.empty() || track_model_.right_boundary.empty()) {
    track_model_.valid = false;
    return;
  }

  computeYawAndCurvature(cl);
  track_model_.total_length = cl.empty() ? 0.0 : cl.back().s;

  for (auto & p : cl) {
    const double left_signed = signedDistanceToBoundary(p, track_model_.left_boundary);
    const double right_signed = signedDistanceToBoundary(p, track_model_.right_boundary);

    p.left_clearance = std::max(0.0, left_signed > 0.0 ? left_signed : -right_signed);
    p.right_clearance = std::max(0.0, right_signed < 0.0 ? -right_signed : left_signed);

    if (p.left_clearance <= 0.0) {
      p.left_clearance = std::max(0.0, std::abs(left_signed));
    }
    if (p.right_clearance <= 0.0) {
      p.right_clearance = std::max(0.0, std::abs(right_signed));
    }
  }

  track_model_.valid = true;
}

void TrackManager::updateCenterline(const nav_msgs::msg::Path & msg)
{
  track_model_.centerline = convertPath(msg);
  updateDerivedFields();
}

void TrackManager::updateLeftBoundary(const nav_msgs::msg::Path & msg)
{
  track_model_.left_boundary = convertPath(msg);
  updateDerivedFields();
}

void TrackManager::updateRightBoundary(const nav_msgs::msg::Path & msg)
{
  track_model_.right_boundary = convertPath(msg);
  updateDerivedFields();
}

TrackModel TrackManager::getTrackModel() const
{
  return track_model_;
}

bool TrackManager::isReady() const
{
  return track_model_.valid;
}

int TrackManager::findNearestCenterIndex(double x, double y) const
{
  if (track_model_.centerline.empty()) {
    return -1;
  }

  int best_idx = 0;
  double best_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < track_model_.centerline.size(); ++i) {
    const double dx = track_model_.centerline[i].x - x;
    const double dy = track_model_.centerline[i].y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_dist) {
      best_dist = d2;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

double TrackManager::projectToCenterS(double x, double y, int * nearest_idx) const
{
  const int idx = findNearestCenterIndex(x, y);
  if (nearest_idx != nullptr) {
    *nearest_idx = idx;
  }
  if (idx < 0) {
    return 0.0;
  }
  return track_model_.centerline[static_cast<std::size_t>(idx)].s;
}

double TrackManager::computeSignedLateralOffset(double x, double y, int center_idx) const
{
  if (center_idx < 0 || static_cast<std::size_t>(center_idx) >= track_model_.centerline.size()) {
    return 0.0;
  }

  const auto & c = track_model_.centerline[static_cast<std::size_t>(center_idx)];
  const double nx = -std::sin(c.yaw);
  const double ny = std::cos(c.yaw);
  const double dx = x - c.x;
  const double dy = y - c.y;
  return dx * nx + dy * ny;
}

bool TrackManager::frenetFromXY(double x, double y, double & s, double & d, int & nearest_idx) const
{
  if (!track_model_.valid || track_model_.centerline.empty()) {
    return false;
  }
  nearest_idx = findNearestCenterIndex(x, y);
  if (nearest_idx < 0) {
    return false;
  }
  s = track_model_.centerline[static_cast<std::size_t>(nearest_idx)].s;
  d = computeSignedLateralOffset(x, y, nearest_idx);
  return true;
}

bool TrackManager::getCenterPointAtS(double query_s, TrackPoint & pt) const
{
  if (!track_model_.valid || track_model_.centerline.empty()) {
    return false;
  }

  const auto & cl = track_model_.centerline;
  if (query_s <= cl.front().s) {
    pt = cl.front();
    return true;
  }
  if (query_s >= cl.back().s) {
    pt = cl.back();
    return true;
  }

  for (std::size_t i = 1; i < cl.size(); ++i) {
    if (cl[i].s >= query_s) {
      const double span = std::max(1e-6, cl[i].s - cl[i - 1].s);
      const double t = (query_s - cl[i - 1].s) / span;
      pt = interpolateTrackPoint(cl[i - 1], cl[i], t);
      return true;
    }
  }

  pt = cl.back();
  return true;
}

}  // namespace race_driver
