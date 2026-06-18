#include "race_driver/racing_reference.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace race_driver
{

double RacingReference::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

bool RacingReference::parseNumericCsvLine(
  const std::string & line,
  std::vector<double> & values)
{
  values.clear();

  if (line.empty()) {
    return false;
  }

  const auto first = line.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return false;
  }

  if (line[first] == '#') {
    return false;
  }

  std::string normalized = line;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::replace(normalized.begin(), normalized.end(), ';', ' ');

  std::stringstream ss(normalized);
  std::string token;

  while (ss >> token) {
    try {
      values.push_back(std::stod(token));
    } catch (...) {
      values.clear();
      return false;
    }
  }

  return values.size() >= 2;
}

bool RacingReference::loadFromCsv(const std::string & csv_path)
{
  source_path_ = csv_path;
  points_.clear();
  valid_ = false;

  std::ifstream file(csv_path);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  std::vector<double> values;

  while (std::getline(file, line)) {
    if (!parseNumericCsvLine(line, values)) {
      continue;
    }

    TrackPoint p;
    p.x = values[0];
    p.y = values[1];

    // iccas2025_path_spline.csv:
    // x_m, y_m, w_tr_right_m, w_tr_left_m
    if (values.size() >= 4) {
      p.right_clearance = std::max(0.0, values[2]);
      p.left_clearance = std::max(0.0, values[3]);
    }

    points_.push_back(p);
  }

  if (points_.size() < 4) {
    points_.clear();
    return false;
  }

  computeGeometry();
  valid_ = true;
  return true;
}

void RacingReference::computeGeometry()
{
  if (points_.size() < 2) {
    return;
  }

  points_[0].s = 0.0;

  for (std::size_t i = 1; i < points_.size(); ++i) {
    const double dx = points_[i].x - points_[i - 1].x;
    const double dy = points_[i].y - points_[i - 1].y;
    points_[i].s = points_[i - 1].s + std::hypot(dx, dy);
  }

  for (std::size_t i = 0; i < points_.size(); ++i) {
    const std::size_t prev_i = i == 0 ? 0 : i - 1;
    const std::size_t next_i = std::min(points_.size() - 1, i + 1);

    const double dx = points_[next_i].x - points_[prev_i].x;
    const double dy = points_[next_i].y - points_[prev_i].y;

    points_[i].yaw = std::atan2(dy, dx);
    points_[i].curvature = 0.0;
  }

  for (std::size_t i = 1; i + 1 < points_.size(); ++i) {
    const double yaw_prev = points_[i - 1].yaw;
    const double yaw_next = points_[i + 1].yaw;
    const double ds = std::max(1e-6, points_[i + 1].s - points_[i - 1].s);

    points_[i].curvature = normalizeAngle(yaw_next - yaw_prev) / ds;
  }

  points_.front().curvature = points_[1].curvature;
  points_.back().curvature = points_[points_.size() - 2].curvature;
}

bool RacingReference::valid() const
{
  return valid_ && points_.size() >= 4;
}

const std::string & RacingReference::sourcePath() const
{
  return source_path_;
}

const std::vector<TrackPoint> & RacingReference::points() const
{
  return points_;
}

bool RacingReference::lateralOffsetHint(
  const TrackPoint & center_point,
  double & offset_d) const
{
  offset_d = 0.0;

  if (!valid()) {
    return false;
  }

  double best_dist2 = std::numeric_limits<double>::max();
  std::size_t best_idx = 0;

  for (std::size_t i = 0; i < points_.size(); ++i) {
    const double dx = points_[i].x - center_point.x;
    const double dy = points_[i].y - center_point.y;
    const double dist2 = dx * dx + dy * dy;

    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      best_idx = i;
    }
  }

  const auto & ref = points_[best_idx];

  const double dx = ref.x - center_point.x;
  const double dy = ref.y - center_point.y;

  // center_path의 left normal 기준 d 계산
  const double nx = -std::sin(center_point.yaw);
  const double ny = std::cos(center_point.yaw);

  offset_d = dx * nx + dy * ny;
  return true;
}

}  // namespace race_driver
