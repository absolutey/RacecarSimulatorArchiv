#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace race_driver
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

inline double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

inline double sqr(double value)
{
  return value * value;
}

inline double distance2D(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

inline double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

inline double angleDiff(double target, double current)
{
  return normalizeAngle(target - current);
}

inline double lerp(double a, double b, double t)
{
  const double u = clamp(t, 0.0, 1.0);
  return a + (b - a) * u;
}

inline double cosineBlend01(double t)
{
  const double u = clamp(t, 0.0, 1.0);
  return 0.5 * (1.0 - std::cos(kPi * u));
}

inline double cosineInterpolate(double a, double b, double t)
{
  return lerp(a, b, cosineBlend01(t));
}

inline double sign(double value)
{
  if (value > 0.0) {
    return 1.0;
  }
  if (value < 0.0) {
    return -1.0;
  }
  return 0.0;
}

inline bool isFinite(double value)
{
  return std::isfinite(value);
}

inline double safeDivide(double numerator, double denominator, double fallback = 0.0)
{
  if (std::abs(denominator) < 1e-9) {
    return fallback;
  }
  return numerator / denominator;
}

inline double wrapS(double s, double total_length)
{
  if (total_length <= 1e-6) {
    return s;
  }

  double wrapped = std::fmod(s, total_length);
  if (wrapped < 0.0) {
    wrapped += total_length;
  }
  return wrapped;
}

inline double forwardArcDistance(double from_s, double to_s, double total_length)
{
  if (total_length <= 1e-6) {
    return to_s - from_s;
  }

  double diff = to_s - from_s;
  while (diff < 0.0) {
    diff += total_length;
  }
  while (diff >= total_length) {
    diff -= total_length;
  }
  return diff;
}

inline double signedLateralOffset(
  double ref_x,
  double ref_y,
  double ref_yaw,
  double query_x,
  double query_y)
{
  const double nx = -std::sin(ref_yaw);
  const double ny = std::cos(ref_yaw);
  const double vx = query_x - ref_x;
  const double vy = query_y - ref_y;
  return vx * nx + vy * ny;
}

inline double curvatureFromThreePoints(
  double x1, double y1,
  double x2, double y2,
  double x3, double y3)
{
  const double a = distance2D(x2, y2, x3, y3);
  const double b = distance2D(x1, y1, x3, y3);
  const double c = distance2D(x1, y1, x2, y2);

  const double area2 =
    std::abs((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1));

  const double denom = a * b * c;
  if (denom < 1e-9) {
    return 0.0;
  }

  return 2.0 * area2 / denom;
}

inline double logistic(double x)
{
  if (x > 50.0) {
    return 1.0;
  }
  if (x < -50.0) {
    return 0.0;
  }
  return 1.0 / (1.0 + std::exp(-x));
}

}  // namespace race_driver
