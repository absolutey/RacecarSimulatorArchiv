// ROS-independent safety contract used by the selectable C++ actuator Guard.
// Numerical command bounds and short-lived lease must BOTH be satisfied
// before any stop proof can mark a proposal CERTIFIED.
#pragma once
#include <cmath>
#include <cstdint>
namespace native_guard_intent_bounds {
constexpr std::int64_t kMaxCreatedAgeNs = 75'000'000;
constexpr std::int64_t kMaxIntentLeaseNs = 100'000'000;
inline bool admissible(double steer, double accel, std::int64_t created,
                       std::int64_t expires, std::int64_t now,
                       double max_steer, double max_accel, double max_brake) {
  if (!std::isfinite(steer) || !std::isfinite(accel) ||
      !std::isfinite(max_steer) || !std::isfinite(max_accel) ||
      !std::isfinite(max_brake) || max_steer <= 0.0 ||
      max_accel <= 0.0 || max_brake <= 0.0) return false;
  // Check ordering BEFORE subtracting to avoid overflow from hostile stamps.
  if (created <= 0 || now < created || expires < now) return false;
  if (now - created > kMaxCreatedAgeNs ||
      expires - created > kMaxIntentLeaseNs) return false;
  return std::abs(steer) <= max_steer &&
         accel >= -max_brake && accel <= max_accel;
}
}  // namespace native_guard_intent_bounds
