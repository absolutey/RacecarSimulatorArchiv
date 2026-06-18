#pragma once

#include <string>
#include <vector>

namespace race_driver
{

enum class DriveMode
{
  IDLE,
  WAIT_TRACK,
  WAIT_SENSOR,
  READY,
  CRUISE,
  AVOID,
  BRAKE,
  STOP
};

enum class CandidateSide
{
  CENTER,
  LEFT,
  RIGHT
};

struct EgoState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double speed{0.0};
  double s{0.0};
  double d{0.0};
  bool valid{false};
};

struct TrackPoint
{
  double x{0.0};
  double y{0.0};
  double s{0.0};
  double yaw{0.0};
  double curvature{0.0};
  double left_clearance{0.0};
  double right_clearance{0.0};
};

struct TrackModel
{
  std::vector<TrackPoint> centerline;
  std::vector<TrackPoint> left_boundary;
  std::vector<TrackPoint> right_boundary;
  double total_length{0.0};
  bool valid{false};
};

struct Obstacle
{
  double x{0.0};              // ego-local x
  double y{0.0};              // ego-local y
  double world_x{0.0};
  double world_y{0.0};

  double distance{0.0};
  double lateral_offset{0.0};
  double width{0.0};
  double length{0.0};
  int point_count{0};

  double s{0.0};
  double d{0.0};
  double s_min{0.0};
  double s_max{0.0};

  double left_free{0.0};
  double right_free{0.0};

  double risk_score{0.0};
  double priority_score{0.0};

  bool blocks_center_corridor{false};
  bool passable_left{false};
  bool passable_right{false};
  bool wall_like{false};
  bool valid{false};
};

struct ObstacleList
{
  std::vector<Obstacle> obstacles;
  bool valid{false};
};

struct LocalPathPoint
{
  double x{0.0};
  double y{0.0};
  double s{0.0};
  double d{0.0};
  double yaw{0.0};
  double curvature{0.0};
  double left_clearance{0.0};
  double right_clearance{0.0};
};

struct CostBreakdown
{
  double collision{0.0};
  double boundary{0.0};
  double curvature{0.0};
  double deviation{0.0};
  double smoothness{0.0};
  double obstacle{0.0};
  double speed{0.0};
  double total{0.0};
};

struct PathCandidate
{
  std::vector<LocalPathPoint> points;

  CandidateSide side{CandidateSide::CENTER};

  double target_d{0.0};
  double peak_shift{0.0};
  double shift_start_s{0.0};
  double shift_peak_s{0.0};
  double shift_end_s{0.0};

  double max_curvature{0.0};
  double min_left_clearance{0.0};
  double min_right_clearance{0.0};
  double min_obstacle_distance{9999.0};

  CostBreakdown cost;
  bool collision_free{true};
  bool valid{false};
  std::string reason{"unset"};
};

struct PlannerDecision
{
  DriveMode mode{DriveMode::IDLE};

  bool valid{false};
  bool has_obstacle{false};
  bool emergency_stop{false};

  PathCandidate best_candidate;
  std::vector<PathCandidate> candidates;

  Obstacle primary_obstacle;

  std::string selected_reason{"none"};
  std::string reject_reason{"none"};

  double target_speed{0.0};
};

struct LocalCorridorDecision
{
  bool valid{false};
  bool chosen_left{false};
  double shift_direction{0.0};
  double peak_shift{0.0};
  double shift_start_s{0.0};
  double plateau_start_s{0.0};
  double plateau_end_s{0.0};
  double shift_peak_s{0.0};
  double shift_end_s{0.0};
  double left_cost{0.0};
  double right_cost{0.0};
  Obstacle blocking_obstacle{};
  std::string reason{"cruise"};
};

struct LocalPath
{
  std::vector<LocalPathPoint> points;

  // Keep this for compatibility with old planner code.
  LocalCorridorDecision decision;

  double max_curvature{0.0};
  bool valid{false};
};

struct SpeedPlan
{
  DriveMode mode{DriveMode::IDLE};
  double target_speed{0.0};
  double curvature_limit_speed{0.0};
  double obstacle_limit_speed{0.0};
  double corridor_limit_speed{0.0};
  std::string reason{"none"};
  bool valid{false};
};

struct ControlCommand
{
  double steering{0.0};
  double acceleration{0.0};
  double target_speed{0.0};
  bool valid{false};
};

struct DriverDebug
{
  DriveMode mode{DriveMode::IDLE};

  int obstacle_count{0};
  int candidate_count{0};

  double ego_s{0.0};
  double ego_d{0.0};

  double primary_obstacle_s{0.0};
  double primary_obstacle_d{0.0};
  double primary_obstacle_distance{0.0};

  double best_cost{0.0};
  double best_peak_shift{0.0};
  double best_max_curvature{0.0};

  double target_speed{0.0};
  double steering{0.0};

  std::string selected_reason{"none"};
  std::string safety_reason{"none"};
};

inline std::string toString(DriveMode mode)
{
  switch (mode) {
    case DriveMode::IDLE:
      return "IDLE";
    case DriveMode::WAIT_TRACK:
      return "WAIT_TRACK";
    case DriveMode::WAIT_SENSOR:
      return "WAIT_SENSOR";
    case DriveMode::READY:
      return "READY";
    case DriveMode::CRUISE:
      return "CRUISE";
    case DriveMode::AVOID:
      return "AVOID";
    case DriveMode::BRAKE:
      return "BRAKE";
    case DriveMode::STOP:
      return "STOP";
    default:
      return "UNKNOWN";
  }
}

inline std::string toString(CandidateSide side)
{
  switch (side) {
    case CandidateSide::CENTER:
      return "CENTER";
    case CandidateSide::LEFT:
      return "LEFT";
    case CandidateSide::RIGHT:
      return "RIGHT";
    default:
      return "UNKNOWN";
  }
}

}  // namespace race_driver
