#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

#include "race_driver/global_raceline_optimizer.hpp"
#include "race_driver/lap_time_evaluator.hpp"
#include "race_driver/racing_reference.hpp"
#include "race_driver/speed_profile_optimizer.hpp"
#include "race_driver/types.hpp"

namespace race_driver
{

class V5RacingPlanner
{
public:
  V5RacingPlanner() = default;

  void configure(
    double min_speed,
    double max_speed,
    double max_lateral_accel,
    double max_accel,
    double max_decel,
    double min_boundary_margin,
    double smoothing_gain,
    double planning_horizon_m);

  bool loadReferenceFromCsv(const std::string & csv_path);
  bool hasReference() const;

  GlobalRaceline buildInitialRaceline(
    const std::vector<geometry_msgs::msg::Point> & center_points);

  LocalPath plan(
    const EgoState & ego_state,
    const TrackModel & track_model);

private:
  GlobalRacelineOptimizer raceline_optimizer_;
  SpeedProfileOptimizer speed_profile_optimizer_;
  LapTimeEvaluator lap_time_evaluator_;
  RacingReference racing_reference_;

  double planning_horizon_m_{8.0};
  double min_boundary_margin_{0.22};

  LocalPath planFromReference(
    const EgoState & ego_state,
    const TrackModel & track_model);

  LocalPath planFromCenterline(
    const EgoState & ego_state,
    const TrackModel & track_model);

  static double normalizeAngle(double a);
  static double distance2D(double ax, double ay, double bx, double by);
};

}  // namespace race_driver
