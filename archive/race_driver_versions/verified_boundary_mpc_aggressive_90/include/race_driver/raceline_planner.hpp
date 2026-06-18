#pragma once

#include <string>
#include <vector>

#include "race_driver/candidate_generator.hpp"
#include "race_driver/cost_evaluator.hpp"
#include "race_driver/racing_reference.hpp"
#include "race_driver/types.hpp"

namespace race_driver
{

class RacelinePlanner
{
public:
  void setParameters(
    double planning_horizon_m,
    double obstacle_margin,
    double shift_pre_margin_m,
    double shift_post_margin_m,
    double max_peak_shift_m,
    double return_blend_length_m,
    double left_right_cost_bias);

  bool loadRacingReferenceFromCsv(const std::string & csv_path);
  void setUseRacingReference(bool enabled);
  void setRacingReferenceOptions(
    double max_offset,
    double blend,
    double boundary_margin);
  bool hasRacingReference() const;

  LocalPath plan(
    const EgoState & ego_state,
    const TrackModel & track_model,
    const ObstacleList & obstacles) const;

private:
  static LocalPath toLocalPath(
    const PathCandidate & candidate,
    const std::vector<PathCandidate> & all_candidates,
    const ObstacleList & obstacles);

  static double normalizeAngle(double angle);
  static double clamp(double value, double min_value, double max_value);

  TrackModel buildRacingPlanningModel(const TrackModel & track_model) const;
  void recomputeCenterlineGeometry(std::vector<TrackPoint> & points) const;

  CandidateGenerator candidate_generator_;
  CostEvaluator cost_evaluator_;
  RacingReference racing_reference_;

  bool use_racing_reference_{false};
  double racing_reference_max_offset_{0.45};
  double racing_reference_blend_{0.70};
  double racing_reference_boundary_margin_{0.25};

  double left_right_cost_bias_{0.0};
};

}  // namespace race_driver
