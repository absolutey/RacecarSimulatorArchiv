#pragma once

#include <vector>

#include "race_driver/candidate_generator.hpp"
#include "race_driver/cost_evaluator.hpp"
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

  LocalPath plan(
    const EgoState & ego_state,
    const TrackModel & track_model,
    const ObstacleList & obstacles) const;

private:
  static LocalPath toLocalPath(
    const PathCandidate & candidate,
    const std::vector<PathCandidate> & all_candidates,
    const ObstacleList & obstacles);

  CandidateGenerator candidate_generator_;
  CostEvaluator cost_evaluator_;

  double left_right_cost_bias_{0.0};
};

}  // namespace race_driver
