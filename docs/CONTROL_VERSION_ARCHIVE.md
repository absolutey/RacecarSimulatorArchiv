# Race driver control archive

## Policy

This repository does not keep only one final module. It keeps the current module plus meaningful historical modules so that any version can be restored after cloning.

## Important versions

- current_main: active latest local module.
- v3_final_before_v4: pre-v4 stable branch.
- v4_5_6_base_20260502_031944: map rollout and boundary corridor base.
- v5_safe_fast_031_65: v5 raceline planner experiment.
- best_boundary_mpc_92: conservative MPC-lite boundary corridor version.
- best_boundary_mpc_897: intermediate MPC tuning.
- verified_boundary_mpc_aggressive_90: aggressive MPC tuning.
- best_boundary_mpc_892_delta075: direct parent of current.
- candidate_steer_rate_010: steer-rate penalty experiment.
- before_hysteresis_20260502_104926: baseline before hysteresis.
- hysteresis_slow_20260502_105510: candidate selection hysteresis experiment.

## Restore

    ./tools/list_race_driver_versions.sh
    ./tools/activate_race_driver_version.sh <version>
