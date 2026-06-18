# Racecar control version selection note

This repository is intentionally archived as a multi-version control module repository.

The current active package is:

- src/race_driver

The strongest evidence from local diff reports indicates:

- current_main is based on best_boundary_mpc_892_delta075.
- The active current version keeps the same configuration family.
- The current source differs from best_boundary_mpc_892_delta075 only by a final controller validity fix: cmd.valid = true.

However, the project is preserved as an archive because the exact best driving result version may depend on the test run, map, and simulator state at the time.

Use:

    ./tools/list_race_driver_versions.sh
    ./tools/activate_race_driver_version.sh best_boundary_mpc_892_delta075
    ./tools/activate_race_driver_version.sh hysteresis_slow_20260502_105510
    ./tools/activate_race_driver_version.sh current_main

Activation replaces src/race_driver with the selected archived package and patches old __RACECAR_ARCHIVE_ROOT__ paths to the current clone root.

## Competition safety note

The simulator package must be treated as immutable. Version switching and path patching should target `src/race_driver` only. Do not modify `src/racecar_simulator` for control tuning.
