# Racecar Simulator Archive

ROS2 racecar simulator and archived race driver control module versions.

This repository is designed for reproducibility:

- `src/race_driver` is the active current control package.
- `archive/race_driver_versions/` stores full historical race_driver packages.
- `tools/activate_race_driver_version.sh` can restore any archived version into `src/race_driver`.
- `VERSION_MANIFEST.tsv` explains each archived control module.

## After clone

Patch portable placeholders to your clone path:

    ./tools/patch_clone_paths.sh

List versions:

    ./tools/list_race_driver_versions.sh

Activate a version:

    ./tools/activate_race_driver_version.sh current_main
    ./tools/activate_race_driver_version.sh best_boundary_mpc_892_delta075
    ./tools/activate_race_driver_version.sh hysteresis_slow_20260502_105510

Basic check:

    ./tools/check_archive_integrity.sh

## Control archive policy

This repository intentionally keeps multiple race driver versions because the exact best-performing controller may depend on the simulator state, map, and test run. The current active module is preserved, but historical modules can also be restored at any time.

## Competition simulator immutability rule

For the original competition rule, simulator files must not be modified. In this archive:

- `src/racecar_simulator/` is preserved as simulator code.
- `src/control_msgs/` is preserved as dependency/interface code.
- `src/race_driver/` is the active control module.
- `archive/race_driver_versions/` stores control-module versions only.
- Tools are designed to replace or patch `src/race_driver` only.
- Do not edit `src/racecar_simulator` for controller tuning.
