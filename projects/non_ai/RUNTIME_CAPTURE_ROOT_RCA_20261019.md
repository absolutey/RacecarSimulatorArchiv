# Runtime capture provenance / cleanup RCA (2026-10-19)

## Basis
The preceding integrated source completed 17 ROS packages, 587 registered tests, 0 failures in the user's Ubuntu log. This does **not** establish run-time qualification.

## Two confirmed executable shell faults
1. RUN_INTEGRATED_SOLO_CAPTURE.sh hard-coded `~/Downloads/F1TENTH_RAB_UNIFIED_CONSOLIDATED_IMPLEMENTATION_20261018`, while current qualified ZIP was verified in a nested isolated workspace. Running the recorder would silently target a different overlay or fail before any ROS launch. The script now derives the stage root from its own physical directory and still asserts every ROS package prefix belongs to its stage.
2. On PREFLIGHT_ARTIFACTS failure, it invoked `stop_launch` before that function was defined and before a launch process existed. Under `set -e`, this could abort evidence ZIP creation. That invalid call is removed; an early preflight failure packages its own evidence and exits before ROS launch.

## Tests and safety contract
- 5 launcher static/CLI checks register with f1tenth_racing_bringup.
- Existing default Python Guard / watchdog / actuation authority, speed profile and ROS message contracts are unchanged.
- No hard real-time, H2H E2E or successful live ROS capture is claimed.
- Any ROS qualification requires fresh Ubuntu execution of this modified package and capture. Full registered test gate now expects at least 592 tests (587 previous + 5 new).

## Correct Ubuntu command (isolated extraction)
```bash
source /opt/ros/humble/setup.bash
source ./install/setup.bash
bash ./RUN_INTEGRATED_SOLO_CAPTURE.sh
```
The launcher writes a dated evidence ZIP under `~/Downloads`; START is requested only after source/map and live preflight.
