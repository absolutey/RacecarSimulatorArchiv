# F1TENTH RAB / Solo capture prelaunch RCA (2026-10-19)

## 2026-10-18T16:53 Ubuntu evidence
- 17 ROS packages, 599 registered tests, 0 errors, 0 failures, 0 skipped (USER-SUPPLIED Ubuntu log).
- Immediately after ROS regression, `RUN_INTEGRATED_SOLO_CAPTURE.sh` stopped **before ROS launch** at `TEST_UNIFIED_EXECUTION.py`: 5 missing `_state_receipt_lock` in the outdated stub; 1 obsolete `H2H_STOP_ENVELOPE_DYNAMIC_OCCUPANCY_NOT_QUALIFIED` string assertion.
- The remaining Solo preflight scripts also had stale Guard/H2H assertions, missing fake-message publishers, and absent `solo`/native attributes in ROS-free test stubs. Tests had not been registered in `colcon test`.
- **Real runtime issue found:** `fast_stop_proof.py` pinned `b6a5bddf...` while the executing `StoppingEnvelope` in this release has SHA256 `13f6ed05...`. The source identity check would reject the normal controller-side stopping envelope before vehicle startup. This is a source provenance repair, not weakening a guard.

## Changes
- Reflect R-B `_state_receipt_lock`, `_control_lock` and non-worker execution in isolated test Stub; assert latest-only behavior with the actual callback.
- Replace retired H2H unconditional rejection marker with checks of the current independent dynamic Red witness (fresh evidence, first swept segment, continuous rollout occupancy checker, missing checker denial) and unchanged 15ms/7ms deadlines.
- Test **atomic GuardedActuatorCommand publication**, not merely the telemetry Ackermann mirror; provide correctly shaped mock messages.
- Use actual `finite_physics_state` in brake tests and a **complete finite state**; do not disable invalid-state blocking.
- Update `TEST_P0_ARCHITECTURE.py` to validate the integrated Solo MPC/SCP launch defaults and current H2H interlock rather than obsolete V0 defaults.
- Re-pin controller-side stopping proof to the exact verified source bytes (`13f6ed05446ed4438076345a0db37f819472c7ca16f38041c3d71ffbab4fb151`) and keep the sweep source pin unchanged.
- Register all nine ROS-independent prelaunch scripts plus an explicit dual-source-pin test as `f1tenth_racing_bringup` CTest/pytest; new regressions run **during colcon test**, before runtime capture.
- Add `TEST_P0_ARCHITECTURE.py` to actual Solo prelaunch path and an offline preflight CLI.

## Status and boundaries
- Local offline complete: 9 scripts + P0 architecture and provenance pin test pass. No local ROS Humble runtime available.
- Last **Ubuntu** result `599/599` belongs to the prior ZIP, not the modified source.
- New ZIP requires **>=609** ROS registered tests, no errors/failures/skips, then Solo capture.
- No 7ms/15ms hard real-time, H2H, Spielberg, vehicle-safety, or lap qualification is asserted.
