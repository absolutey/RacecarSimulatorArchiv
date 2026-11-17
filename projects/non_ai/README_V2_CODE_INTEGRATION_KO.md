# F1TENTH RAB — Code Integration V2 (2026-10-18)

**Status: CODE_INTEGRATION_CANDIDATE. The Ubuntu V1 baseline passed 16/16 ROS packages and 327/327 tests, but V2 was NOT ROS-built or run on the user's Ubuntu yet.**

## V2 full-source changes, not a standalone replacement snippet

1. **Guard ↔ Simulator atomic actuation lineage**: new `control_msgs/msg/GuardedActuatorCommand.msg`; Guard sends one typed `CERTIFIED` or `MINIMUM_RISK_BRAKE` envelope. Ego Simulator *only* subscribes to `/guarded_actuator_cmd0`, never raw `/ackermann_cmd0`. Publisher graph, epoch, producer, map SHA, source tick, time lease, speed/steering/accel limits and Intent replay are validated before refreshing the simulator command lease. `/ackermann_cmd0` remains read-only legacy telemetry. Simulator `actuator_applied_receipt0` now carries this exact command identity without a cross-topic join. This protects against accidental mixing, NOT adversarial DDS publisher forgery or OS scheduling failures.
2. **Actual Applied lineage qualification evidence**: both `qualify_solo.py` and `qualify_h2h.py` consume Simulator applied receipts and require at least one `CERTIFIED` applied tick and zero unmatched non-deadman applied ticks. Minimum-risk brake is never counted as a certified actuation. Fail-closed if evidence missing.
3. **H2H executable entry alignment**: `qualify_h2h.py` now requests `solo:=false`, MPC/SCP H2H experimental profile, Red dynamic Guard and long-horizon prediction. `--user-path` now calls `f1h2h`, *not* `f1sim` (SOLO). `f1h2h` whitelist prevents disabling safety flags. The experimental H2H startup is **not** a proven safe overtaking demonstration.
4. **R-A/S0-R high-speed execution proposals**: add periodic steering-rate and discrete jerk necessary bounds via uniform speed-profile scaling; preserves periodic curvature, acceleration, braking and stop-horizon restrictions. No arbitrary 5.0m/s cap. These are *discrete sampled necessary conditions*, not continuous-time steering/jerk or independent stop certificates.
5. **G0-G9 provisional evidence assessor**: strict source/run binding, both ICCAS/Spielberg receipts, SHA validation, metrics for timing/physical motion/faults, missing-evidence denial. Assessor **NEVER promotes** product authority. Official G0-G9 policy and approval remain unimplemented.
6. Explicit `solo:=true` in solo qualification and safe argument support in `f1solo`, refreshed stale non-ROS tests to check the new real H2H fail-closed interlock rather than an obsolete unconditional reject.

## Test and qualification evidence

- **V1 Ubuntu** (user-supplied evidence): 16/16 ROS packages finished; 327 tests, 0 errors/failures/skips.
- **V1 identical non-ROS container broad collection**: 205 PASS, 6 FAIL (outdated tests and fixture defects), 18 collection ERRORS (ROS/ament unavailable).
- **V2 non-ROS broad collection**: 248 PASS, 1 SKIP (ROS-only), 19 collection ERRORS (18 inherited missing ROS modules + 1 new ROS interface smoke test). No collected test failed.
- **V2 independent focused suite**: 71 PASS, 1 SKIP with `tests tests_integration src/verification/solo_champion/test_tournament_tools.py`; Python AST and shell syntax PASS.
- **NOT VERIFIED**: ROS-built new `.msg` wire type, actual Guard/Simulator integration, H2H runtime, 15ms and 7ms E2E, native C++ ROS Guard, independent watchdog/process, full two-map physical laps and real-world model uncertainty.

## Next definitive Ubuntu step

`bash RUN_V2_ROS_VERIFY.sh` in the freshly extracted isolated package root. Do NOT merge into `~/f1tenth_racing` and do NOT infer H2H qualification from colcon tests. The script only performs integrity, ROS build and tests; no vehicle actuation.

## 35 items

See `INTEGRATION_35_ITEM_STATUS.csv`. V2 closes a meaningful simulator-identity dataflow gap at source level, not all original 35 items. Guard 15ms end-to-end, GuardAhead 7ms, native Guard execution process, isolated watchdog, MPC/backup execution isolation, fully qualified Spielberg trajectory, physical H2H complete loop, official G0-G9 promotion remain open.
