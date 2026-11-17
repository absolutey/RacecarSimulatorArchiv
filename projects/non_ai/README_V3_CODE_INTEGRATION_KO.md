# F1TENTH RAB — V3 안전 실행경로 통합 (2026-10-18)

## 검증된 이전 기준선
- V1 Ubuntu: ROS 16 packages / 327 tests PASS, 0 errors, 0 failures.
- **V2 Ubuntu 사용자 실제 로그: ROS 16 packages / 328 tests PASS, 0 errors, 0 failures, 0 skipped.**
- V2 ROS 빌드 성공이 H2H 완주/Guard 15ms/GuardAhead 7ms의 증거라는 뜻은 아님.

## V3 실제 추가·수정 코드
1. `guard_status_lease.py`: Simulator process is independent of Guard ROS process; verify status epoch, source, monotonically increasing timestamp/sequence, local ROS and steady clock freshness, `RUNNING`, `ready`, and `certificate_verified`. Never infer stop proof from status alone.
2. `simulator.py`: A CERTIFIED command is admitted only with independently received, live Guard heartbeat, on top of V2 atomic GuardedActuatorCommand identity/proof/replay check. Any non-running, stale, invalid Guard status revokes previous CERTIFIED command lease, triggering existing simulator-local deadman deceleration at next physics tick. Minimum-risk brake is not gated by running status. Physics step also rechecks live status so Guard process death does not leave a previous actuator lease active until full command timeout. Threading RLock serializes status, command, reset, and physics callbacks, protecting command/receipt consistency under MultiThreadedExecutor.
3. `test_guard_status_lease_v3.py`: 10 pure unit tests, placed under real ROS sim package test tree so colcon should discover them. Covers no-status denial, clock delay/regression, Guard SAFE_HOLD, identities, monotonic rollback, hard fault, and structural wiring.
4. `INTEGRATION_35_ITEM_STATUS.csv`: precisely statuses GAP-04/06/12 only. **No other GAP is closed or officially qualified by V3.**

## V3 evidence and constraints
- V3 nonROS focused suite: 69 PASS, 1 SKIP (ROS-only), AST PASS. The 10 new tests are included within 69.
- V3 Ubuntu ROS build/full test NOT executed here; expected 328 baseline + 10 new discovered tests = 338, subject to real discovery.
- This is **simulator-internal independent enforcement**, NOT an independent C++ Guard executable, real-time watchdog process, cryptographic DDS authentication, 15ms/7ms scheduling guarantee, physical stop certificate, or H2H qualification.
- No change to `~/f1tenth_racing`, no automatic merge or live vehicle commands.

## Next
Run `bash RUN_V3_ROS_VERIFY.sh` inside an extracted isolated V3 root, with ROS 2 Humble. Requires 338/338 registered tests PASS. Do not claim full 4-layer completion. Then continue native Guard execution isolation, MPC/Fallback isolation, complete H2H abort/rejoin with independent Red proof, dual-map numerical racing policy qualification, and official G0–G9 gates. Failure of any ROS test must be classified by root cause; do not mass rollback.
