# F1TENTH RAB — P0 Execution Path Repair (2026-10-18)

## 위치 / 용도

Parent: `F1TENTH_RAB_P0_TICK_SEMANTICS_REPAIR_20261018` (same source baseline; untouched).
New immutable isolated candidate: `F1TENTH_RAB_P0_EXECUTION_PATH_REPAIR_20261018`.
Ubuntu ROS runtime / 15ms+7ms deadline compliance / full lap are **NOT YET QUALIFIED**.

## 확정된 실패 증거

Latest old Tick candidate, ICCAS2025 Solo Intent #508: Guard certified 507 previous intents,
then `STOP_CERTIFICATE_NOT_VERIFIED` because measured 17.355ms wall vs 15ms deadline.
Thread CPU 3.789ms; wall minus CPU 13.566ms (NOT proven to all be preemption).
R-B GuardAhead exceeded 7ms 145 times / 1310. No physical lap, 4.266m/s peak.

## 실제 수정 (6개 관리 소스 파일)

1. `actuator_guard_node.py`: hard brake output and SAFE_HOLD authority revocation occur BEFORE synchronous first-brake log; watchdog error log after its brake. 15ms post-proof wall condition and H2H hard interlock preserved. Guard uses bounded stop proof and STILL independently decides.
2. `stopping_envelope.py`: exact physical dynamics, swept obstacles, jerk, reaction time and stop completion unchanged. An optional `evaluate_bounded()` checks a monotonic wall deadline once per four rollout steps and returns `qualified=False, reason=PROOF_BUDGET_UNPROVEN` upon exhaustion. Normal `evaluate()` unmodified in decision semantics. One preceding-pose object reused across steps to avoid allocating a full State per tick.
3. `guard_ahead.py`: the SAME current-state and N+1 proof run with bounded deadline where supported. No candidate is granted authority without both full certificates. Timeout => UNPROVEN => fail closed; 7ms remains.
4. `map.py` + `swept_collision.py`: cache immutable map origin rotation trigonometry; raster occupancy/SAT and all thresholds unchanged.
5. `fast_stop_proof.py`: maintain current shared StopProof and swept collision SHA pin; it is NOT an old copied evaluator. Invalid source remains fatal at startup.

NO modification to R-A path, R-B long/lateral control formula, execution speed cap policy, MPC activation, reference identity, physical dynamics or Simulator deadman. NO permission to run H2H.

## Tests actually executed in this container

- `TEST_STOP_PROOF_EQUIVALENCE.py` 1,356 case outcomes unchanged (model, geometry and map, compared with immutable old oracle).
- `TEST_INTEGRAL_MAP.py` 4,500 rotated/flipped rectangle predicates unchanged.
- `TEST_GUARD_TICK_BINDING.py` 14/14.
- `TEST_P0_ARCHITECTURE.py` 7/7.
- ROS-free Controller GuardAhead / FinalPreflight pytest subset 24/24 (PYTHONPATH from package setup.py parent directories).
- `TEST_P0_EXECUTION_PATH.py` 8/8: prior incident #508 full stop preserved, expired and mid-rollout budget cannot pass, first emergency brake published before log, first cause retained, stale state lets deadman take over.
- Microbenchmark `FIRST_STOP_SNAPSHOT_508`: 350 unbounded runs on container; previous median 625.648µs, updated 468.758µs; NOT a ROS WCET qualification; see `BENCHMARK_COMPARISON.json`.

## Ubuntu (do not delete / overwrite current or canonical workspaces)

Ensure the ZIP is in `~/Downloads`, verify its printed SHA256, then:

```bash
cd ~/Downloads
ZIP=F1TENTH_RAB_P0_EXECUTION_PATH_REPAIR_20261018.zip
DIR=F1TENTH_RAB_P0_EXECUTION_PATH_REPAIR_20261018
# Verify SHA256 by matching the standalone *.zip.sha256 file or provided digest.
test ! -e "$DIR" && unzip -q "$ZIP"
cd "$DIR"
python3 -B VERIFY_RELEASE.py
python3 -B TEST_P0_EXECUTION_PATH.py
python3 -B TEST_P0_ARCHITECTURE.py
bash BUILD_FORENSIC.sh
bash RUN_INTEGRATED_SOLO_CAPTURE.sh
```

Produces `~/Downloads/F1TENTH_RAB_P0_EXEC_SOLO_*.zip` for first-brake, deadlines, actual speed, distance and collision/lap review. **STOP at failed gate; do not apply to Canonical/NEXT.**

## What still is NOT fixed

- No guarantee of OS scheduling/preemption bounded to 15ms: if CPU/wall budget exceeded, Guard MUST fail closed.
- A slow R-B algorithm and its placement in the ROS subscription callback are not yet structurally split (requires system-level ROS scheduling proof).
- GuardAhead might fail more conservatively on deadline expiration; this is intentional and must be quantified in real Solo.
- Primary MPC adoption and R-A high-speed v(s) closure not qualified.
- Physical lap, Spielberg, H2H not qualified.

## Evidence policy

Do not infer a "PASS" from 327/327 unit regressions or a START service acceptance. Report the FIRST Guard rejection and full Applied/Ego samples; leave full-speed qualifications pending until continuous physical lap and deadline tail latency are evidenced.
