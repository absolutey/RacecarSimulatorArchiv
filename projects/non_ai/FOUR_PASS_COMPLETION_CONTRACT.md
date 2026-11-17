# F1TENTH RAB — 4회 통합 완성 계약 (2026-10-18)

## 출처와 정직한 기준선
- 기준: 사용자 V7 네이티브 환경변수 수정 ZIP `F1TENTH_RAB_V7_NATIVE_ENV_HOTFIX_20261018.zip`의 소스와 `INTEGRATION_35_ITEM_STATUS.csv` (35건).
- V6 Ubuntu 공식 등록 회귀: 16 packages / 433 tests PASS (사용자 로그). V7 원본: 457 tests / 24 failures (그중 23개 테스트가 `APPEND_ENV` 경로 오염으로 실패; 나머지 1은 CTest wrapper 집계). Native hotfix: 선택 package / 23 pytest PASS, 24 등록 검사 0 FAIL (사용자 로그). **V7 전체 등록 회귀는 미확인**.
- 상태표는 아직 이전 버전의 `PATCHED_NOT_ROS_TESTED` 등 오래된 상태가 섞여 있음. 빌드 PASS와 제품 코드완성·실증 자격증명은 서로 다른 주장이다.
- 사용자 목표: 네 계층을 네 번 분할하는 게 아니라, **모든 단계에서 4계층 전 범위를 동시에 대상으로 다루는 4회 큰 통합 패스**.

## 1회차 — 4계층 전체 소스 완결(목표)
하나의 통합 소스에서 모든 실행 계약을 연결한다. 내부 구현 순서는 의존성 순서로 관리하지만 결과 제출은 네 계층 전체 소스, 전체 launch/config/msg/install/test 변경을 하나로 묶는다. 다음 구조를 통합한다.

1. **승인 권한을 먼저 닫는다**: C++ Guard 본체/ROS executor/심사/stop proof/Red dynamic check/watchdog/동일 tick/epoch/lease/Applied. Python Guard 실험 경로는 기본 권한이 될 수 없다. 승인 불능 시 즉시 fail-closed.
2. **R-B 계산 경쟁을 끊는다**: Primary MPC와 Fallback이 별도 실행 단위에서 동일한 state/ref revision을 참조하고 deadline 기반 중재. GuardAhead current/N+1 7ms 증명과 독립 Guard의 15ms E2E 측정·무승인 차단을 launch까지 연결.
3. **두 맵 고속 참조를 정본화한다**: ICCAS2025와 Spielberg의 SCP 경로+v(s), curvature/jerk/steering-rate/accel/braking/stopping-horizon/벽거리를 물리 예산으로 검증. 임의의 5.0/5.6/5.8m/s 캡 복원 금지. 맵별 정본 SHA 및 버전 승인.
4. **실제 H2H 폐루프를 연결한다**: 오직 Ego 1대·Red 1대. R-C 인지→R-D 예측→R-E 추월→S0-R nominal/override→R-B→Guard→Simulator→독립 evaluator. 계획이 깨지면 ABORT/제동/FOLLOW·이동 중 안전 재정렬/새 계획 재시도; 낡은 committed plan 재사용 금지. Red 또는 벽과 충돌 허용 안 함.
5. **검증·운영 경로를 코드로 일체화한다**: G0~G9 증거 수집·조건 판정, qualifers, `f1solo/f1safe/f1sim/f1h2h`, 메시지 정의, ROS CMake/package.xml, QoS, launch. G0~G9 공식 자격 승인 없이 자동 승격 불가. 미지원 B2 NMPC는 구현하거나 정식 범위 재결정 없이는 미완료로 유지.

1회차 출고 조건: 전체 35개 결측 각각에 실제 source commit/diff + 테스트를 매핑, 죽은 실행경로 0, 미구현 코드를 구현 완료로 표시한 항목 0. 단위 테스트 통과만으로 완료 판정 금지. **1회차를 코드 완료라 부르려면 35개 기능별 구현 정적 증거가 필요.**

## 2회차 — 모든 컴파일/실행 결함 수렴
- 1회차 후보를 **사용자 Ubuntu ROS2 Humble 격리 워크스페이스**에서 정적 포맷/타입, C++ compile/link, `colcon list/build/test/test-result`, ros2 pkg executable, launch preflight/ROS 메시지 serialize/QoS/real topic smoke 검증.
- 정상 PASS 패키지는 그대로; 실패한 패키지·관련 종속성만 `--packages-select` / `--packages-up-to` / `--packages-above`로 선택 빌드/테스트. 증거와 원인 보존, 테스트 개수 변화 원인 기록. blind rollback 금지.
- 출력·deadman·Guard heartbeat/epoch·참조 변경·NaN/Inf·시간 역행·Red dropout·actuator race·ABORT 상태전환을 코드 수준으로 전수 재검증.
- 추가 소스 패치가 필요하면 바로 근본 결함으로 수술하고 전체 회귀를 재실행; **실패 0, 수집오류 0, SKIP 0 및 unknown gate 0**만 2회차 종료 후보.

## 3회차 — 독립 교차검증
- 검증기가 구현 코드와 같은 결론을 독립적으로 계산하는지 확인. Python↔C++ stop proof parity, native/pursuit MPC fallback consistency, Guard Intent↔Applied/physical ground truth, reference SHA/identity, real time latency distribution.
- Fault injection: Python Guard kill, C++ Guard kill, watchdog kill, R-B stall, MPC deadline, Red prediction stale/ID swap, missing collision info, wrong map/hash, ROS domain isolation, epoch replay, malformed intent; 모든 unsafe applied 0.
- ICCAS2025·Spielberg 모두 동일 구성으로 수행, G0~G9의 독립 증거를 모은다. 재현용 seed/ROS_DOMAIN_ID/environment sha/log를 저장.

## 4회차 — 시뮬레이션 실증/최종 판정
- 두 맵 각각 Solo 고속 직선 가속/코너 감속, 물리 lap 검증, 5.0m/s cap 잔존 검토, 안전 제동, wall collision 0.
- 두 맵 각각 Ego·Red 정면 H2H 추월: FOLLOW→OPPORTUNITY→COMMIT→OVERTAKE→REJOIN→FOLLOW, Red 변화로 실패하면 ABORT→안전 재정렬→새 기회 재시도. Red collision 0.
- 주기 중단·시계 jitter·센서 dropout·프로세스 사망·예측불일치에 대한 fail-closed 검증, G4 15ms E2E/G5 7ms E2E 실측. 물리적으로 가능한 성능만 승인.
- 최종 산출: 소스 정본 ZIP + 소스 SHA + 빌드 회귀 + 35건 status(실제 증거 기반) + 양맵 실행 bag/JSON/oracle + G0~G9 verdict. 실증 실패 시 자격판정 FAIL 유지하며 실행 코드만 완료라 기록 가능.

## 각 회차가 *아닌 것*
- 1=Guard, 2=R-B, 3=R-A, 4=H2H **아님**.
- V8,V9,V10처럼 버전 번호만 늘리는 것이 목적 **아님**.
- 433/433 PASS 같은 단위 테스트 결과를 기능 실증으로 간주 **안 함**.
- 안전을 무력화하거나 과거 arbitrary 5.0m/s cap을 보존해 '통과'시키는 작업 **안 함**.

## 현 소스에서 확인된 코드 단절
- `src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/actuator_guard_node.py`: 여전히 Python Guard Node가 독립 actuator authority. `experimental_native_guard`는 Solo·simulation 조건에서만 허용되고 완전한 C++ ROS Guard Node는 없음.
- `src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/node.py`: `LatestOnlyControlWorker`라는 단일 Python worker 내에서 Fallback fresh proof와 MPC solve가 순차 처리.
- `src/driving/algorithms/f1tenth_racing_raceline/data/spielberg_centerline_preliminary_racing_trajectory.*`: Spielberg는 예비 centerline 기반; ICCAS에는 별도의 a0~a3 numeric assets 존재.
- `src/platform/f1tenth_racing_reference/f1tenth_racing_reference/abort_recovery_gate.py`: parked/nominal aligned 복귀를 판정하며 이동 중/off-track 재정렬 경로 생성기는 아님.
- `src/verification/qualification_gates/gate_model.py`: G0~G9는 provisional verdict checker; 공식 promotion authority 아님.

## 35건 세부 매트릭스
`RAB_35_GAP_FOUR_PASS_CLOSURE_MATRIX_20261018.csv`의 모든 행을 완료 증거로 닫아야 한다. status를 임의 PASS로 바꿔서는 안 된다.
