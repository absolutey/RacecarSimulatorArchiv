# F1TENTH RAB 3회차 코드 전수감사 — 2026-10-19

## 감사 대상·판정 기준

- **입력 정본:** `F1TENTH_RAB_PASS2_TWO_TEST_FIX_FULL_SOURCE_20261019.zip`. 사용자 Ubuntu 결과는 **17 패키지 빌드 / 686 등록 테스트, 오류 0·실패 0·SKIP 0**이다. 이 수치는 PASS3 신규 소스의 Ubuntu 재검증 결과가 아니다.
- **범위:** 17개 ROS2 패키지의 파일 목록·패키지 정의·Python 구문 및 주요 안전/실행 경로, 4개 양맵 Solo/H2H 구성, Guard·GuardAhead·MPC·R-A3·Red ABORT·G0~G9의 정적 계약.
- **미실행:** ROS Humble native C++ 신규 선택 빌드, 프로세스 장애 주입, 실시간 최악지연/15ms·7ms 측정, Solo/H2H 물리 주행. 사용자의 요청대로 실증은 이번 회차 제외.
- **구분:** `PASS_STATIC`은 파싱·명시적 코드 연결·선택 오프라인 테스트 통과만 뜻한다. 전 경로 구현 완료, 런타임 PASS, 기능 안전·WCET 인증이라는 뜻이 아니다.

## 새로 확정한 원인과 수정 (P0 코드 연결 오류)

**G01 — C++ Guard 선택 시 최종 액추에이터 승인이 구조적으로 차단됨**

- 원본 `src/platform/f1tenth_racing_native_proof/src/native_guard_ros.cpp`는 `Node("native_actuator_guard0")`로 생성하며, `src/platform/f1tenth_racing_bringup/launch/race_foundation.launch.py`도 `name="native_actuator_guard0"`였다.
- 그러나 `src/environment/f1tenth_racing_sim/f1tenth_racing_sim/simulator.py`의 `receive_guard_status`와 `_command_guarded_locked`, `src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/independent_watchdog_node.py`의 `on_guard_status`, `src/platform/f1tenth_racing_bringup/scripts/race_coordinator.py`의 `guard_reason`은 **발행자 노드명이 `actuator_guard0`인 경우에만** 승인한다.
- **패치:** C++ 노드 내부 이름과 launch 선택 시 이름을 모두 `actuator_guard0`으로 통일. Python Guard는 `UnlessCondition`, C++ Guard는 `IfCondition`으로 **동시에 활성화되지 않으므로** 단일 권한 조건은 그대로 보존했다. C++ Guard는 Solo 전용이며 H2H 선택 금지 유지. Simulator·독립 watchdog·Race Coordinator의 단일 발행자 검사 규칙을 완화하지 않았다.
- **검증:** 새 identity 계약 pytest 3개 + 기존 Humble API pytest 4개 = 7/7 오프라인 PASS. ROS Humble C++ 재컴파일과 실행 증거는 별도 필요.

## 8개 구현 영역별 정적 감사

| 영역 | 실제 코드 연결 점검 | 판정 및 경계 |
|---|---|---|
| ① GuardAhead 7ms | 7ms 한계, 현재/다음 tick Native dual-stop, 초과 시 UNPROVEN | 구조 PASS; 최근 Solo 186/1221 초과이므로 **시간성 미완성** |
| ② 독립 Guard / MPC 격리 | C++ Guard 단독 선택, Spawn 프로세스·초기화 확인, 단일 최종 발행자 | Guard 노드명 P0 **수정**; 실행 중 원활한 사용 미검증 |
| ③ Guard 15ms / Applied | C++ 15ms 증명 예산, 최종 lease 재검사, simulator GuardStatus+외부 watchdog VETO | 구조 PASS; 물리 최악시간과 제동 실증 미검증 |
| ④ Spielberg R-A3 | 4개 후보 독립 정책 ID, SCP+true-joint 등록, 지도 SHA 정책 | 등록 PASS; true-joint는 **미승격 실험 후보**, 수렴최적성·실주행 미판정 |
| ⑤ 고속 MPC | R-B state QoS BEST_EFFORT/depth1·75ms, LTV/NMPC, 별도 solver 프로세스 opt-in | 구조 PASS; 독립 프로세스 timeout 복구 제한 및 고속 동역학 미검증 |
| ⑥ H2H | park/rejoin 안전 게이트, ABORT 후 새 Red 시각 fence, 계획신원 재시도 | 구조 PASS; 이동 중 실패/복귀/성공 E2E 미검증 |
| ⑦ 양맵 실행 | ICCAS/Spielberg × Solo/H2H 네 profiles 및 개별 rosbag, HUD, 사전 검사 | dry-run PASS; 실제 launch·evidence capture 미수행 |
| ⑧ G0–G9 | 원본 SHA·동일 run/source 바인딩, G8 ABORT + 신규 예측 + 신규 Plan ID 조건 | 정적 검증 PASS; provisional만 생성하며 **공식 승격 불가** |

## 중요한 잔여 코드·설계 리스크 (자동 PASS 불가)

1. **P1 / MPC 격리 복구 제한:** `process_primary_worker.py`의 `take()`는 시간초과 시 `_active_sequence`를 계속 점유한다. 이후 `submit_request()`는 영구 거부될 수 있다. 이는 **안전한 Fallback이 유지되는 fail-closed 동작**이지만 Primary 서비스의 자동 복구 폐루프가 아니다. 타임아웃된 solver가 내부 warm-start를 바꿀 수 있으므로 단순히 `_active_sequence=-1`로 풀면 안 된다. 프로세스 폐기·새 solver 사전기동·이전 결과 봉쇄를 구현한 뒤 독립 회귀가 필요하다.
2. **P1 / GuardAhead 7ms:** 기존 실증 186/1221 초과. Native dual-stop 코드는 선택 경로이며 실행 시 7ms를 만족한다는 증거가 없다. 요구조건 미완료로 기록한다.
3. **P1 / C++ Guard의 Python Guard 동등성:** 두 경로는 완전히 같은 구현이 아니다. Solo용 선택 경로에는 동적 Red 증명이 없으며 H2H가 의도적으로 금지된다. 소스 정합성·API PASS와 런타임 fail-closed 증명은 별개이다.
4. **P1 / H2H 복구:** 현 코드는 ABORT 후 stationary S0-R 복구와 새 Red 관측을 강제한다. 주행 중 자동 hard-latch 해제는 의도적으로 금지되어 있다. 실제 ABORT→FOLLOW→새 예측→새 계획→추월을 종단까지 검증해야 완성이다.
5. **P1 / G0–G9 공식 증거:** 현재 평가기는 원본 증거 없는 PASS를 거부한다. 코드만으로 공식 승격할 수 없으며 이는 안전한 설계이다.
6. **P2 / 운영 최적화:** 새 C++ Guard와 MPC 프로세스·Native GuardAhead는 명시 선택 설정으로 유지된다. 전부 기본값으로 활성화되어 제품 기능이 완성된 것처럼 주장하지 않는다.

## 검증 현황 및 후속 Ubuntu 단계

- Python/JSON/YAML 원본과 전체 17개 `package.xml`을 구문 분석했다. 결과 수는 기계 검증 스크립트 `PASS3_OFFLINE_SOURCE_AUDIT.py`의 출력으로 보존한다.
- 정적 20개 구조 계약 PASS; 선택 Native/Humble/권한동등성 7/7 PASS; Solo/H2H 네 구성 `DRY_RUN_ONLY` PASS; 기존 P0 보존 7/7 PASS.
- **Ubuntu에서는 이미 PASS한 나머지 패키지의 소스를 재수정하거나 빌드할 필요가 없다.** `f1tenth_racing_native_proof`와 `f1tenth_racing_bringup`만 `--packages-select`로 재빌드·테스트 → 마지막에 17개 등록 테스트 회귀를 수행한다. 등록 테스트는 신규 3개를 포함하여 최소 689개를 예상하지만, 실제 개수는 colcon 결과로만 확정한다.
- 신규 빌드·회귀 결과가 PASS해도 **코드 최종 동결은 P1 복구 정책 및 C++ Guard 실행 경로의 계약 결정 전까지 유보**한다. 실증은 이후 별도 단계에서 시행한다.

**3회차 결론:** **686 PASS 기반 전체 소스에서 확인된 구체적인 C++ Guard 신원 결함 1건을 패치했고, 20개 구조 계약을 재검증했다.** 8개 영역의 최종 제품 완성을 증명하지는 않았으며, 남은 복구·실시간·H2H 문제를 미완료로 명확히 분리했다.
