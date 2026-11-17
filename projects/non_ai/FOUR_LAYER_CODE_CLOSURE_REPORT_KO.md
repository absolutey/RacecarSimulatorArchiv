# F1TENTH RAB — 698 PASS 격리 개발본 4계층 코드 완결성 재감사

작성: 2026-10-19. **구형 `~/f1tenth_racing`과 병합하지 않은 독립 후보**.

## 결론 / 판정 범위
- 출발 기준선: 사용자 Ubuntu 로그 `17개 패키지, 698 tests, 0 errors, 0 failures, 0 skipped` (신규 패치 전).
- 이번 후보: ROS 소스 실파일을 직접 조사하고 **확정된 결함 3개 부류**를 보완함. Guard 입력계약/시간 감시, MPC 송수신 IPC 실패 후 복구.
- 검사: 17개 패키지, 구조·호출 검사 29개 PASS, 오프라인 연계 테스트 20 PASS, Native C++ Guard 회귀 9 PASS (검사 집합 일부 중복).
- **본 수정본의 17개 ROS 패키지 Ubuntu 빌드·등록 회귀는 아직 미실행.** 기대 기준선은 최소 706 테스트, 오류·실패·SKIP 0. 실증 미실행.
- 등록 코드 회귀가 모두 PASS여도 고속 안전 주행, 실시간 worst-case 또는 공식 자격증명으로 간주하지 않음.

## 이번 실제 코드 수정
| 경로 | 근본 원인 | 변경된 동작 | 테스트 |
|---|---|---|---|
| `native_guard_ros.cpp` + `native_guard_intent_bounds.hpp` | 기존 C++ Guard가 조향각/가감속 절대 물리 한계와 100ms Intent 임대 상한을 직접 검사하지 않고 인증 가능 | 의심 메시지에 정지증명 수행하기 전에 Guard 자체가 차단; Fail-closed | C++ 실제 컴파일/실행 경계 검사 + ROS 소스 연결 |
| `native_guard_ros.cpp` | 25ms wall timer가 같은 ROS `/clock` 샘플 2번 읽으면 무조건 영구 고장 | 시각 **역행은 즉시 거부**, 일시적 동일 시각은 대기, ROS 시각 75ms 초과 정체 시 차단 | 구조 조건 재현 검사; 완전한 ROS 동작은 실증 대기 |
| `process_primary_worker.py` | MPC 프로세스의 `is_alive()` 후 Queue write/read가 깨지면 callback 예외나 영구 점유 가능 | 송신 실패는 즉시 Primary 거절, 수신 실패는 예외 + 프로세스 격리 교체 예약. 기존 late result는 거부 | 송신/수신 OSError·EOFError·ValueError 6개 재현 |
| `CMakeLists.txt` + 새 2개 테스트 | 기존 회귀가 위 코드 경계 검사를 포함하지 않음 | Native C++ 계약 검사 suite 2개 CTest 등록 | 등록 Ubuntu 검증 필요 |

## 4개 계층별 실제 수준

### ① Independent Guard / Safety
- 현존: Python Guard(기본), 별도 C++ ROS Guard(Solo 선택), 독립 Watchdog, Simulator의 GuardedActuatorCommand 최종 인가, 벽/차량 충돌 및 정지 증명.
- 이번 수정: C++ Guard 물리 명령/임대 자체 거부, 동기식 watchdog 위양성 완화. **한 번이라도 역행하면 hard latch / 장기 시각 정체 시 hard latch**.
- 미확정: C++와 Python Guard 전체 행동 등가성, 동적 Red 증명은 C++에서 미구현(`H2H` 금지), 15ms worst-case, 프로세스 사망 시 물리 제동 실증.

### ② R-B / GuardAhead
- 현존: BEST_EFFORT latest-only 75ms CarState QoS, fallback-first, LTV/NMPC 후보, 독립 MPC solver process, Primary/Fallback 게이트, 7ms 2-tick GuardAhead.
- 이번 수정: IPC 파이프 실패는 fallback + 비동기 recycle로 연결. CPU/ROS scheduling 7ms은 **미입증**.
- 미확정: 유효 Primary가 얼마나 자주 채택되는지, 7ms 이전 186/1221 overrun의 실제 해결 여부, `v(s)` 실차 고속 추종.

### ③ R-A / S0-R
- 현존: ICCAS2025/Spielberg map identity·centerline 검증, R-A0/A1/A2와 Spielberg R-A3 864-point SCP·true-joint 산출물, reference source/override 상태 머신, 4개 실행 profile.
- 이번 수정: 이 계층 운영 파일 **변경 없음** (이미 698 테스트 통과한 코드 유지).
- 미확정: SCP true-joint 실험 후보의 최적성/완전 수렴, 동역학/타이어 제약하 고속 추종, 두 지도 runtime.

### ④ R-C/R-D/R-E H2H + G0–G9
- 현존: Red 감지·예측, committed overtaking, 예측 이탈 시 ABORT, Fresh Red epoch 및 parked nominal restart 게이트, 새 plan identity retry, G8 증거 조건.
- 이번 수정: 이 계층 운영 파일 **변경 없음** (698 테스트 통과한 코드 유지).
- 미확정: H2H 폐루프 및 Red 충돌 회피, 시간 동기화, G0–G9 실제 증거 승격. G8은 단일 추월로 통과 불가.

## 시험계획 / 유지 조건
1. 격리 Ubuntu 워크스페이스에서 먼저 `f1tenth_racing_native_proof`, `f1tenth_racing_control` **2개만 선택 빌드·테스트**.
2. `test_native_guard_bounds_fourlayer`, `test_process_primary_channel_fourlayer`가 **실제로 수집·실행됐는지** 확인. 누락이면 전체 회귀 PASS 판정 금지.
3. 나머지 15개 소스/빌드는 보존한 채 전체 17개 `colcon test` 1회 실행. **최소 706 PASS, 실패 0, 에러 0, skipped 0**.
4. `VERIFY_RELEASE.py`로 본 통합 후보 SHA/추가 파일 검증.
5. 시뮬레이터 ROS2 실증, Guard 15ms, GuardAhead 7ms, 양맵 고속 MPC, H2H, G0–G9는 모두 별도 실증/정식 증거 단계에서 검증. 성공 전 완료 표시 금지.

## 유지한 원칙
- `~/f1tenth_racing` 정본과 아카이브는 접근·변경하지 않음. 구형 40개 충돌 병합 진행하지 않음.
- 임의 고정 5.0m/s 실행 cap 재도입하지 않음. 물리 한계 및 fail-closed 유지.
- 코드 오류 검사와 물리적 실증 자격을 분리. 698 PASS는 **이전 기준선 증거**이며 이번 새 소스에 승계하지 않음.
