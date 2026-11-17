# F1TENTH RAB — 4회 중 1차 실제 소스 후보 (2026-10-18)

## 중요: 출고 상태
- V7 native ENV hotfix 전체 소스를 그대로 포함한 **증분 소스 후보**이다. 35개 결측 전부가 완성된 것이 아니다.
- 현재 환경은 ROS2 Humble을 포함하지 않으므로 전체 ROS build/test 검증은 수행하지 못했다.
- 사용자 Ubuntu: V6 433/433 등록 PASS. V7 원본 457/24 fail(23 tests +1 CTest 집계); native 환경변수 hotfix 후 native 선택 ROS 23/23 PASS, 등록 24/24 PASS. **V7 전체 재검증 결과는 없다.**
- Guard 15ms/GuardAhead 7ms, 양맵 H2H 안전 및 운용 자격검증은 인증하지 않는다.

## 이번에 실제 수정한 코드
1. R-B `node.py`: worker의 오래된 physics state 확인과 ActuatorIntent ROS publish를 동일 receipt lock에서 실행해 TOCTOU race를 닫는다. 기존 Guard/Simulator 인증 권한은 변경하지 않는다.
2. R-E legacy `node.py`: ICCAS 하드코딩을 제거하고 launch의 환경파일·선택 맵과 SHA를 사용. 다른 맵을 잘못 선택한 참조는 거부.
3. R-E successor `successor_node.py`: 동일 검증된 양맵 로더 사용; 상태 timestamp/sequence 최신성, finite 검사; 무효 S0-R reference의 cached nominal plan 차단. 비활성 R-E 진행 상태 경로 투영 인자 검사.
4. R-E 공통 `runtime_environment.py`: 두 맵의 파일 바이트 SHA·source track ID·센터라인 SHA·finite geometry를 검사.
5. R-E 공통 `state_input_contract.py`: 단조 physics sequence/stamp, 정확한 나이 상한, nonfinite fail-closed.
6. ROS 패키지에 자동 수집되는 `test_runtime_environment_contract_v8.py`(16개), `test_state_input_contract_v8.py`(13개) 추가.

## 직접 수행한 검증
- offline pure-algorithm focused pytest: 63 PASS (includes the 29 new tests, V6 worker and GuardAhead tests).
- 두 설치 원본 ICCAS2025 66점, Spielberg 864점 map + centerline SHA 검증 PASS.
- source AST / ZIP CRC / versioned source and release manifests verified.
- ROS 의존 테스트는 이 환경의 `rclpy` 부재로 수집되지 않았다; Ubuntu에서 실행해야 한다.

## 실행
- Ubuntu Downloads에 ZIP 저장 → ZIP 압축해제 → `bash RUN_FIRSTPASS_ROS_VERIFY.sh`.
- 검증은 격리된 작업공간에만 적용하고 기존 `~/f1tenth_racing`은 수정하지 않는다.
- ROS에서 오류가 발생하면 패키지별로 원인을 추적·수정하고 기존 PASS를 보존한다.

## 남은 필수 코드: 1차 통합 완료로 표기할 수 없음
- C++ ROS Independent Guard 실행기 및 기본 경로, MPC/Fallback 독립 병렬 worker, 7ms/15ms E2E, Spielberg 정식 최적화
- H2H 이동 중 ABORT 회피·재정렬·재시도 전체 제어폐루프, B2 NMPC, G0~G9 공식 promotion 정책
- 위 항목의 구현 전까지 4계층 모두 완성으로 주장하지 않는다.
