# F1TENTH RAB — V4 독립 watchdog 및 적용증거 통합

## 실제 확인된 검증 기준선
- V1: Ubuntu colcon 16개 / 327 PASS.
- V2: Ubuntu colcon 16개 / 328 PASS.
- **V3: 사용자 Ubuntu 로그, colcon 16개 / 338 PASS / 0 errors / 0 failures / 0 skipped.**
- 이전 세 기준선은 보존하며 과거 PASS를 V4의 PASS라고 주장하지 않는다.

## V4 실제 코드 범위
1. 새 별도 ROS 프로세스 `independent_guard_watchdog` (`independent_watchdog_node.py`)와 순수 상태기계 (`independent_watchdog_core.py`) 생성. Guard 상태, 최신 실제 physics state, collision을 독립 관측. Guard가 `RUNNING`, ready, certificate_verified이고 상태가 fresh, epoch/source 일치할 때만 ALLOW 메시지를 발행. 다른 조건은 REVOKE, 프로세스가 정지하면 simulator-side lease time-out.
2. `external_watchdog_lease.py`: epoch/producer/sequence/ROS+monotonic time, 상태 시퀀스, 기본 JSON schema를 검증. 최신 증거 부재/퇴행/위조 상태/future-state/safe-hold/revoked -> FAIL CLOSED. 이 자체로 정지 증명이나 구동 승인을 발행하지 않음.
3. `simulator.py`: Car0에서는 새 독립 watchdog 사용이 필수 (`EGO_EXTERNAL_WATCHDOG_MANDATORY`). 기존 V3 Guard status, V2 atomic `GuardedActuatorCommand` 검사 + 새 veto를 모두 통과해야 CERTIFIED 명령이 수락되고 다음 물리 tick에 유지됨. 매 tick watchdog 재검증, 상실시 로컬 deadman, Applied 영수증에 watchdog 상태/사유 기록. Emergency minimum-risk brake는 새 veto가 없어도 적용될 수 있게 유지함.
4. `race_foundation.launch.py`, `f1tenth_racing_control/setup.py`: 같은 epoch/source로 별도 프로세스 등록 및 실행. RELIABLE/VOLATILE 토픽에 대해 퍼블리셔/서브스크라이버 QoS 호환.
5. `AppliedLineageAudit(require_external_watchdog=True)`를 실제 Solo/H2H qualifier에 연결. Applied 결과에서 모든 CERTIFIED tick에 독립 watchdog의 fresh witness가 없으면 자격검증 거부. 기존 과거 fixture 검사는 옵션 기본 OFF로 계속 재현됨.
6. 신규 ROS 패키지 자동수집 테스트 **36개** + qualification integration 시험 7개, 관련 오프라인 테스트 86개 PASS. `INTEGRATION_35_ITEM_STATUS.csv`는 영향 받은 GAP-04/06/12/29만 업데이트.

## 완료하지 못한 코드/계약 (분명한 결측)
- Guard 단독 C++ ROS 실행기, 15ms 전체 경로 실시간 보증과 WCET, GuardAhead 7ms 보증, MPC/Fallback 별도 실행 프로세스, Spielberg 최적화 racing-line 자격화, H2H ABORT/REJOIN 완전 폐루프, 공식 G0–G9 승격체계 미완료.
- ROS namespace/node name 검사는 보수적 시뮬레이터 gate이지 암호학적 발신자 인증이 아니다; 실제 공격 모델은 DDS Security와 별도 qualification 필요.
- 온라인 watch process 재시작 후 safe-state 재진입, PID/CPU 고장 주입, 원격 actuator 적용 증거는 Ubuntu/SIL에서 추가 검사 필요.
- V4 ZIP에 대한 Ubuntu `colcon build/test`는 **미실행**. 338+36=374개 목표는 ROS 테스트 수집이 증명되기 전에는 추정일 뿐임.
- 정본 `~/f1tenth_racing`은 수정하지 않으며 압축을 별도의 격리 워크스페이스에 풀어 검증.

## 다음
`bash RUN_V4_ROS_VERIFY.sh`로 격리 워크스페이스 전체 빌드/회귀. 실패하면 원인 분류 후 해당 부위 수정, 기존 PASS 코드를 무조건 rollback 하지 않음. 실증 시험 없이 4계층 완료라고 주장하지 않음.
