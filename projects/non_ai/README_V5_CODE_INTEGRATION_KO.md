# F1TENTH RAB — V5 계획된 ABORT 이후 주행 재개 계약

## 확정 Ubuntu 기준선
- V1 327 / V2 328 / V3 338 / **V4 374** ROS 등록 테스트, 전부 errors/failures/skipped 0 (사용자 Ubuntu 제출 로그).
- V4 Guard watchdog 실행 항목은 패키지에 설치됨. C++ 독립 Guard, 15ms/7ms 최악실행시간, H2H 런타임은 여전히 미검증.

## V5에서 실제 작성한 코드
1. `f1tenth_racing_reference/abort_recovery_gate.py`: 현재 차량이 물리적으로 정지했고, 레이싱라인과 위치·방향이 일치하며, Ego/Red 충돌 토픽이 알려졌고 충돌 래치가 없으며, Red 예측이 최신 TRACKED·신원유효·연속 2.0초 이상의 점유 예측에서 이격됐을 때만 nominal 재진입의 *필요조건*을 충족시킨다. 물리적 주행 허가가 아님.
2. S0-R: 인정된 ABORT 플랜이 만료되면 즉시 invalid active reference를 발행한다. 자동으로 과거 추월 경로를 재활용하지 않는다. 각기 다른 신선한 Ego 상태가 3회 도착하고 위 조건이 연속 충족돼야 NOMINAL을 다시 발행한다. 충돌 True는 이어지는 False 메시지로 해제되지 않는다. 복귀 신호에는 기존 plan ID·revision 결속.
3. R-E: ABORT 후 플래너를 무조건 리셋하던 경로 없음. S0-R의 동일 plan ID/revision 회복 메시지를 받은 후에만 ABORT 래치를 해제한다. 기존 추월 계획을 폐기하고 R-D 새 예측으로 탐색한다.
4. Independent Guard: 계획된 H2H ABORT 레퍼런스 만료시 이전 actuation lease를 즉시 취소하고 MINIMUM_RISK_BRAKE/Simulator deadman을 유지한다. 예외적으로 **충돌/하드래치가 없는** 계획된 ABORT만, 이후 충분한 새로운 정지 CarState와 S0-R의 검증된 NOMINAL을 받은 뒤 기존 arm() + 새로운 정지증명으로 재진입할 수 있다. 이미 발생한 위험한 Guard 인증 실패·충돌 hard-latch를 해제하지 않는다.
5. Launch: S0-R이 Control·Guard와 정확히 동일한 environment_file을 사용하도록 연결.

## 수행한 검사
- V5 순수 ABORT 복귀 게이트 pytest 24 PASS.
- R-E successor 5개 회귀 모듈 pytest 46 PASS (그중 기존 Q0 23 → 25).
- 순수 오프라인 결합 70 PASS. 새 ROS 어댑터 계약 테스트 5개는 Ubuntu ROS에서만 실행 가능.
- 이 환경은 ROS Humble/rclpy 메시지 코드를 실행할 수 없음. V5 Ubuntu build/colcon/H2H SIL **실행 안 함**.

## 알려진 제한 / 다음 코드 결측
- ABORT 후 Ego가 이미 레이싱라인에서 이탈한 경우에는 이번 V5가 자동 복귀 경로를 만들지 않는다. 안전이 증명되지 않으면 정지 대기한다.
- Guard가 충돌 또는 별개 위험으로 이미 `emergency_unrecoverable` 하드래치된 상황은 회복할 수 없다. 같은 run 안에서 모든 ABORT 시나리오를 성공 처리한다는 주장은 금지.
- S0-R 새 NOMINAL 자체는 인증된 실행 명령이 아니다. Guard 물리·벽·Red stop proof, watchdog lease, Simulator가 각 틱마다 별도 승인해야 한다.
- 15ms/7ms WCET, 독립 C++ Guard, R-B MPC/Fallback 독립 Worker, Spielberg 고속 racingline 정식 자격화, 동적 OFFTRACK→REJOIN, G0~G9 최종 승격은 아직 구현/검증 미완료.
- S0-R에서 사용하는 ROS 토픽 ID/계획 신원은 암호학적 발신 인증이 아니다. DDS 공격 모델은 별도.

## Ubuntu 재검증
`bash RUN_V5_ROS_VERIFY.sh` — V4 374 PASS에 새 31개 ROS 수집 테스트를 더한 **최소 405개** PASS 기준. 테스트 수가 다르면 소스 수술/계약변경/테스트 수집 문제를 먼저 분석할 것. 등록 테스트 PASS는 E2E 실증 증명 아님.
