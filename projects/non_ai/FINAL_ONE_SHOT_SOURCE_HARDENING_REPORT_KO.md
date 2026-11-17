# F1TENTH RAB — 단일 통합 수술 코드 후보 (2026-10-19)

## 출발 기준선
Ubuntu ROS 17 packages, 535/535 registered tests PASS (사용자 제출 로그의 **이전** 정본).
이 후보는 변경된 소스이므로 이전 PASS를 자체 PASS로 재사용하지 않음.

## 실질적인 새 구현
- **R-B B2 NMPC**: `NonlinearDynamicBicycleMpc`, nonlinear vehicle dynamics full horizon, SLSQP, hard finite/state/lateral/slip/accel/rate constraints, explicit `mpc_formulation:=nmpc` launch path, isolated primary worker, deadline error -> fallback; base LTV unchanged.
- **R-B**: 만료된 Primary의 `warm_start` 오염 방지, R-A 속도 프로파일의 비정상 입력 선제 거부.
- **R-E**: Red 신뢰도 NaN/Inf·범위 초과, 미래 소스 시각, 시각 비유한·유효하지 않은 provenance 선제 거부.
- **Guard**: 전체 15개 물리 상태의 수치 유한성에 결측 하나라도 있으면 신규 Guard braking command lease조차 발행하지 않음. Simulator 별도 deadman 제동 이용.
- **R-A/S0-R**: 기존 양맵 v(s) 실행계/Spielberg R-A2 SHA 바인딩 보존. 새 속도 제약 검사는 R-B 실행 진입부에 실제 연결.

## 안전 계약
Guard 독립 권한, watchdog, fail-closed, source/Applied 신원은 보존. `mpc_formulation=ltv` 기본값이며 NMPC 자동 활성화 없음. 임의 5.0m/s 고정 cap 재도입 없음.

## 확인 상태
- 이 컨테이너에는 ROS Humble/rclpy가 없어 native Ubuntu `colcon` 전체 수행 불가능.
- 사용자 Ubuntu에서 새 후보 검증은 `RUN_FINAL_INTEGRATED_ROS_VERIFY.sh` 실행으로 시작; 목표 17 ROS packages, >=587 registered tests PASS.
- 오프라인 Python 코어(Guard/R-B/Red) 선택 테스트를 실행해 결과 별도로 제출. C++ Guard ROS 완전 실행기·Spielberg R-A3 SCP 수렴·15ms/7ms 실시간·ICCAS/Spielberg H2H 실증은 이 코드 변경으로 자동 증명되지 않음.
- 이번 후보는 코드 변경을 실제 포함하지만 4계층 전체 자격검증 완료본이라고 표시하지 않음.
