# F1TENTH RAB — V6 R-B 실행기 분리 + GuardAhead 유한상태 차단 보강

## 근거와 기준선
- V1 Ubuntu ROS 테스트 327 / V2 328 / V3 338 / V4 374 / **V5 405 PASS** (사용자가 제출한 로그).
- V6는 V5 소스 전체를 포함하며 5개 소스를 추가/수정한다. 사용자 Ubuntu 정본 `~/f1tenth_racing`은 수정하지 않는다.

## 코드 변경
1. `control_worker.py`: 전용 Python worker thread, Condvar, 크기 1 최신 physics sequence mailbox, 덮어쓰기, 단조 증가 제한, 종료 시 미처리 명령 폐기, 예외 삼킴 대신 로깅·metrics. worker가 실패해도 옛 명령을 재발행하지 않는다.
2. `node.py`: `control_worker_enabled=True` 기본값. ROS state subscription은 시퀀스·물리 stamp 확인/최신 상태 제출만 수행하고, MPC/Fallback/GuardAhead는 전용 worker에서 수행한다. 5ms ROS timer는 비교를 위한 OFF 모드에서만 존재. `reference`, `race_start`, `watchdog`와 worker 사이 계산 상태를 RLock으로 직렬화한다.
3. `node.py`: 긴 계산 동안 새로운 CarState가 들어온 경우 낡은 계산 결과의 Intent 발행 금지. watchdog/Guard/Simulator의 기존 fail-closed 안전권한 계속 유지.
4. `node.py`: 250ms `control_worker_metrics` JSON에 queue lag, compute duration, overwritten receipts, errors, active/last sequence 기록. 관측용이며 차량 명령의 안전권한이 아니다.
5. `guard_ahead.py`: 15개 차량 상태 필드의 NaN/Inf를 **전부** 검사하며, 1틱 모델 예측값이 비유한이면 정지증명 엔진을 호출하지 않고 거부. 시험 스텁이 무조건 `qualified=True`를 반환해도 승인하지 않는다. 예산 만료 후 인증 불가를 추가 강제한다.

## 로컬 검증
- 최신 worker fault-injection 11개 / GuardAhead 비유한 입력 17개 = 28개 신규 테스트.
- `f1tenth_racing_control` ROS 의존 테스트 3모듈 제외 순수 비ROS 회귀 120 PASS; 제외 모듈은 ROS 패키지 미설치로 수집불가, Ubuntu에서 전체 검증 필요.
- Python AST/컴파일과 해시 검사 수행. ROS 2 빌드/colcon, 실제 worker latency 및 E2E 실증은 여기서 미실행.

## 남은 미구현/미보증
- Worker는 별도 Python thread이며 C++ 독립 RT 실행기가 아니다. Python GIL/OS 스케줄링/주기 지터 보증 없음.
- MPC Primary 계산과 Fallback 예측은 여전히 동일 worker 순차 호출 (GAP-10), 독립 deadline/workers 미완성.
- GuardAhead 7ms, Guard 15ms worst-case 실제 적용 시간은 미증명. 독립 Guard는 기존 Python ROS 본체.
- Spielberg 정식 고속 레이싱라인, H2H 이동 중 재정렬, G0~G9 승격 미완료.
- `control_worker_enabled=False` 비교 모드는 기존 5ms timer 방식으로 유지되나 정상 안전 자격 증명에 사용할 수 없다.

## Ubuntu 적용·검증
`bash RUN_V6_ROS_VERIFY.sh` — 기존 정본 불변/격리, ROS Humble 전 패키지 빌드+전체 colcon test + 새 28개 발견 / 총 >=433, 0 errors/failures/skips + Applied 증거 단위 테스트.
