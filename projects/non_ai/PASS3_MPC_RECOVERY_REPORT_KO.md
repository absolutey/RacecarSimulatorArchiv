# F1TENTH RAB 3회차 — MPC 프로세스 시간초과 복구 최종 코드 보완

## 기준선

- 사용자 Ubuntu 최신 전체 회귀: 17개 패키지, 695 tests / errors 0 / failures 0 / skipped 0.
- 기존 7ms/15ms / 상태 75ms / 독립 Guard 승인 조건 보존. 실증은 의도적으로 미실행.

## 확정 결함

`IsolatedMpcProcessWorker.take`가 작업 시간초과 시 `_active_sequence`를 유지하고,
이후 `submit_request`가 active>=0인 동안 항상 False를 반환하여 Primary가 장기간
영구 비활성 상태가 될 수 있음.

## 수정 설계

- 시간초과/프로세스 사망/잘못된 회신은 즉시 Primary를 거부하고 기존 Fallback+Guard 경로 유지.
- 정상 제어 callback은 프로세스 종료·새 프로세스 생성 작업을 수행하지 않음.
- 별도의 supervisor thread가 기존 solver 세대를 폐기하고, 새로운 spawn solver의
  ready 확인이 끝난 뒤에만 신규 Primary 계산 허용.
- 이전 요청의 sequence는 재사용할 수 없고, 이전 세대 queue/reply는 새 solver에 전파되지 않음.
- 기동 복구 자체가 실패하면 `permanent_failure=True`, 계속 Primary fail-closed.
- 진단 `primary_process_recovery`는 관측 전용이며 actuator 승인 불가.

## 수정 범위

1. process_primary_worker.py (운영 소스)
2. node.py (진단 출력)
3. test_process_primary_recovery_thirdpass.py (신규 회귀 3개)
4. 기존 소스 manifest/릴리스 해시/실행기 (배포 증거)

## 자격검증 구분

- 독립 실행 Python 오프라인 5/5 PASS (기존 프로세스 2개+신규 3개).
- Ubuntu ROS 선택 패키지 `f1tenth_racing_control` 및 전체 17개 PASS는 미실행.
  등록 테스트 최소 698개, 0오류/0실패/0SKIP을 차단 게이트로 사용함.
- ROS 프로세스 통신 실제 장애·고속 MPC·15ms/7ms WCET·H2H·G0~G9는
  여전히 별도 실행/실증 전 `NOT_QUALIFIED`.
- 이 후보는 첫 번째 3회 계획의 '코드 동결'에 대한 마지막 잔여 결함 수정본이며,
  실제 Ubuntu 698+ PASS 전에는 동결로 주장하지 않음.
