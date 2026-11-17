# 완성 여부: 확인된 사실만 기록

## 최신 실행 입력

사용자가 제공한 `F1TENTH_RAB_P0_SCHED_SOLO_20261018T115258.zip` 분석: START PASS, 최초 Guard 거부 Intent #237, Guard 15.554ms wall, 4.004ms CPU, involuntary context switches 45, GuardAhead 첫 237 control cycles 중 54 over-7ms, CPU 자체 over-7ms 27. 물리 lap 0. 상세 provenance는 전수 감사 ZIP 참조.

## 직접 교체한 source

- `.../node.py`: state mailbox, timed compute, no stale issuance, diagnostic decoupling, corrected forensic event identity.
- `.../guard_ahead.py`: approved response must also finish before proof deadline.
- `.../actuator_guard_node.py`: approved response must finish before 15ms.
- `.../speed_contract_audit.py`: pure audit new file (no run-time authority).

## 설계상 남은 원인

- Guard Python `rclpy.spin()`와 일반 Linux scheduling은 최악 보장을 구성하지 않는다. 45 context switches의 opponent PID는 현재 기록에 없다. C++로 옮겨도 RT scheduling/CPU topology가 없다면 15ms upper bound는 증명되지 않는다.
- GuardAhead 후보마다 current/N+1 full proof를 유지하므로 비용이 7ms를 넘을 수 있다. 그저 time-out시 fail-closed는 여전히 완주 보장이 아니다.
- 통상 속도 19.65m/s의 이상화 정지 시간 10.075초 (reaction 0.25 + v/2), 현재 8초 stop horizon 보다 김. 고속 권한 v(s)와 필요한 증명범위를 통합해야 한다.
- H2H 동적 occupancy 독립 cert 부재. Red 차단을 강제 해제해서는 안 된다.

## 타이머 설계가 실제로 보장하는 범위

동일 rclpy SingleThreadedExecutor에서 callback body 분리 = timing isolation 아님. `receive_state_snapshot`은 빠르고 최신 1건으로 coalescing하지만 계산 timer가 CPU 점유하는 동안 다른 callbacks도 대기할 수 있다. 그러므로 실주행 통과와 15ms hard realtime 인증은 미완이다.
