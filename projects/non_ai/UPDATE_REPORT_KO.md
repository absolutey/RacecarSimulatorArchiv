# F1TENTH RAB Continuity Upgrade 2026-10-17

## 실제 적용한 코드

**정본 보존:** 원본 NEXT / canonical에 쓰지 않는다. 직전 247-file integrated candidate에만 변경을 누적한 독립 복제본.

1. `simulator.py`: `CarState state0` RELIABLE writer history를 1→16으로 확대. `odom`, `scan`, control command QoS는 변경하지 않음. 40 Hz의 상태 시퀀스가 Guard에 도달하기 전 최신값에 의해 덮어쓰이는 가능성을 줄이려는 **가설 기반 수정**. 전달 보장이나 실시간 성능은 ROS에서 미증명.
2. `actuator_guard_node.py`: RELIABLE state0 subscriber history 1→16. `StateTimeline`이 실제 수신된 상태 시퀀스와 누락 구간을 기록하며, 미래 Intent의 exact binding 상태를 추가 확인한다. 결손 물리 상태 합성/보간 없음. 이전 40ms 제한, final independent StopProof, HARD_LATCH, `solo=false` 차단 모두 그대로.
3. `state_timeline.py`: bounded state receipt history 16, ROS+steady TTL 분리, 스탬프/시퀀스 역행 및 gap 탐지. **관측·결속 용도이며 액추에이터 승인권 없음**.
4. `cycle_trace.py`: 순수 Python monotonic duration/limit 영수증. 제어권 없음.
5. `guard_ahead.py`, `final_preflight.py`, `node.py`: 두 번의 FULL stop preflight를 그대로 수행하며, 기존 `r_b_compute_stages0` 진단에 각 proof elapsed time과 후보 수를 기록. GuardAhead 7ms와 Guard 15ms는 변경하지 않음.
6. `actuator_guard_node.py`: 실시간 Guard full certificate의 15ms 초과 건수/최대 소요시간을 내부 카운터에 보존. **계측값은 제어 승인을 완화하지 않음**.
7. `EVALUATE_SOLO_EVIDENCE.py`: Recorder physical_passes/충돌/실제속도의 최소 사후 검증만 수행. `physical_passes>=1`은 출고 최종 oracle이 아님; 안전·명령/Applied·lap 독립 검사까지 추가 필요.
8. 전수 해시 manifest, 독립 빌드/시험, Fulltrace Solo 실행 수집. NEW candidate root에 exact path binding. 실행은 Ubuntu ROS Humble에서 수행해야 함.

## 반드시 인정할 현재 미완료

- ROS2 Humble ROS 실빌드·317+ tests: 사용자 환경에서 새 소스 검증 필요.
- QoS 변화가 #833 누락을 실제로 해결하는지: 미검증. 긴 stop proof가 Guard executor를 블로킹할 수 있음. 재현되지 않거나 개선폭이 없으면 executor/worker 아키텍처 개입해야 함. 40ms만 올려서는 안 됨.
- GuardAhead 7ms: **미해결**. 이번에는 호출별 비용을 실제로 계측하도록 변경; 동일 FULL TWO STOP PROOFS 유지. 이후 실제 병목에 맞춰 계산 개선.
- Moving state fault는 여전히 HARD_LATCH. 정상 jitter가 HARD로 승격되는 사건을 회피하는 정책은 **미완성**. 안전근거 없이 자동 재출발 불가.
- 잔여 0.0146m/s 정지 유지: 미수정.
- R-B 고속 MPC Primary, Spielberg 정본 R-A, Red H2H 동적 Guard: 미완성. `solo=false` 차단 그대로.
- `20m/s`는 시뮬레이터 한계이지 안전하게 인증된 vmax가 아님.

## 이번 후보의 검사 순서

1. SHA 검증, 247개 소스 manifest·신규 파일 checksum 검증.
2. ROS 없는 `test_state_timeline_upgrade.py` + 현행 GuardAhead/Preflight 회귀 테스트.
3. ROS Humble 16개 패키지 빌드, 기존 전체 테스트 + 신규 테스트.
4. `RUN_INTEGRATED_SOLO_CAPTURE.sh`는 새 격리 소스로 1회(추가 반복 주행 금지) 진단 수집.
5. 실패 시 START+10.132초 대비 `MISSING_PHYSICS_TICKS` 빈도, `FUTURE_STATE_*`, per-proof time, GuardApplied, 물리 lap을 대조. 0 clean lap 또는 hard fault이면 출고 불가. 개선되지 않을 경우 blind QoS increase/retry 금지.

## 알고리즘과 논문 연결

- S2-021 / S2-032: 실제 받은 state 순서·유효기간·상태결속/메시지 큐.
- S2-037: ROS2 콜백·latency 추적, 최악 실행 지연 분석.
- PAPER-000039: 시간제약/feasibility, 현재 단계는 측정만 수행.
- S2-036: R-A/R-B longitudinal velocity architecture, 직전 후보 초기 가속 개선 보존.

## FAIL-CLOSED 불변조건

- 인증 실패 후보가 `ackermann_cmd0`에 도달하는 경로 추가 없음.
- H2H Guard interlock 제거 없음.
- 5.0m/s cap을 근거 없이 재도입하지 않음; 20.0m/s도 자격으로 표시하지 않음.
- previous candidate 247 sources with all 10 cumulative modifications (plus new changes) 유지.
- 원본 보호, 결과 증거 보존, 오류 발생 시 안전계약 완화 금지.
