# F1TENTH RAB — 단일 실행·안전 아키텍처 통합 후보 (2026-10-18)

## 명확한 상태

본 산출물은 **실제로 소스를 변경한 통합 개발 후보**이지, 완성된 고속 H2H 레이싱 제품이나 Guard 실시간 자격증명이 아니다. 이전 최신 Scheduler Instrumented 소스에서 출발한다. Canonical/NEXT 아카이브는 변경하지 않는다.

### 통합된 변경 (현재 코드에 실제 반영)

1. **R-B 물리 상태 수신과 계산 실행 분리:** ROS `state0` callback은 단조 증가 sequence+physics stamp의 최신 상태 1건만 mailbox에 저장한다. 25ms control timer가 이 최신 스냅샷을 가져가 Pursuit/MPC/GuardAhead/Intent를 수행한다. 이전/역행 상태는 폐기, 잃어버린 tick을 생성하지 않는다. CPU-level parallelism은 **없다**. 동일 단일 스레드 Executor에서 콜백 작업을 나눈 것이다.
2. **명령 발행 시 신선도 재검사:** 계산 전/후 physics source stamp와 ROS now가 75ms 이내인지, 계산 경과가 25ms 이내인지 확인한다. 실패 시 오래된 Intent를 만들지 않고 독립 Guard의 기존 Deadman이 정지시킨다. source sequence/revision을 덮어쓰거나 최신값으로 가장하지 않는다.
3. **비필수 계측 경로 분리:** Intent/ControllerStatus 뒤에서 JSON timing 직렬화를 수행하던 작업을 별도 타이머로 이전한다. 64건 FIFO, 매 콜백 최대 2건, 과부하시 오래된 진단 기록 폐기 가능하다. **안전성 증명 자료에는 이 lossy stream만 의존하면 안 된다.** 기존 actor_trace/stop snapshots, ROS control remain authoritative.
4. **최종 인증 응답 시 마감조건 재검사:** Guard와 GuardAhead의 전체 증명이 통과하더라도 스레드가 마지막 마감 확인 직후 선점되어 응답시간을 넘는 경우 허가하지 않도록 return boundary에 재검사한다. 기존 15ms / 7ms 그대로.
5. **고속 정지지평 계산:** `speed_contract_audit.py` 순수 오프라인 진단 추가. v=19.65m/s, b=2m/s², 0.25s reaction, 8s horizon 충돌을 수치화한다. **속도권한을 부여하거나 Guard 승인을 대체하지 않는다.**
6. 기존 Pinfix single-authority StopProof, H2H 차단, fail-closed, 한 틱 신원 결속, 단독 `ackermann_cmd0` 발행은 보존한다.

### 구현되지 않았으므로 합격이라고 주장하지 않는 작업

- C++ 독립 Guard 런타임 및 OS-level CPU isolation / kernel scheduler bound
- GuardAhead 7ms worst-case proof latency 물리 보장; 후보별 반복 정지증명 개선은 추가 설계가 필요
- 총 수신→제어→Guard→Simulator 지연의 유한 upper bound 증명
- S0-R 전 맵/경로별 고속 v(s)와 실제 physical stopping corridor authority 통합
- R-B MPC Primary 실주행 채택 및 Spielberg 완주
- R-C/R-D/R-E 단일 Red H2H와 독립 dynamic occupancy certification
- 물리 랩 Oracle, G0~G9 full certification

### 실주행 위험 평가

기존에는 physics-state callback 안에서 한 번에 명령을 계산했다. 이번 후보는 타이머 기반 25ms 계산으로 옮기므로 상태/타이머 위상 지연이 달라진다. 특히 R-B와 Guard의 exact/latest-or-one-tick 계약에 따라 **새로운 HARD STOP이 발생할 수도 있다**. 이것이 바로 아직 로컬 ROS 자격이 없는 이유다. 오프라인의 함수·물리모델 동등성 검사는 최악 CPU 스케줄·모든 타이머 위상을 증명하지 않는다.

## 아키텍처 목표: 여섯 통합 작업을 하나의 소스 정본으로

| 작업 | 반드시 보존할 계약 | 완료 기준 | 이번 후보 상태 |
|---|---|---|---|
| A Independent Guard | 실제 Ego/벽 swept collision, 독립 proof, 15ms, fail-closed | 경계 테스트 + deadline/WCET + process-death stop + applied deadline | 일부 응답 마감 보강, **미완** |
| B R-B/GuardAhead | source state seq/TTL/revision, 7ms current+N+1 | 유계 후보 평가, 모든 승인명령 동등한 full proof | mailbox + 반환마감 보강, **미완** |
| C High-speed authority | 동역학, friction, corner, reaction+full-stop horizon; no arbitrary 5m/s | R-A v(s) to Guard-certified Ego across two maps | nominal mismatch diagnosis, **미완** |
| D Solo ICCAS+Spielberg | 실제 방향/corridor lap, no collision | clean 1+ physical laps, speed acceleration/braking on two maps | **미완** |
| E Single Red H2H | dynamic time-indexed Red occupancy, ABORT/FOLLOW/retry, Guard ON | scenario+fault-matrix no Red/Ego-wall collision | interlock preserved, **미완** |
| F Release qualification | source SHA+ROS graph+deadline+faults+lap invariants | G0~G9 evidence audited | offline partial, **미완** |

## 실제 코드 연결

- `node.py`: `receive_state_snapshot` → `_pending_state` → `run_latest_state_cycle` → existing `receive_state` computation → preflight → postcompute gate → `publish` → `actuator_intent0`.
- `guard_ahead.py`: unchanged candidate set, each candidate same current+next full proof; `done` response cutoff now also checks 7ms/25ms boundary.
- `actuator_guard_node.py`: same `_certificate`, `_brake`, `_validate_and_publish_intent` and single `ackermann_cmd0`; full-stop proof conditional response cutoff added.
- `speed_contract_audit.py`: static non-authoritative horizon discrepancy report.
- `forensic_trace.py`: actual state receipt `RB_STATE_ENTER/EXIT`, compute `RB_CYCLE_ENTER/EXIT`. Older RCA consumer that assumes state callback is compute must be updated.

## 이번 후보의 테스트 범위

- 컨테이너에서 source-based 9개 신규 mailbox/gate 시험, 기존 P0 구조 7개, 실행 8개, Tick 14개, 스케줄 계측 5개, GuardAhead/FinalPreflight 24개, 1356 StopProof Golden, 4500 occupancy Golden.
- 사용자 Ubuntu ROS 빌드 16 package/327 test 및 물리실행은 **이번 소스에 대해서는 수행되지 않음**. 과거 327 PASS는 이 소스의 PASS로 계승하지 않는다.

## 우선순위와 다음 gate

이 후보를 '완료' 또는 Canonical 채택으로 올리지 않는다. 단일 통합 소스 수술을 계속하려면 A+B의 C++ certified Guard / WCET-aware bounded proof와 R-B timer/executor phase contract를 같이 완성하고 G0~G9 전제조건을 충족해야 한다. 실패가 나와도 버전 파일을 무조건 늘리거나 Safety latch를 끄지 않고 이 후보의 구조적 미구현 항목을 직접 수정한다.
