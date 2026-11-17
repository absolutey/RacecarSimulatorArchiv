# P0 Solo 2026-10-18 10:39:38 — 첫 정지 인과관계 및 고정 물리스텝 계약 수술

## 증거와 범위

- 사용자 Ubuntu 실제 run: `F1TENTH_RAB_P0_SOLO_20261018T103938.zip`; SHA256 `b38c4fe5b70a677a7286376c6e1790b759ab14c0005821c9283faf127eb54675`
- 적용 후보: `F1TENTH_RAB_P0_ARCHITECTURE_INTEGRATED_20261018` + 해당 런타임 실행 증거. 원본 ZIP SHA256 `4458a58613b912e6c67665774279b5e9e505d66eb5f502dfe8589f7d413bf310`.
- 로컬 source `actuator_guard_node.py`, simulator `simulator.py`, 정본 `competition_environment.yaml` 대조.
- ZIP 속 actor trace (`GUARD_STOP_FAILURE_SNAPSHOT`, `GUARD_VALIDATE_ENTER`, `GUARD_BRAKE`) + telemetry + full_topic_trace 교차검사.

## 실제 결과 (경주 미달성)

- 16 ROS 패키지·327 ROS 테스트 PASS; PRESTART_LIVE PASS, 첫 START 수락.
- 최고 실제 Ego 속도 3.9403088457882345 m/s; 실제 XY 이동거리 19.549194724223383 m.
- 정상 물리 랩 0 (recorder lap=1, physical_passes=0).
- R-B MPC Primary 채택 0; Pursuit / Fallback 실행.
- GuardAhead 7ms 초과 204/1283 계산 기록; wall-clock p95 약 8.914ms, p99 11.767ms. 별도 잔존 결함.
- 첫 Guard brake: START + 약 6.319초, Intent #252, 이전 승인 #251, 상태 #736(명령 원본)→#737(Guard 최신), `GUARD_REJECT:STOP_CERTIFICATE_NOT_VERIFIED`.
- 이 최초 실패에서는 Guard 15ms 인증 마감 초과 증거가 없음. `stop_envelope_result=null`; 원본 `_certificate`는 동역학 정지증명 실행 전에 조기 `False`를 반환.

## 최초 원인: ROS 발행시각을 물리적 적분 시간으로 착각

Guard `actuator_guard_node.py` `_certificate` 의 N-1 소스 틱 허용 분기:

```
source_intent.physics_sequence == 736
Guard.latest.physics_sequence  == 737
ROS stamp gap     == 0.015242463 s
Physical dt       == 0.025000000 s (simulator physics_step reads physics_timestep_s)
Previous v        == 3.2565984521061115 m/s
Latest v          == 3.2065984521061117 m/s
XY displacement   == 0.08141496130265284 m
Old reach bound   == max(v)*stamp_gap +0.02 == 0.06963858141208468 m -> FALSE
Correct tick bound== max(v)*physics_dt+0.02 == 0.1014149613026528 m -> TRUE
speed delta       == 0.05 m/s <0.15 m/s
Yaw delta         == 0.031135rad <0.15rad
Stamp age         == 21.459053ms <100ms
```

Simulator는 `physics_step()`의 타이머가 호출될 때마다 `Dynamics.step(..., config["simulation"]["physics_timestep_s"])`를 호출하며, `physics_stamp`에는 함수 실행 후 실제 현재 ROS 시각을 기록함. timer 호출 간격은 실시간 jitter로 변동한다. 따라서 `stamp_next - stamp_prev`는 **실제 고정 물리스텝이 아니다**. 이번에는 선행 상태 #736의 stamp gap이 36.103ms였고 이어 #737 stamp gap이 15.242ms였으나, 각 실제 dynamics integrator는 25ms를 적분했다.

`_certificate`는 `STOP_CERTIFICATE_NOT_VERIFIED`라는 통합 reason만 Guard 계약에 전달하므로 증상이 정지증명 실패처럼 보였다. 그러나 조기 `False`이므로 이 최초 거부에 대해서는 **StopEnvelope 시간초과·차량 정지불능 모두 직접 원인이 아니다**.

Guard 이후 watchdog의 `INTENT_WATCHDOG_TIMEOUT` 연속 래치는 **최초 거부 이후의 후속 상태**이다. 변경할 대상 아님.

## 변경 범위

1. `actuator_guard_node.py` 1틱 지연에 대한 상태 위치 연속성 검사를 메시지 간 타임스탬프 차이가 아닌 **정본 시뮬레이터 고정 물리 timestep**으로 판단.
2. 환경 `physics_rate_hz`와 `physics_timestep_s`의 일치를 요구. 무효 구성은 fail-closed.
3. 입력 position/speed/yaw NaN/Inf는 fail-closed.
4. 불연속이 실제 발생하면 별도 branch `PHYSICS_TICK_DISPLACEMENT_UNREACHABLE`과 계산값을 기록해 이후 원인 추적.
5. ROS stamp의 시퀀스 정합성·단조성·100ms 신선도, 물리 속도·회전변화 제한, 최초부터 완전한 StopEnvelope, Guard 15ms, GuardAhead 7ms, H2H 차단은 보존.
6. `BUILD_FORENSIC.sh`와 `RUN_INTEGRATED_SOLO_CAPTURE.sh`를 새 동일 스테이지 경로로 정합화. `SOURCE_MANIFEST`, `RELEASE_HASHES`, `RELEASE_MANIFEST` 갱신. 원본 P0는 비변경.

## 검증 (오프라인)

- 동일 실행 증거에서 원본의 **실제 `_certificate()` 함수** → `False`, stop proof 호출 0회.
- 수정 후 **실제 `_certificate()` 함수** → full StopEnvelope 호출 단계 통과.
- 실제 미변경 `StoppingEnvelope.evaluate()`로 동일 맵·상태·명령 재생 → `qualified=True`, `CONDITIONAL_STOP_ONLY`, 65스텝, 1.625초, 2.403464m (로컬 실행). 실시간 마감 자격은 아님.
- `TEST_GUARD_TICK_BINDING.py` 14개: 타임스탬프 jitter 허용, teleport 거부, velocity/yaw jump 거부, stamp 역행·만료 거부, sequence mismatch 거부, nonfinite 거부, 물리 설정 불일치 거부, 정지증명 실패 강제 거부, 안전예산 보존.
- 기존 `TEST_STOP_PROOF_EQUIVALENCE.py`: 원본 Golden 1356개 stop / swept / occupancy outcome 불변.

## 아직 수정되지 않은 실시간·최적화 영역

1. GuardAhead 7ms 초과 204/1283회, `FALLBACK_STOP_PREFLIGHT_UNPROVEN` 205/1290 회, 정상추종 130/1290 회.
2. Guard 15ms 스케줄 지연은 **이전 런에서** 관측됐으나 이번 첫 실패와는 별개의 문제. 미해결 상태.
3. Solo baseline pursuit; MPC Primary, 최적 고속, 실제 랩, Spielberg, H2H 미자격.
4. Simulator ROS stamp의 의미와 순수 simulated physical time 분리는 추후 `physics_sim_time` 메시지 확장으로 정식 명세화 가능. 현재 안전 계약은 고정 step을 사용하되 원본 timestamp/sequence 검증 유지.

## 반드시 지켜야 할 릴리스 게이트

- 이 후보는 Ubuntu 빌드/ROS 테스트/실주행 **미검증**이다.
- 실제 주행 후 `first_brake`의 reason/intent/physics seq를 다시 검토한다.
- Guard 15ms·GuardAhead 7ms 횟수, accepted Intents, speed, physical laps를 함께 판정한다.
- 실패하면 임의 rollback 또는 5m/s cap 복원 없이 새로운 원인을 증거 기반 분석한다.
