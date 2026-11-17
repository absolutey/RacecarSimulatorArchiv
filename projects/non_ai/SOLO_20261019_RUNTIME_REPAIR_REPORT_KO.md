# ICCAS2025 Solo 실측 RCA 기반 통합 수정 — 2026-10-19

## 실측 근거 (사용자가 제공한 보존 증거)

- 이전 정본 ROS 테스트 17 packages, 610/610 PASS.
- Guard 최초 15ms 인증 거부: 15.553ms 벽시계 / 3.391ms 스레드 CPU / 약 12.162ms 스케줄링 간극. CFS 등 외부 원인 미규명.
- R-B GuardAhead 7ms 초과 156/1353 주기, 초과 주기에서 FALLBACK_STOP_PREFLIGHT_UNPROVEN.
- Guard 비상제동 210개 Applied 기록, 독립 시뮬레이터 GUARD_INVALID_EMERGENCY_BRAKE 거부 3건. 개별 현장 메시지 전문 없음.
- Ego 실제 4.882m/s 최고, pursuit 모드, 비충돌 34초. 20m/s MPC 성능은 입증되지 않음.

## 변경 코드와 한계

1. **Simulator 응급 제동의 수신-적용 시간차 해소**: Guard가 요청한 최소위험 제동 명령은 여전히 신원·출처·lease·물리 tick·0 조향·유한/정해진 감속 값(-2/0/+2)·비인증 증명 속성을 통과해야 한다. 승인되더라도 Simulator는 각 물리 tick에서 자신의 실제 속도 부호로 가속도 부호를 정하고, 제동력은 원래 2m/s²·모델 감속한도·0속도까지의 남은 1 tick 감속력 중 최솟값으로 제한한다. ZERO 넘어 역주행·속도 증가 없음. **CERTIFIED 승인권한을 만들지 않음.** 독립 deadman은 우선권 유지.
2. **Guard/R-B forensic 인코딩 오버헤드 축소**: 제어 콜백이 JSON 직렬화를 수행하지 않고, 진단 큐에 독립 이벤트 스냅샷만 넣는다. 데몬 기록 스레드가 인코딩한다. 진단 이벤트의 유실 가능성은 여전히 별개로 기록되며 안전승인과 무관하다. OS 스케줄링 WCET 보증은 아니다.
3. **GuardAhead 타이밍**: 7ms 유지, current+next *두 번의 독립 정지증명 유지*, 25ms 제어주기와 reserve 유지. 시작 시 남은 시간이 0 이하이면 인증증명을 실행하지 않고 fail-closed. wall-vs-CPU gap 및 시작 예산 기록 추가. 후보/충돌 모델/제동제약을 완화하지 않음.
4. **ROS 테스트**: 응급 비상제동 부호반전/0속도/변조 및 lease 만료, 7ms 경계, 두 정지증명, 비동기 포렌식 부작용 등 신규 테스트. 기존 코드의 선택 테스트는 기존 소스를 유지.

## 필수 재실증 판정

- 검증 스크립트 목표: ROS 등록 테스트 **641개 이상**, failures/errors/skipped 0. ROS 환경에서만 확정 가능.
- Solo 새로운 증거에서 Guard 15ms 초과 0건, GuardAhead 7ms 초과 0건, emergency invalid 거부 0건과 CERTIFIED/Applied, 충돌 0을 **동일 물리 tick 기준**으로 재확인.
- OS 선점 12ms가 다시 발생하면 변경된 스케줄링 외부 원인을 별도 추적해야 한다. 본 수정은 OS 지연을 제거했다고 주장하지 않음.
- 별도 양맵 MPC/NMPC 고속 주행 및 H2H 자격검증은 여전히 필요.

## 실행

```bash
EVIDENCE_ROOT=/tmp/f1tenth_solo_timing_evidence bash RUN_SOLO_TIMING_REPAIR_ROS_VERIFY.sh
# 전체 회귀가 PASS한 격리 워크스페이스에서만:
bash RUN_INTEGRATED_SOLO_CAPTURE.sh
```

정본 워크스페이스나 이전 ZIP을 덮어쓰지 않음.
