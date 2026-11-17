# RAB 통합 결함 수술 — 범위, 실증 결과, 미완성 안전 게이트

## 범위 및 출처

- 원본: 사용자 Ubuntu NEXT의 정확한 247개 소스(`F1TENTH_NEXT_FULL_SOURCE_READONLY_20261016T161737Z.zip`), SHA `511dc0b62bab07c35fe966758037405445af34e2e01691563b8b976b4a3f7855`.
- 기존 NEXT에는 S0-R 승인 패치와 Guard Sync V1이 포함된 상태다. 본 후보는 그 **최신 상태를 바탕으로** 만든 격리 소스다.
- 이 환경에서는 ROS 2 Humble/rclpy, 공식 40Hz simulator 실행이 불가. Ubuntu 빌드, 전체 ROS 테스트, 실제 차량 랩 합격은 아직 증거가 없다.
- 코드 수술은 포함돼 있으나 장애 원인별 실시간 검증을 완료했다고 주장하지 않는다.

## 결함 F-01~F-10 이행 상태

| ID | 결함 | 이번 후보 변경 | 남은 실제 합격 게이트 |
|--|--|--|--|
| F-01 | Guard RUNNING+hard fault 모순 | `_brake`에서 SAFE_HOLD, hard latch 미해제, ready=false | ROS fault injection, 프로세스 재기동 및 정지 거리 |
| F-02 | R-B 고속 가속 상쇄 | current-station 종방향 target, lookahead steering 분리, FF mismatch 차단, 공간 제동 제약 | 연속 START→가속→감속→재가속 및 안전 랩 |
| F-03 | Guard 한 틱 지연 | 35ms stamp 단독제약 제거; exact seq+TTL+freshness+full certificate | 지터/역행/만료/누락 40Hz 통합 검증 |
| F-04 | GuardAhead 7ms | 인증 우회 없음, 미수정; 프로파일링/CPU 실증 필요 | 각 증명 단계 7ms worst-case, 전체 25ms |
| F-05 | H2H Guard dynamic Red 안전 인증 | **미구현**; H2H 차단 의도적으로 유지 | 독립 동적 점유 swept space-time certification, fail closed, 비정상 Red 시험 |
| F-06 | R-E 5.0 ceiling | successor planner 단독 하드 cap→sim physics-bound, DEFAULT AUTO; Guard 미구현과 분리 | H2H certification 후 고속 실행 |
| F-07 | Spielberg R-A 궤적 | 공식 R-A0→R-A1→R-A2 자동 생성 스크립트 제공, 이 환경에서 완료 불가 | 생성 + SHA/policy + 실제 벽 충돌 없는 정상 laps |
| F-08 | 환경 ICCAS hardcode | R-C/D/E 및 launch에 selected environment/track/map sha 전달, 서로 다른 맵 차단 | Spielberg ROS E2E 인지·예측·추월 |
| F-09 | MPC/SCP 실행 | 현재 코드 유지, Solo runner pursuit baseline만 우선 | MPC Primary direct adopt, fallback 전환, 2맵 주행 |
| F-10 | 실제 완주 오판정 | 평가 코드를 아직 변경하지 않음; Recorder/랩 oracle 개선 필요 | physical passes + distance + gate + 0 collisions |

## 안전 설계 판단

- `SAFE_HOLD`는 레이싱 합격 실패로 간주하되 fail-closed 운행 자체는 보존.
- 모든 Guard 재인증은 선행 메시지의 상태 시퀀스 및 stamp **정확 일치** 뒤에 기존 차량 물리+StopEnvelope certificate를 새로 평가. Never apply during pending.
- 35ms 물리 스탬프 간격은 Simulator dt=25ms의 **관측간 지터**와 혼동되지 않도록 변경했다. 단, one-tick sequence, state_age≤100ms, sender age≤100ms, bounded wait 40ms, exact stamp match와 기존 진로 불연속 검증을 유지한다.
- R-B 전방 감속 제약에서 `2.0 m/s²`는 별도의 공격적 cap이 아니라 현재 독립 StoppingEnvelope의 기본 계획 제동 모델과 일치시킨 *보수적 전방 제약*. 이후 simulator 매개변수의 단일 출처로 통합이 필요하다.
- H2H 이행 전 Guard `solo=false` 블록을 해제하면 안전 계약을 위반한다. 동적 Red 소스의 신선도, track identity, prediction horizon, ego footprint swept collision를 Guard 내부에서 독립 검증해야 한다.

## 검증에 성공한 것과 아직 못한 것

- 실제 소스 SHA에 근거한 일관된 격리 변경본을 제작.
- Python 문법 compile, AST/명시적 로직 테스트, 모델 기반 R-B 정지→가속 상쇄 회귀시험.
- Guard hard fault/recovery/future-tick 미적용 등의 오프라인 회귀시험과 원래 R-B 관련 단위테스트 PASS.
- **실시간 성능, 충돌, 랩, H2H, G0~G9는 모두 UNQUALIFIED.**

## 사용자 Ubuntu 안전 절차

1. ZIP SHA 확인→기존 NEXT/Canonical/Phase2 untouched→별도 격리 후보 풀기.
2. `VERIFY_CANDIDATE.py` 및 후보 unit tests→16개 package 빌드와 전체 테스트.
3. 통과 시 ICCAS2025 pursuit/baseline Solo 증거 수집. START 수락만으론 PASS 아님.
4. R-B/Guard 실제 주행이 정상인 경우에만 MPC/SCP, Spielberg 라인 생성과 실행, H2H 인증 구현을 검증.
5. 전체 자격검증 이전에는 canonical 승격, 자동 rollback, H2H interlock 제거 금지.
