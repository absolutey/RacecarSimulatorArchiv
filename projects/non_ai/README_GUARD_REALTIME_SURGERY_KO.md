# F1TENTH RAB — Guard/GuardAhead 실시간 인증 연산 근본 수술 V1

## 근거와 현재 판정

2026-10-18 ICCAS Solo 실증: START PASS, 첫 Guard 거부는 Intent #218 / physics #591, `GUARD_15MS_DEADLINE=17.315968ms`, 이후 HARD_LATCH 및 129.823ms callback 공백, 물리 랩 0. GuardAhead 7ms도 최초 거부 전 45회 초과. 이 후보는 위 실패를 실제로 **완주 해결했다고 주장하지 않는다**.

## 바꾼 코드 (기존 Startup Contract Repair 정본 그대로 복제, 격리 변경)

- `world/map.py`: 읽기 전용 PGM 점유 셀 불리언의 `uint64` summed-area table을 **맵 로딩 시 한 번만** 계산. 동일 `< threshold` 점유 정의의 사각 영역에 대해 상수 시간 FREE 판정. 임계치가 변경되면 캐시 사용 금지. 지도 밖 FAIL-CLOSED 유지.
- `world/swept_collision.py`: 이미 있던 FREE-only broadphase 판정에 정적 적분영상 사용. 장애물이 1셀이라도 있으면 기존 원래 SAT+보수적 sweep을 그대로 수행. radius는 명시적으로 동일값만 캐싱.
- `safety/stopping_envelope.py`: 고정 footprint padding과 disk radius를 인스턴스당 한 번 산출. **65스텝, reaction=0.25s, brake=2m/s², jerk/rate, dynamic limits, collision, stop constraints는 모두 동일**.
- `guard_ahead.py`/`final_preflight.py`: 현재/N+1 전체 정지증명 모두 유지. CPU 시간계측 추가, 기존 7ms wall-clock fail-closed 변경 없음.
- `actuator_guard_node.py`: 독립 15ms wall-clock deadline 유지. 해당 경로에 Guard 콜백 스레드 CPU 시간/벽시계 차이를 기록해 연산비용과 선점을 분리. 첫 거부 증거는 큐잉된 추적 채널로 전달하고, 짧은 실패 로그만 동기 출력.
- `forensic_trace.py`: 한 콜백마다 `open/write/close` 수행하던 계측을 bounded async diagnostics-only 큐로 분리. Guard 신뢰/검증/Apply 판정에는 연결되지 않음. CTRL-C 정상 종료시 flush. **실패한 경우에도 Guard는 인증 거부**.

**절대 변경하지 않음:** Guard 15ms·GuardAhead 7ms·R-B 25ms·state age 100ms·future 40ms·simulator 물리·R-A·속도 임의 cap·H2H 차단·Guard 명령 독점. safety/steering/acceleration/check count 단축 없음.

## 로컬 동등성 시험

- 과거 실패 시 상태 #591 + 명령 #218, 175개 stop rollout, 330개 swept, 330개 footprint, 520개 점유 영역 = **1,356개 golden output 0 difference**. 정상/역주행/동역학 한계/맵 밖/충돌 포함. golden은 수정 전 코드에서 산출.
- 회전 origin 및 Y flip 조합을 가진 합성 지도 6개 × 750영역 = **4,500개 동일한 셀 임계치 판정**, threshold 변경시 캐시 무효화 점검.
- 비동기 추적채널 4,000개 순서·수량 자기시험 PASS.
- 동일 컨테이너 단독 계산 350회: 65-step #591 예시 p50 **약 0.930ms→0.696ms**, 다른 서버 / ROS 호스트에서 재검증 필수. 이것은 실제 worst-case 15ms 통과를 입증하지 않는다.

## Ubuntu 실행 (Canonical/NEXT 수정 없음)

```bash
(
  set -euo pipefail
  cd ~/Downloads
  sha256sum -c F1TENTH_RAB_GUARD_REALTIME_SURGERY_20261018.zip.sha256
  test ! -e F1TENTH_RAB_GUARD_REALTIME_SURGERY_20261018
  unzip -q F1TENTH_RAB_GUARD_REALTIME_SURGERY_20261018.zip
  cd F1TENTH_RAB_GUARD_REALTIME_SURGERY_20261018
  python3 -B VERIFY_RELEASE.py
  bash BUILD_FORENSIC.sh
  bash RUN_INTEGRATED_SOLO_CAPTURE.sh
)
```

출력: `~/Downloads/F1TENTH_RAB_GUARD_RT_SOLO_*.zip`. ZIP 업로드 후 **Guard 최초 인증 거부**, `cpu_elapsed_us` vs `elapsed_us`, R-B candidate p95/p99, START/physics lap/속도/충돌 모두 분석한다.

`BUILD_FORENSIC.sh`에서는 16패키지 colcon build / full ROS tests; `RUN_INTEGRATED_SOLO_CAPTURE.sh`는 라이브 START 전 인증을 그대로 시행한다. 신규 테스트 불합격 시 STOP. 이 컨테이너에는 ROS2 Humble이 없어 실제 빌드·랩은 미검증.

## 안전 조건 및 남은 과제

- `STOP_CERTIFICATE_NOT_VERIFIED`가 다시 나타나면 **벽시계 마감 초과는 계속 거부**. Guard의 CPU 시간보다 벽시계만 큰 경우 OS 스케줄·진단/콜백 부하 문제로 분리하고 RCA 후 수정한다.
- 고속 20m/s는 시뮬레이터의 최대값일 뿐 자격이 아니다. `StoppingEnvelope(max_time=8s, brake=2m/s²)`는 20m/s의 완전 제동 가능성 증명을 보장하지 않는다.
- H2H/Spielberg/MPC Primary의 안전 자격과 ICCAS 실제 완주는 아직 미완료.
