# P0 현재 완료/미완료 게이트 (2026-10-18)

| 게이트 | 판정 | 근거 |
|---|---|---|
| 최신 RT 원본 + 실제 적용 Pinfix 단일 소스 | 소스 통합 완료 | PINFIX SHA/공유 StopProof 동등성 176 |
| 관리 소스 247개 SHA 일치 | 오프라인 PASS | VERIFY_RELEASE.py |
| 새로운 Solo/H2H 기본 실행 계약 | 오프라인 PASS | TEST_P0_ARCHITECTURE.py 7건 |
| 전체 StopProof 안전 판정 유지 | 오프라인 PASS | 1,356건 Golden |
| 지도 충돌 점유영역 유지 | 오프라인 PASS | 4,500건 비교 |
| 비동기 계측 자료보존 | 오프라인 PASS | 4,000 이벤트 |
| 최신 ROS 16개 빌드·327개 회귀시험 | **이번 릴리스 미실행** | Ubuntu에서 별도 실행 필요 |
| ICCAS START~물리 완주 | **미인증** | 최신 선행 런은 +2.574s Guard 정지 |
| Guard 15ms worst-case | **실패 원인 규명/해결 미완** | 선행 런 15.305ms wall/3.238ms thread CPU |
| GuardAhead 7ms worst-case | **미완** | 선행 런 49회 초과 |
| R-A3 SCP + MPC 실제 적용 | **미인증** | experimental opt-in만 존재 |
| Spielberg 경로·속도 실제 완주 | **미인증** | numeric artifact 및 연속 랩 필요 |
| 단일 Red H2H 추월 | **미구현 안전부분/실행차단** | Guard dynamic occupancy 미인증 |

## 다음 엄격한 작업순서

1. 이 릴리스를 Ubuntu Downloads 격리 경로에 압축해제, 고정 ZIP SHA 및 설치 패키지 origin 확인, 16 packages/327 regression 재검증.
2. **Baseline 실주행 1회**로 이전 첫 15ms Guard 지연이 재현되는지 확인. 과도한 repeated run/블라인드 rerun 금지.
3. 필요 시 별도 독립 run에 `F1TENTH_SCHED_DIAG=1`로 누적 `/proc/schedstat` 추적. 경쟁 원인의 정밀 규명에는 `sched_switch` 커널 trace가 추가로 필요하며, 시스템 권한 확인 후 별도 시행. 트레이스는 스케줄 우선순위를 변경하지 않음.
4. 원인 분류: Guard 15ms wall vs CPU vs sched wait, R-B callback 25ms and GuardAhead 7ms. 실제 증거를 기반으로 실행 스케줄 구조를 수정하고 그때까지 마감/안전판정 변경 금지.
5. Guard/GuardAhead 실시간 게이트를 통과한 뒤 R-A v(s) → R-B 실제 명령 → Guard Applied → Ego의 랩/속도를 확인; MPC/SCP는 별도 opt-in qualification.
6. 양맵 Solo physical lap/고속제동 증거 확보 전에는 H2H interlock 해제 금지.

이번 패키지는 **P0 소스 신원·실행 프로파일 정합성 보정**이며 실시간 계산의 근본 해결로 과장하지 않는다.
