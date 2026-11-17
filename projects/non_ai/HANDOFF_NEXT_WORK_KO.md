# 통합 후보 다음 수술 항목 — 전체 6개 작업군 인수인계

기존 최신 Scheduler 소스 기준 소스 불변량은 본 후보에 보존되어 있다. 그러나 이번 변경은 **P0의 타이머·메시지 신원 기반 골격**이며 사용자가 요구한 **전체 일괄 완성**과 같지 않다.

1. **실시간 Guard:** 별도의 C++ executor/whole-state physical step+full stopping proof 구현. 현 Python Guard와 완전 동일한 물리모델/마찰/jerk/충돌/footprint Golden으로 binary equivalence를 입증. CPU scheduling/IRQ/cgroup/WCET를 런타임 계약으로 포함하며 독립 braking deadman 유지. Python Guard와 중복 발행 절대 금지.
2. **R-B + GuardAhead:** current/N+1 proof를 생략하지 않는 유계 후보 탐색. terminal state마다 이미 계산한 공통 기하를 재사용하되, 다른 입력/명령의 stop certificate를 캐시해 재사용 금지. 계산 실패는 명시적 UNPROVEN. bounded executor/worker를 실제 분리하려면 무단 stale emission, racing revisions, state sequence 재정렬을 테스트. 지금 mailbox는 같은 thread에 있어 '진짜 RT isolation'이 아니다.
3. **실행 고속권한:** 8초 physical proof horizon과 v>15.5m/s의 불일치부터 해결. 지도/차량 물리·벽·마찰·코너/제동·reaction+jerk 의존 공간별 envelope를 R-A→S0-R→R-B→Guard에 일치시킨다. 5m/s legacy fixed cap 복원 불가. 스칼라 cap=20m/s는 물리 허용 범위일 뿐 safety grant 아님.
4. **MPC Primary + Solo 두 맵:** baseline/SCP, ICCAS/Spielberg의 경로·속도 생성, 실제 MPC/fallback Applied, physical lap oracle, 멀티랩. 현재 Pursuit only.
5. **1 Red H2H:** R-C LiDAR 추적, R-D uncertainty/TTL, R-E FOLLOW/PASS/REJOIN/ABORT/FOLLOW retry, independent dynamic occupancy. Guard의 현재 H2H interlock은 완성될 때까지 ON.
6. **자격검증:** 15ms/7ms 엄격 계약, stale message, process death, DDS delivery, end-to-end Applied latency, OS scheduling, physical lap, safety/collision G0~G9.

## 현재 후보 추가 위험

- state receive와 25ms processing timer의 상대 위상에 따라 Guard가 최신 또는 한 tick 전 상태만 수용하는 계약을 초과할 수 있다.
- bounded diagnostic queue가 64개 초과하면 가장 오래된 stage JSON을 버린다. 진단 데이터 완전성은 보장되지 않는다. Guard forensic actor trace와 ROS 데이터를 주 증거로 유지.
- 75ms freshness guard와 25ms callback wall limit는 *스케줄 실행시간 보장*이 아니다.

## 수정 금지

Independent Guard OFF, fail-open, Guard 15ms/GuardAhead 7ms 증가, 검증없는 H2H ON, 5.0m/s 고정상한 복원, Simulator bypass, provenance spoofing, certified stop proof 생략.
