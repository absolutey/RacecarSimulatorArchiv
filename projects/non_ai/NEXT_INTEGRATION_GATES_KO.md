# V2 다음 실제 통합 목표

이번 수술은 C++ 네이티브 stop-proof 엔진을 구현·연결하고, source/state lease & timer 위상 문제를 수정했다. **추가 진단만 한 패키지는 아니다.** 다만 현재 구현됐다는 사실과 Ubuntu qualification은 별개다.

## 1. 시간 마감 qualification (Guard 15ms / GuardAhead 7ms)

네이티브와 Python 두 프로파일에서 SOURCE SHA 동일한 ROS executable graph 확인 → 실제 First Guard reject, Guard & GuardAhead per-call wall/cpu/sched context 비교 → 비안전성 테스트 오류는 롤백하지 않고 도구 수정; 안전실패는 fail-closed 상태 유지. 커널 `sched_switch`는 정확한 선점 상대 귀속이 필요할 경우 **한 번만 제한된 로컬 추적**으로 기록. CPU affinity/FIFO 무단 자동 적용 금지.

## 2. 고속 실행 speed authority

R-A v(s) → S0-R ID/revision → R-B desired/ref → Guard exact physics stop horizon. 현재 8s +2m/s²로는 목표 19.65m/s를 인증하지 못함. 수학적 정지시간·거리, 마찰·곡률·스윕 벽 충돌·전후방 가감속이 일치하는 시간/거리 지평을 설계하되 기존 고정 5m/s 제한 복원 금지. 엔진 최대 정지지평을 그냥 늘리고 WCET를 무시해서도 안 됨.

## 3. MPC / 두 맵 / Red H2H

ICCAS/SCP baseline과 Spielberg 실제 물리 랩 Oracle, MPC Primary actuator 적용, Red LiDAR R-C, R-D prediction TTL/uncertainty, R-E FOLLOW/PASS/REJOIN/ABORT/FOLLOW retry, time-indexed Ego–Red dynamic occupancy independent Guard. 미인증 H2H Guard hard-interlock 계속 유지.

## 4. 자격 완료 기준

16/16 ROS 패키지 빌드, 전체 회귀, StopProof Python/native hard fail-closed boundary+corpus parity, Guard latency, GuardAhead 7ms, process death, R-A real ego speed distribution, 두 맵 clean physical laps, H2H no collision, source/release manifest and actual ROS package prefix.
