# 4단계 작업 진척과 미해결 원인

| 단계 | 이번 후보에서 새로 완료 | 아직 미완료 | 사유 |
|---|---|---|---|
| 1. Independent Guard 실시간 안전 | Native full-stop port(V2 계승), 활성 맵 SHA binding, 독립 ROS 안전권한, lease/end-time 재검증 | ROS Guard C++ 전체 실행 스케줄링, 15ms WCET 및 하드웨어 Deadman | 네이티브 계산 커널이 빨라도 Linux 스케줄러·ROS callback의 최악 실행시간은 증명되지 않음 |
| 2. R-B/GuardAhead 실시간 제어 | mailbox 5ms, 소스시각/TTL, native N+1 preflight(V2 계승), 후보 동등성 테스트 | 7ms real-time WCET, 최악 후보 예산, MPC 정상 채택 | 시뮬레이션 동등성만으로 OS 지연 상한이 없음 |
| 3. 고속 공간가변 v(s) | **이번에 실제 런타임 제어 경로 연결**: R-A 원본 v(s) → model/horizon/curvature/accel/backbrake closed loop → R-B Pursuit/MPC reference. 원본 계획과 audit 분리 | R-A 자체 최적화/Spielberg 자동 qualification, full trajectory H2H stop authority, actual high speed execution | 현재 2m/s²/8초 full stop + 43m ICCAS 트랙에서 19~20m/s는 제약상 불가능. 새로운 임의 fixed cap 아님 |
| 4. MPC/양맵/H2H/평가 | geometric Red swept occupancy building block + physical-lap independent oracle | 실제 MPC Primary 사용, ICCAS+Spielberg clean laps, Red prediction 8초 horizon, 독립 H2H Guard, R-E ABORT/retry, G0~G9 | 전부 완성되지 않아 H2H interlock이 필수 유지됨 |

네 단계 전부 **제품 기준 완료는 0/4**. 사용자가 요구한 전체 제품 완성을 주장하지 않는다. 이 번들에서는 위험한 미인증 실행을 방지한 상태로 위 핵심 코드 부분을 실제 전진시켰다.
