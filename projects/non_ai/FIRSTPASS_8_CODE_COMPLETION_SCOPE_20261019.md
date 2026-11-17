# F1TENTH RAB — 1회차 8개 영역 전체 코드 통합 (2026-10-19)

이번 산출물은 **실증 이전 코드 구현 후보**다. 기준선: Ubuntu 17개 ROS 패키지와 668개 등록 테스트 PASS. 이번 수정본의 ROS 빌드/전체 등록 회귀는 Ubuntu에서 **실행 전**이며 완료로 표시하지 않는다. `RUN_FIRSTPASS_8_IN_ONE_ROS_VERIFY.sh`에서 변경 패키지 선택 빌드/테스트→전체 빌드/테스트를 순차 확인한다.

| # | 영역 | 이번 실제 코드 산출물 | 완료 경계 / 유보 |
|---|---|---|---|
| 1 | GuardAhead 7ms | Native C++ 현재+N+1 정지증명 묶음 ABI, Python `evaluate_dual` 선택 경로, ICCAS/Spielberg 각 130사례 교차시험 0 불일치 | 7ms 최악 실행시간은 실증 전 미인증. 원래 Python 증명과 7ms 차단 불변 |
| 2 | 독립 Guard/격리 | Solo용 `rclcpp` C++ ROS Guard 노드, 단독 publisher launch 선택, map+native proof 원본 신원검사. `IsolatedMpcProcessWorker` 별도 `spawn` 계산 프로세스 | C++ ROS 컴파일/노드 활성화는 Ubuntu 미검증. H2H는 원래 Python Red Guard. 기본값 구 경로 유지 |
| 3 | Guard 15ms+Applied | C++ 단일-소유권 증명/명령/lease/watchdog 구현, 기존 Python Guard·독립 watchdog·Ego deadman 유지 | 15ms E2E/WCET/장애주입 및 실제 Applied 증거 필요 |
| 4 | Spielberg R-A3 | R-A3 실제 sparse SCP 경로+속도 생산 성공: 864점, 3단계, 22.45006→22.39985초 *offline estimate*; 별도 true-joint 실험용 후보 3단계, 59.65555→58.04090초 | true-joint는 보수적 초기속도로 추정 시간이 느리며 운영 승격 금지. 기본 R-A2 유지 |
| 5 | 고속 MPC | 기존 LTV/NMPC, 임의 5m/s 고정 cap 없는 R-A→S0-R→R-B 체인 보존; 독립 프로세스 선택, 두 맵 `scp` 실험 프로파일 추가 | 실제 Ego 고속 추종·Guard 승인 검증은 실증 단계 |
| 6 | H2H 복구·재시도 | ABORT→S0-R parked 재진입 후 **ABORT보다 최신 Red 예측**을 강제, 기존 ABORT 이전 emergency MERGE/FOLLOW와 새 계획 정합성 유지, G8 재시도 증거 강화 | 이동 중 임의 hard-ABORT 래치 해제 없음. full H2H 통합/성공 실증 필요 |
| 7 | 양맵 실증 자동화 | `FIRSTPASS_MULTIMAP_RUNNER.py` 4개 Solo/H2H 실행 구성, 해시·신원 사전검사, 단독 실행·독립 rosbag/HUD·증거 보존 | 기본은 dry-run. 실제 실증 **이번에 수행하지 않음** |
| 8 | G0–G9 | `BUILD_FIRSTPASS_G0_G9_RECEIPTS.py` 실제 동형·동일 run/map/source 원본만 수집, SHA·symlink·실행신원 차단, G8 증명 강화 | 원본 증거 없으면 MISSING, 공식 승격은 독립 평가 이후. 가짜 PASS 작성 금지 |

### 절대 유지할 안전 계약
- Guard의 액추에이터 최종 권한과 fail-closed / 독립 simulator deadman.
- Guard 15ms 및 GuardAhead 7ms, R-B CarState 75ms, QoS BEST_EFFORT/depth=1 회귀 보존.
- H2H Red 동적 충돌 증명 및 Parked 후 복구; C++ Guard를 H2H에서 자동 사용하지 않음.
- `scp_true_joint_experimental`은 미자격 후보로만 등록. 경로 속도 최적화의 예측 랩타임을 실주행으로 해석 금지.
- 기존 668 Ubuntu PASS와 새 소스 Ubuntu 미검증 결과 구분.

### 테스트
- 컨테이너에서 독립 C++ dual-tick vs Python 260 사례(두 맵 각 130) mismatch 0, native stop 1176 사례 parity PASS. CPU benchmark는 WCET 자격이 아님.
- Python 선택 테스트는 별도 실행 로그와 같이 확인. 전체 ROS 테스트는 사용자 Ubuntu에서만 가능.
- 일관된 단일 릴리스 ZIP이며 canonical `~/f1tenth_racing`를 덮어쓰지 않음.
