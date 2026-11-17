# F1TENTH RAB — 2차 실제 코드 통합 후보 (2026-10-18)

## 절대적 상태 구분
- 사용자 Ubuntu 1차 격리 검증: **17/17 packages, 492/492 registered tests PASS**, 별도 Applied 7 PASS.
- 본 2차 후보는 컨테이너 오프라인 검사 결과이며 **Ubuntu ROS 빌드/전체 테스트, 15ms·7ms, H2H 실증은 아직 미검증**이다.
- 기존 `~/f1tenth_racing` 및 모든 기존 ZIP은 수정하지 않는다.
- 전체 35개 결측 중 C++ ROS Guard 본체, 고속 정식 양맵 경로, 이동 중 ABORT 재정렬, NMPC 등 아직 미완성. **코드 완료 4/4 선언 불가.**

## 구현한 실행 코드와 증거
1. `f1tenth_racing_control/primary_worker.py`: MPC 전용 one-flight 독립 스레드, 늦거나 오래된 결과 재사용/승인 금지, 예외 격리 및 메트릭. `node.py`에서 fallback 사전 증명과 MPC를 동시 실행하며 mutable solver를 분리한다. **Python 스레드로 하드 RT 인정 불가**.
2. `f1tenth_racing_control/red_dynamic_guard.py`: NaN/Inf/범위 밖 source_confidence가 Python 비교에서 통과하던 문제 제거.
3. `f1tenth_racing_reference/node.py`: ABORT·BLOCKING_HOLD 계획의 COMPLETE가 nominal을 즉시 다시 열던 위험한 경로 차단. ABORT는 기존 parked+Red+collision gate로만 복귀; 이동 중 재정렬 미구현. 입력 미래/과거 Red 예측 캐시 배제.
4. `f1tenth_racing_raceline/foundation_adapter.py`: 공식 Spielberg 환경파일이 config 폴더에 있을 때 자산 탐색 실패 보완. 맵·센터라인·메타데이터 모두 SHA 검증 후 최적화.
5. `GENERATE_SPIELBERG_RA_CANDIDATE.py`: 동일 환경의 R-A0→R-A1→R-A2→R-A3 생성 도구. 28초 제한 컨테이너 시험에서 알고리즘 생성까지 끝나지 않았고 후보 최적화 **미완료**. 승인된 policy를 자동 수정하지 않는다.
6. ROS 패키지 등록 신규 테스트 16개 (Primary 7, Guard Red 4, Spielberg asset 2, S0-R 3).

## 검증 절차
- V1~1차 이전 PASS는 보존. 2차 추가 Python 선택 테스트는 오프라인 51 PASS (신규 레드/worker 포함), RA 입력 SHA 단독 검사 PASS.
- Ubuntu: 같은 source ZIP을 Downloads에 두고 격리 workspace에 해제, `bash RUN_SECOND_PASS_ROS_VERIFY.sh` 실행.
- 기존 ROS 등록 492 + 이번 ROS 등록 16 = **508개 이상** 완전 PASS 목표. 전체 registered tests 외 native library/GuardApplied audit 추가 검사.
- 이후 독립 프로세스 검증·C++ guard 자격화·양맵 실증은 별도 증거가 필요.

## Spielberg 오프라인 후보 생성 (별도 수동 실험, 실행시간 제한 없음)
```
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -B GENERATE_SPIELBERG_RA_CANDIDATE.py \
  --output-dir "$PWD/SPIELBERG_OFFLINE_OUTPUT" \
  --sim-share "$(ros2 pkg prefix f1tenth_racing_sim)/share/f1tenth_racing_sim"
```
실험에서 R-A3 수치 산출물이 나온 경우에도 `runtime_tracking_qualified`는 false, 정상 ROS Guard 승인 아님.
