# F1TENTH RAB 4계층 코드 통합 변경 V1 (2026-10-18)

**판정: PARTIAL CODE INTEGRATION / NOT QUALIFIED FOR ROS OR PHYSICAL H2H.**

기준 파일: `F1TENTH_RAB_UNIFIED_CONSOLIDATED_IMPLEMENTATION_20261018(1).zip`.
이 패키지는 코드를 실제로 변경한 완전한 소스 트리입니다. 그러나 원래 감사된 35개 결측을 **전부** 완료한 것은 아닙니다.
원본의 327개 ROS PASS 기록은 이전 소스의 이력이며 이 후보의 PASS로 계승하지 않습니다.

## 구현한 실질 변경

- **Guard / H2H:** H2H launch 무조건 거부를 없애고 완전한 Perception/Prediction/Overtaking 플래그 + 독립 동적 Guard 게이트가 있을 때만 *실험 진입*을 허용합니다. Red `OpponentPrediction` 독립 수신, 추적 ID·시간·정확한 차량 정지시간 예측범위·동적 swept 충돌 검사 추가. 즉시 25ms command step과 기존 전체 stopping rollout에 같은 Red witness 사용; 데이터 부재·불일치·증명시간 초과 시 fail-closed.
- **Reference 갱신:** 실제 H2H에서 참조가 변경될 때 RUNNING 상태의 계약 신원만 교체. 기존 seq/시계 재생 방지 유지, 새 명령은 반드시 새 독립 정지증명 획득. 불량 참조는 정지.
- **Speed / R-A:** 고정 `speed_cap_mps:=5.0` 표기를 `AUTO` = 환경 물리 최대값으로 대체하되 모든 명령의 속도·충돌·제동 물리 Guard 제한 유지. 기존 H2H 오버라이드의 5.8m/s 전역 거부를 필요한 소스·물리 증빙·Guard로 이관. Spielberg centerline 기반 재현성 있는 provisional trajectory + SHA 정책 등록. **이 경로는 SCP 최적화나 차량 안전 자격 획득 경로가 아님.**
- **R-B / MPC:** 기본 SOLO MPC/SCP experimental profile 정합화. `nmpc` 미지원 B2는 자격 도구에서 명시적 거부 (구현한 것으로 속이지 않음). H2H 실험 script `f1h2h` 설치; `f1sim`, `f1safe`, `f1solo` launch 인자 충돌 정리.
- **Simulator / 평가:** Ego–Red 보수적 swept-disk 충돌 탐지; Guard 원본 Intent와 Applied 수신증의 비권한 진단 연결. `qualify_solo.py`에 물리 lap oracle 연결 및 source/map qualification gate 명시.
- **등록 참조:** `OFFLINE_CENTERLINE_PRELIMINARY_UNQUALIFIED`는 일반 실행에서 계속 거부. Spielberg 전용 `*_PRELIMINARY` 실행 프로파일일 때만 소스 신원을 보존하며 실험적 제출을 허용; true qualification 불가.

## 완료되지 않은 코드/증명 항목 (중요)

1. 15ms **end-to-end** 명령 승인/전송/적용 시간과 Guard 독립 C++ ROS 실행기, 별도 watchdog 독립 프로세스 및 네이티브 정식 배포 미완료. 현재 15ms는 Guard 내부 stopping proof의 실행 중 예산 검사이며 절대 E2E 보증 아님.
2. R-B Fallback/MPC 별도 스레드 또는 별도 executor로 확실하게 분리되지 않음. GuardAhead 7ms 전체 후보 집합 및 OS sched latency deadline 실증/코드 최적화 미완료. Native GuardAhead 정식 기본 ON 불가.
3. Spielberg 경로는 **예비 중심선 기반**, 고속 최적화 SCP·vehicle jerk/steering rate 포함된 안전속도권한/양맵 H2H 실승격 아님.
4. 장시간 Red 예측 모델은 순간 동적 점유 상계로 실패시 정지하나, opponent 가속·비관측 거동의 실물 정량 상한은 별도 검증 필요. 현재 실험 게이트를 실제 안전 인증으로 해석하면 안 됨. Red prediction 9s 추정은 차의 상대 감속 가능성을 '보장'하는 것이 아님.
5. H2H ABORT/REJOIN~FOLLOW/재시도 전체 폐루프 실차·Gazebo 실행 검증, 실행 상태전이와 Guard 권한 종단 추적 강제, B2 `nmpc`, G0~G9 공식 승격 일괄 자동화 미완료.
6. Simulator Red state physics tick 시간 정렬의 비동기성, 양맵 노드 초기화/시계·QoS 불일치에 대한 실제 ROS 2 런 검사가 남음.

## 이 컨테이너에서 수행한 코드 검사

- 비ROS 단위 테스트 **211 passed**; ROS 의존 7개 테스트 모듈은 이 컨테이너의 `rclpy` / 컴파일된 `control_msgs` 부재로 수집 실패. 211개 PASS는 성공적으로 수집 가능한 코드 테스트만 의미.
- 프로젝트 추가 통합 테스트 20개 PASS (211개에 포함).
- Python AST, Bash 문법, 정확한 내용 해시/ZIP 전수검사 및 source manifest verifier 실행.
- ROS 2 Humble, colcon, native 16 packages/327 ROS tests, 15ms E2E, 7ms E2E, 실제 주행 실증 **여기서는 수행 불가**.

## 사용자 Ubuntu에서 적용/검사 (단일 후보 격리, 기존 PASS 보존)

```bash
cd ~/Downloads
unzip -n F1TENTH_RAB_FOUR_LAYER_CODE_INTEGRATION_V1_20261018.zip
cd F1TENTH_RAB_UNIFIED_CONSOLIDATED_IMPLEMENTATION_20261018
python3 -B VERIFY_RELEASE.py
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
# H2H는 미자격 상태이므로 물리적 안전운행/실증 인정 불가.
# 실험적 런치: ros2 run f1tenth_racing_tools f1h2h
```

소스 파일별 SHA: `SOURCE_MANIFEST.json`, 릴리스 전체 파일 해시: `RELEASE_HASHES.json`.
변경 목록과 미완 35개 판정: `INTEGRATION_CHANGESET.csv`, `INTEGRATION_35_ITEM_STATUS.csv`.
원본 보존: `legacy_release/`, `INTEGRATION_DELTA.patch`.
