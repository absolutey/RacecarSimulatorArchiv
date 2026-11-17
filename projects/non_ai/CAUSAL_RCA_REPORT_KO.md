# F1TENTH RAB — 최초 치명정지 재현의 인과 사슬과 미확정 지점

## 데이터 출처
- 최신 실행 증거 `F1TENTH_RAB_CONTINUITY_SOLO_20261017T232153.zip` (SHA256 `2aaf091f4ac335c38eef7433836509917b88bd2ae226b2c643cff769245805bf`)
- 후보 소스 `F1TENTH_RAB_CONTINUITY_UPGRADE_20261017.zip` (SHA256 `560c3c3e1f3bd7994694704ccf76838dec1b99cdd80c92d36c88ac04d8acb257`)
- 사용자 Ubuntu에서 해당 후보 16 ROS 패키지, 327개 테스트 PASS. 실주행은 FAIL.

## 확인된 원인 사슬
1. Simulator는 물리 상태 1,328개, #256~#1583을 독립 수집자에게 40Hz(약25ms)로 전달했다. 물리엔진 전체 정지 가설은 이 관측과 부합하지 않는다.
2. R-B `node.py:257`는 `state0`의 `depth=1` 수신자. `receive_state`의 실제 `physics_stamp`를 현재시각과 비교해 걸러내지 않고 `last_state_ns = now_ns`로 기록한다(364~370행). 이것은 구형 샘플을 콜백 처리시각 기준으로 최신처럼 보이게 하는 결함이다.
3. R-B 정상 상태기반 계산 278개는 주기적으로 발생하지만 원천 물리상태 연령 약 322~380ms. 상태 이벤트가 2~4개 연달아 일어나는 군집 후 약 300~380ms의 긴 공백이 반복된다. 상태기반 계산 4ms 이하라는 기록은 300ms 이상 지연이 같은 callback 본체에서 소비됐다는 가설을 약화시킨다. 같은 기간 watchdog이 25ms로 반복됐으므로 프로세스 전체가 연속해서 막힌 것은 아니다.
4. R-B `watchdog()` 826~837행은 마지막 *콜백 실행 시각*이 일정 기간 갱신되지 않으면 `STALE_STATE`를 일반 `ActuatorIntent`로 발행한다. `publish()` 839~869행은 과거 source-state stamp/seq를 유지하고 새로운 created_ros_stamp를 달아 발행한다. 첫 Intent #1의 source-state #310은 실제 발행시 481ms 이상 오래됐음.
5. Guard `ActuatorGuardContract`의 state freshness 100ms, stop-proof의 정확한 state binding을 만족할 수 없으므로 그 Intent는 인증 불가. 기존 계약 함수의 격리 재생에서 거부와 `SAFE_HOLD`를 확인.
6. 실제 Guard의 첫 `SAFE_HOLD`는 START+0.0917초, 승인 0건. 최초 `_brake(reason)` 호출 이벤트는 남아 있지 않고, 이후 watchdog의 `INTENT_WATCHDOG_TIMEOUT`이 상태 사유를 덮어쓸 수 있다. 따라서 최초 ROS 콜백의 정확한 오류 문자열은 기록만으로 결론낼 수 없다.

## 이전 보고서보다 더 좁힌 부분
**'ROS2 Controller 입력 파이프라인이 장기간 정상 40Hz를 처리하지 못한다'**는 현상은 정상 주행 구간의 전체 반복 패턴으로 강하게 입증된다. 하지만 `state0` 메시지가 R-B 프로세스의 DDS 수신 캐시에 언제 들어왔는지, `rclpy` Executor가 해당 subscription callback을 언제 runnable로 보았는지는 원본에서 수집하지 않았다. `depth=1` 단독 원인이라고 단정할 근거는 없다.

## 이번 패키지의 목적과 안전 경계
새 패키지는 *기존 후보를 격리 복제하고 R-B와 Guard의 콜백 메서드를 진단 래퍼로 감싼다*. 기존 메서드 본문·검증 조건·Guard/StopProof/H2H interlock을 수정하지 않는다. Simulator 발행은 기존 그대로이며, 로그를 읽는 별도 QoS probe(BEST_EFFORT1, RELIABLE1, RELIABLE16)를 추가한다. Python 파일 쓰기와 추가 ROS 구독 자체가 작은 성능 교란을 유발할 수 있으므로 결과를 최종 자격 주행으로 취급하지 않는다.

구체적인 입증 항목:
- RB_STATE_ENTER physics age가 300ms 이상인가? RB_WATCHDOG_ENTER는 정상적으로 25ms 실행되는가?
- RB_REFERENCE_EXIT/STATE_EXIT가 수백 ms를 소비하는가? 아니라면 Executor 콜백 장시간 점유 가설은 약해진다.
- QoS probe 3개는 모두 신선한가, RELIABLE1만 늦는가?
- 최초 GUARD_BRAKE 사유, 그 전후 GUARD_INTENT_ENTER 및 GUARD_VALIDATE_ENTER 순서가 무엇인가?
- STATE_ENTER가 제때 발생하는데 publish stamp만 늦는다면 계산 코드 또는 시계 처리 문제이며, STATE_ENTER 자체가 늦다면 수신/스케줄 경로 문제.

**운영 중인 NEXT나 Canonical에 절대 적용하지 않는다.** 제어·Guard 조건을 바꾸는 실험이 아니라 진단 전용이다. 실행 후 결과는 `~/Downloads/F1TENTH_RAB_LATCH_FORENSIC_*.zip`이며 분석에 필요한 최초 오류 기록과 R-B 콜백 시간 기록을 포함한다.

## 실행 (ROS2 Humble Ubuntu)
```bash
cd ~/Downloads
unzip -nq F1TENTH_RAB_FIRST_LATCH_INSTRUMENTED_RCA_20261017.zip
cd F1TENTH_RAB_FIRST_LATCH_INSTRUMENTED_RCA_20261017
python3 -B VERIFY_FORENSIC.py
bash BUILD_FORENSIC.sh
bash RUN_INTEGRATED_SOLO_CAPTURE.sh
```
메모: `unzip -n`으로 기존 파일을 덮어쓰지 않도록 하고, 최초 생성 디렉터리가 이미 있다면 새 격리 경로를 사용하도록 확인. 75초 제한 + Guard 항상 ON. 20m/s 고속/H2H 검증 아님.
