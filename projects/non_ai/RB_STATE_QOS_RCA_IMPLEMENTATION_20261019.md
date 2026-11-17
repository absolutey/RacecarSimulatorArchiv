# F1TENTH RAB — R-B state0 QoS RCA 근본 수정 (2026-10-19)

## 원본/문제와 출처
- 부모 ZIP: `F1TENTH_RAB_SOLO_RUNTIME_RCA_REPAIR_20261019.zip`, SHA256 `6c56636a6152839603cf963820da24a179454ae1b2a97849a7a0f66cf30124de`.
- 실패 증거: `F1TENTH_RAB_CONSOLIDATED_SOLO_20261019T151200.zip`, SHA256 `fa5d5515b6419f7ddccf6d4f9415f374b4ff1efe50d43c04f77fa49772ee136b`.
- 그 증거의 R-B `state0` 수신 지연: 443개 전부 75ms 초과, 중앙값 357.215ms, max 425.261ms. Guard 동일 토픽 수신 중앙값 0.51ms.
- 동일 실행환경 QoS 분리실험: `BEST_EFFORT/depth1` 중앙값 1.019ms / max 12.258ms / 75ms 위반 0/1,438, `RELIABLE/depth16` 중앙값 0.863ms, `RELIABLE/depth1` 중앙값 361.995ms / 위반 359/359.
- 기존 ROS 코드 회귀는 통과했으나 실제 Solo 상태 처리 0회, Guard CERTIFIED 0회, Ego v=0이었다.

## 실제 수정
1. `f1tenth_racing_control/node.py`의 R-B `state0` 구독에서 암시적 `QoSProfile(depth=1, RELIABLE)`을 명시적 `BEST_EFFORT/depth=1`로 변경한다. 최신 상태 우선 정책에 부합한다.
2. R-B 상태 수신 콜백은 최근 8개 상태의 ROS 생성→수신 지연과 마지막 수신 시각·시퀀스를 상수시간으로 저장한다. 기존 250ms worker 진단 타이머에서만 JSON 발행한다. Guard의 별도 `RELIABLE/depth16` 구독은 그대로이다.
3. `PREFLIGHT_LIVE.py`는 START 전에 실제 R-B·Simulator DDS endpoint의 이름·reliability·durability, 그리고 최근 8개 R-B 수신 지연이 모두 75ms 이내인지 확인한다. 워커 진단 불일치/미수신/과거 상태라면 **START 서비스 호출 없이 실패**한다. Guard의 상태 수신이 정상이어도 R-B가 비정상인 경우를 걸러낸다.
4. DDS 미들웨어는 endpoint `History (Depth): UNKNOWN`을 반환할 수 있다. 이 경우 reliability와 node identity는 반드시 확인하되 depth의 `0/UNKNOWN`만 출처 해시로 고정된 실제 구독 코드의 depth와 대조한다. 관측치 없는 depth를 측정했다고 주장하지 않는다.
5. 유효·무효·누락·위조·오래된 입력/QoS 계약 신규 pytest 27개 등록. 설치된 파이썬 소스와 비격리 canonical 작업환경은 수정하지 않는다.

## 안전 요구사항 변경 없음
- 제어 계산의 **75ms** 원본 시간 제한 보존. `state_freshness_s`를 늘리지 않는다.
- Guard 정지증명 **15ms**, GuardAhead **7ms** 원본 계약 보존. 검증기/계약 우회 금지.
- 기본 실행 속도 cap 변화 없음, 임의 5.0m/s 캡 재도입 없음.
- 독립 Guard watchdog·비상 제동·원자적 Applied 승인 그대로. START 차단 관문은 Guard가 아닌 실행 사전검사이다.

## 검증과 완료 경계
- 이 도구 환경에서 가능한 부분: 순수 Python 신규 27개 테스트, 소스 컴파일, 정본 SHA/ZIP CRC 및 실행기 정적 확인.
- Ubuntu ROS Humble 필요: 선택 패키지 colcon, 전체 ROS 17패키지 668개 이상 등록 테스트 0 실패/스킵, 변경된 `PREFLIGHT_LIVE.py`가 실제 ROS endpoint를 수집하는지 확인.
- Solo 재실증은 코드 회귀를 통과한 후 별도로 실행. v>0, Guard CERTIFIED, 75ms 위반 0, 7/15ms, 충돌/랩, 이동 중 emergency brake 적용을 원본 증거로 검사해야 함.
- H2H 완주·실시간 WCET·R-A3 최종 수렴에 대해서는 이 수정으로 자격검증을 주장하지 않는다.
