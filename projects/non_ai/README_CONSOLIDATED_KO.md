# F1TENTH RAB — Consolidated Implementation, 2026-10-18

## 작업 정본
기준: `F1TENTH_RAB_UNIFIED_RUNTIME_CONTRACT_V2_20261018.zip`, 사용자 Ubuntu ROS 16패키지 + 327 단위시험 PASS 로그. 이 폴더는 격리 신규 후보로 Canonical/NEXT를 변경하지 않는다. **제품 완성본이나 실시간 자격인증본이 아니다.**

## 실제 추가·수정된 코드

1. `execution_speed_authority.py` + R-B `node.py` 실제 경로: R-A에서 S0-R이 승인한 공간가변 `v(s)` 원본을 복제하고, **실행 제안 속도만** 현재 `StoppingEnvelope` 지평(8초), reaction(0.25초), 제동 2m/s², 실제 차량 가속한계, Guard lateral limit, 곡률, 폐회로(마지막→첫 샘플)의 전·후방 도달가능성에 맞춰 낮춘다. 이는 필요한 제약일 뿐 최종 안전인증이 아니다. Full current/N+1 GuardAhead + 독립 Guard 전체 증명은 그대로 ON. 임의 5.0m/s 캡을 새로 넣지 않는다. nonfinite/역행/불가능한 입력은 참조 자체를 fail closed 거부. 속도 감사 결과는 기존 controller 실행 provenance로 발행하며 인증권한 없음.
2. 원본 ICCAS R-A2/R-A3 CSV(약 43m 폐회로)를 사용한 순수 수치평가: authored 최고 19.65/19.96m/s, 새 필요조건 반영 최고 7.43/7.32m/s. 이는 해당 트랙의 곡률·2m/s² 제동 제약에서 나타나는 수치이지 자의적 정속캡이 아니다. 실제 주행이 이 값으로 합격됐다는 의미도 아니다.
3. `native_stop_proof.py`, R-B/Guard `node.py`: 동등성 receipt와 **실제 활성 PGM 맵 SHA256까지** 일치할 때만 실험적 C++ 엔진 사용. ICCAS만 대조한 receipt를 Spielberg에서 재활용하면 기동 거부. H2H fail-closed 차단 유지. 기존 native parity 1176 + current/N+1 130 보존.
4. `dynamic_occupancy_certificate.py`: 단일 Red 예측 점유와 Ego 경로의 시간 신선도·연속성·보수적 회전 무시 원판 충돌/상대 swept segment 교차를 거부하는 순수 테스트 빌딩블록. **Guard H2H에 연결하지 않음**: C++/ROS 상태 신원·예측 전체 8초 커버·벽 충돌·정지가능성·불확실성의 독립 인증이 부족해 SOLO interlock 유지.
5. `physical_lap_oracle.py`: 자기신고 `lap=1`을 완주로 오인하지 않는 별도 시작선 2회와 진짜 전진 1바퀴, 트랙 corridor, 충돌·역주행·도약 거부 독립 검증 도구. 실제 주행 결과와 자동 연결은 추후 작업.
6. `RUN_NATIVE_SOLO_INTEGRATED.sh`: 기존 ROS 전체 overlay를 사용한 **명시적 시뮬레이터 Solo 네이티브 실험 프로파일**. 오프라인 모델/모든 SHA receipt 실패시 START 전에 실패하고, 최종 Guard는 계속 독립적으로 모든 명령을 인증한다. 실제 실행·안전의 최악 시간·양맵 완주 검증은 별도.

## 이번 후보의 자동검증과 남은 한계

- 추가 소스 기반 Python 15 경계시험(Python unit tests, H2H geometry/랩/속도); native library active-map receipt fault-injection.
- 기존 native C++ full stop 1,176 cases, GuardAhead current/N+1 130 cycles, 기존 1,356 Golden + 4,500 지도, 기존 V2 계약 테스트를 단일 `VERIFY_UNIFIED_OFFLINE.sh`로 묶음.
- **이 후보는 Ubuntu ROS 327 재실행과 실제 Solo·Spielberg·H2H 완주를 아직 수행하지 않았음.** 예전 PASS는 이 새 소스의 PASS가 아님.
- Guard node 자체는 Python ROS; native는 정지 시뮬레이션 커널. OS WCET, 15ms Guard와 7ms GuardAhead 내 실행, publisher→Simulator latency 미확정. MPC Primary 채택과 실제 Red 추월, 전체 G0~G9 역시 미완.

## Ubuntu

```bash
cd ~/Downloads
unzip -q F1TENTH_RAB_UNIFIED_CONSOLIDATED_IMPLEMENTATION_20261018.zip
cd F1TENTH_RAB_UNIFIED_CONSOLIDATED_IMPLEMENTATION_20261018
bash VERIFY_UNIFIED_OFFLINE.sh
bash BUILD_FORENSIC.sh
# ROS 빌드와 오프라인 동등성 성공 후 명시적 Solo 시뮬레이터 검증만 수행
bash RUN_NATIVE_SOLO_INTEGRATED.sh
```

Canonical 소스 덮어쓰기·기존 ZIP 삭제·H2H 인터락 비활성 금지. 결과가 실주행에 실패하면 첫 인과 오류를 분석하고 해당 부위 전체를 수정한다. 인증시간 제한을 늘리거나 오래된 5m/s 캡을 복구하지 않는다.
