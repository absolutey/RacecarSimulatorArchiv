# F1TENTH RAB — Unified Runtime Contract V2 (2026-10-18)

## 정본 / 완성 판정

- 직접 출발한 기준: `F1TENTH_RAB_UNIFIED_EXECUTION_CANDIDATE_20261018.zip` (사용자 Ubuntu: ROS 16개 패키지 빌드 및 327개 테스트 통과).
- 결과는 **전체 격리 소스를 포함한 새로운 V2 개발 후보**다. Canonical·NEXT·기존 소스는 수정하지 않았다.
- **네이티브 C++ StopProof** 및 R-B/Guard 통합 경로를 실제 소스로 구현했다. 다만 **기본 launch는 원래 Python Guard+GuardAhead**로 동작하며 실험적 네이티브 통합은 기본 비활성이다. 새 후보 자체는 Ubuntu ROS 빌드·Solo·H2H 인증 전이다.

## 한 번의 코드 정합화로 고친 부분

1. `actuator_guard_node.py`: 이전에는 `_certificate()` 실행 **전** ROS/steady 시각을 `contract.validate`에 전달했다. 현재 **증명 완료 후 시각**으로 TTL/상태 freshness를 검증하고, 최종 publish 직전에 다시 검사한다. 만료·시각 역행·receipt 만료는 fail-closed 제동. 15ms 변경 없음.
2. `node.py`: Simulator 25ms 상태 발행과 R-B 25ms 제어 타이머의 불리한 위상을 방지하려고 **5ms 최신-1건 mailbox drain**으로 변경. 물리 상태당 계산은 1회, 가짜 physics_sequence·stamp 생성하지 않음. 이 조치는 실제 OS 스케줄링이나 Guard의 15ms를 *보장하지 않는다*.
3. `native_stop_proof.cpp` + `native_stop_proof.py`: Python `Dynamics` Pacejka+low-speed blend, jerk/steering limits, 25ms stop rollout, 반응시간, 8초 정지지평, footprint padded swept SAT, 위험 마진, 엄격 deadline을 C++17로 별도 구현. 각 엔진은 맵·파라미터의 독립 인스턴스를 가진다. CPU 스케줄러 선점과 RT 우선순위를 변경하지 않는다.
4. `node.py` (R-B): 명시적 `experimental_native_guardahead:=true`와 정확한 바이너리/모델 SHA 확인증이 있을 때만 **GuardAhead proposal**의 current+N+1 full proof를 실험적 C++ 엔진으로 수행. 기존 Python 모델을 여전히 사용해 미래 상태를 구성하고, 증명 결과를 R-B 내부 후보에만 이용한다. 독립 Guard의 최종 승인권을 전혀 대체하지 않는다.
5. `actuator_guard_node.py`: `experimental_native_guard:=true`도 **명시적 Solo 시뮬레이션 실험 전용**이다. Guard는 자신의 독립 맵·현재 물리 상태·시간계약에서 실제 full-stop rollout을 수행한다. R-B의 proof 결과·문자열 등을 승인 근거로 재사용하지 않는다. 시작 시 receipt 검증 실패→기동 fail-closed. 네이티브 예외→인증 실패. H2H 단독 가드 차단 보존.
6. `race_foundation.launch.py`: 실험적인 두 backend는 모두 **기본 false**, 바이너리/receipt 파라미터는 **기본 빈값**. `RUN_INTEGRATED_SOLO_CAPTURE.sh`의 기본 실행 역시 Python 경로이며, 오직 사용자가 `F1TENTH_EXPERIMENT_NATIVE_SOLO=1`로 명시해야 로컬 컴파일·원본 모델 비교 후 시험용 네이티브 경로를 켤 수 있다.
7. 구버전 하드코딩 실행경로/배포 해시 관리: V2 격리 디렉터리 이름, source/release manifest, 회귀 시험 스크립트 및 표적 검증기를 함께 변경했다.

## 실제 로컬(CAAS) 오프라인 검증

- Python/native `StoppingEnvelope`: **1,176 상태/조향/가속 입력, 판정·이유·물리 정지시간·정지거리·동역학 수치·스텝 불일치 0**. 그중 실제 ICCAS 중심선 주변 구성 550건, 추가 random/stress 625건 + 원본 실패 입력 1건.
- 경과된 `deadline_ns`: **30/30 fail-closed**.
- 같은 current+N+1 GuardAhead 후보 탐색: **130개 제어주기, 93개 양측 승인 포함, 선택 명령/인증 이유/판정 불일치 0** (검증용 generous wall budget: *실주행 7ms 자격 주장 아님*).
- 실제 과거 안전한 #252(65스텝), #508(61스텝) 독립 재생: **C++와 Python 동일 `StopCertificate`**. 각 350회 측정 중앙값은 각기 **약 25~27배 개선**. 컨테이너 결과이므로 Ubuntu의 WCET나 CPU 선점 보장은 아님.
- Guard 원본 함수 AST 실행: 정상 승인 및 증명 중 만료, 승인 후 발행 직전 만료, clock regression, state stale 등 **8개** 시험.
- receipt: 로컬 소스·물리맵·바이너리 해시 확인, 결과/소스/바이너리 변조 거부. C++에서 샘플 수치 동등성은 확인했지만 **수학적 형식검증이나 경계 전역 완전성 증명은 아님**.
- 기존 1,356 StopProof+collision Golden, 지도 4,500, 기존 P0/Unified 테스트는 별도 통합 오프라인 게이트에서 수행.

## Ubuntu 사용법 — 먼저 현재 개발 후보 검증

이 ZIP만 `~/Downloads`에 놓고 기존 기준본을 덮어쓰지 않는다.

```bash
cd ~/Downloads
unzip -q F1TENTH_RAB_UNIFIED_RUNTIME_CONTRACT_V2_20261018.zip
cd F1TENTH_RAB_UNIFIED_RUNTIME_CONTRACT_V2_20261018
bash VERIFY_UNIFIED_OFFLINE.sh
bash BUILD_FORENSIC.sh
```

첫 명령은 native C++를 `/tmp`에 컴파일하고 Python 모델 동등성·해시 확인증까지 생성한다. 두 번째는 ROS 16패키지 빌드/회귀시험이다. ROS 빌드 중 327개 테스트는 **실제로 사용자가 실행해 결과를 올리기 전까지 V2 PASS라고 주장하지 않는다**.

### Solo 실험 프로파일 (다음 통합 자격검증에서만 사용)

기존과 동일한 Python 실행(새 타이밍 계약만 적용):

```bash
bash RUN_INTEGRATED_SOLO_CAPTURE.sh
```

독립 Guard와 GuardAhead 각각 네이티브 full proof를 실험적으로 적용:

```bash
F1TENTH_EXPERIMENT_NATIVE_SOLO=1 bash RUN_INTEGRATED_SOLO_CAPTURE.sh
```

후자는 자체 동등성 게이트·receipt 검사를 통과하지 않으면 START 전에 중단된다. 두 프로파일 모두 Guard always ON, `solo:=true`, H2H/interlock 유지, 15ms·7ms 유지. **실주행 실패/충돌/완주 판정 없이 정본 승격 금지.**

## 여전히 미완료인 사항

- 커널 CPU 선점 상대 식별 / isolated core / SCHED_FIFO의 전 시스템 WCET 증명 (C++ 포트 자체만으로는 해결되지 않음).
- Guard proof 15ms / GuardAhead current+N+1 7ms **실제 Ubuntu 최악마감**, 제동 적용 종단간 latency, Deadman process-death qualification.
- 신형 5ms mailbox와 Guard의 exact/one-tick binding이 전체 부하/phase에서 안정적인지 실주행 확인.
- R-A 고속 v(s)와 S0-R/R-B/Guard의 stopping-horizon + 경로별 물리 가능 speed envelope 연결. 임의 5.0m/s cap 복원하지 않음.
- MPC Primary 활성화·SCP/Spielberg 경로/실제 한 랩·C++ H2H dynamic occupancy·R-C/D/E 단일 Red 추월·G0~G9.

**경계:** 이 ZIP은 큰 병목인 full stopping rollout 네이티브 포트와 실행·시간신원 계약을 실제로 코딩한 통합 개발 후보. 전체 F1TENTH 최종 제품 완료/실시간 합격 ZIP은 아니다.
