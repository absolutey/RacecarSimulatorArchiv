# F1TENTH RAB — P0 통합 구조 보정 (2026-10-18)

## 정확한 최신 기준

- Guard RT 통합 소스 SHA256 `2db1342435d9d9c3486494cd32a89225cc6dd7741799ad19e28f59badcf387ce`
- 실주행에서 적용된 Startup Pinfix `fast_stop_proof.py` SHA256 `b7052b2ad6109b99eef8cd65e175158baa961682bdc37f539cb0b0ecc7a7da3f`
- 최신 설계–실제 구현 감사 23항목 및 Guard RT Solo `20261018T011458` 실패 증거

## 실제 완료한 P0 소스 수술

1. Pinfix 1개 소스를 전체 247개 관리 소스에 통합. `StoppingEnvelope.evaluate`의 구형 복사본 없이, 검증된 단일 본체 상속. 원본/최적화 지도 소스 해시 두 개 검증은 유지.
2. `race_foundation.launch.py`: 기본 무인자 실행을 안전한 Solo/Pursuit/baseline으로 지정. `execution_profile` 검사로 H2H 미자격 조합을 실행 전에 거부. 실험용 Solo SCP / Solo MPC는 명시적 설정이 있어야만 선택 가능하며 여전히 `EXPERIMENTAL`이다.
3. 전체 소스/실행 스크립트 동일 스테이지 경로 정합화. 빌드 후 모든 colcon 패키지의 설치 origin 확인.
4. 배포 정적 해시에서 `.pytest_cache` 등 생성물을 제외하여 빌드→재검증 때 정적 파일이 오염된 것으로 오판하지 않음. 안전 소스·설정·실행 도구 해시는 계속 검사.
5. 선택형 비파괴 `/proc` 스케줄링 추적기 추가. 별도 CPU/우선순위 조작 없음. baseline 주행에서 기본 **비활성** (`F1TENTH_SCHED_DIAG=1`에서만 수집).

## 실행 프로파일

- 기본 `SOLO_BASELINE`: `solo=true`, `pursuit`, `baseline`; 최적 랩이라고 주장하지 않음
- `SOLO_SCP_EXPERIMENTAL`: `solo=true`, `pursuit`, `scp`; 자체 지도 최적경로 주행 미검증
- `SOLO_MPC_EXPERIMENTAL`: `solo=true`, `mpc`, `scp`; 25ms 내 제어 지속 미검증
- `solo=false` / H2H perception/prediction/overtake 옵션은 런치 구성 단계에서 거부. 독립 Guard의 2차 H2H 차단도 그대로 유지

`controller_kind` 또는 `trajectory_candidate`만 따로 변경하면 프로파일과 맞지 않아 실패한다. 예: `execution_profile:=SOLO_MPC_EXPERIMENTAL controller_kind:=mpc trajectory_candidate:=scp`. 단, 실제 qualification 없는 상태이므로 기동 가능성과 안전한 완주를 혼동하지 말 것.

## 격리 Ubuntu 실행

```bash
(
set -euo pipefail
cd "$HOME/Downloads"
ZIP=F1TENTH_RAB_P0_ARCHITECTURE_INTEGRATED_20261018.zip
DIR=F1TENTH_RAB_P0_ARCHITECTURE_INTEGRATED_20261018
# ZIP SHA256은 다운로드 화면 본문의 고정 체크섬으로 검증
sha256sum "$ZIP"
test ! -e "$DIR"
unzip -q "$ZIP"
cd "$DIR"
python3 -B VERIFY_RELEASE.py
python3 -B TEST_P0_ARCHITECTURE.py
bash BUILD_FORENSIC.sh
# 위 단계가 모두 성공한 경우에만:
bash RUN_INTEGRATED_SOLO_CAPTURE.sh
)
```

빌드는 16개 패키지의 ROS 설치 overlay 신원 및 full `colcon test`를 검사한다. 실주행 결과 `~/Downloads/F1TENTH_RAB_P0_SOLO_*.zip`을 분석해야 다음 판단이 가능하다.

추가 진단이 필요하면 *별도의 새 ROS 실행*에서 `F1TENTH_SCHED_DIAG=1 bash RUN_INTEGRATED_SOLO_CAPTURE.sh`를 사용한다. `/proc/<pid>/task/<tid>/schedstat`가 0이거나 OS 권한에 막히면 분석 가능 범위가 제한된다. 최초 주행에서 진단 부하를 얹지 않도록 기본 비활성이다.

## 미완료·주의

이 릴리스는 **Guard 15ms 초과 해결 패키지가 아니다**. `P0` 중 소스 통합/실행 구성을 해결했고, CPU 실행시간 부족의 구조적 원인은 아직 실험이 필요하다. GuardAhead 7ms, MPC Primary 실제 채택, ICCAS/Spielberg 물리랩, 고속 v(s)와 안전 stop envelope, H2H 모두 미인증이다. 기존 최신 주행은 3.061m/s/6.35m/첫 거부 +2.574초의 불합격 런이다.

Canonical, NEXT, Archive, 과거 ZIP에 대한 변경은 없다.
