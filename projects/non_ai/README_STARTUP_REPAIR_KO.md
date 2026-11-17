# F1TENTH RAB Startup Contract Repair — 실제 소스 업데이트

## 코드 변경과 안전한 범위

- 기반: 2026-10-17 계측 후보 **272개 기준 파일**, 작동하지 않았던 reference exact-path whitelist를 **실행 중인 S0-R 패키지 소스가 가진 동일 스테이지 신원 검증**으로 교체. `Downloads`의 폴더 이름만으로 신뢰하지 않으며 **module source == installed artifacts stage**까지 강제한다.
- S0-R: 기존 CSV SHA256, map, trajectory identity와 symlink-traversal 검사 그대로 유지. 타 후보 소스에서 다른 후보의 data를 승인하지 않는다.
- Guard: invalid reference 수신 시 과거 reference ID를 지워 재무장 차단; 사전 reference invalid의 진짜 사유 표시; 최초 `_brake()` 기록 별도 보존.
- 실행: ROS 빌드·전체 colcon 회귀시험 후 진짜 설치 share 폴더에서 S0-R 전체 정책 해시 로딩 시험, **실제 ROS state+S0-R+Guard+R-B** prestart 상태검사 통과 전 `/race_start` 발행하지 않음.
- Controller 종방향/Guard StopProof/100ms 신선도/40ms future-tick/H2H interlock/기존 fault injection 조건을 변경하지 않았다.

## Ubuntu 단계

```bash
cd ~/Downloads
sha256sum -c F1TENTH_RAB_STARTUP_CONTRACT_REPAIR_20261018.zip.sha256
unzip -q F1TENTH_RAB_STARTUP_CONTRACT_REPAIR_20261018.zip
cd F1TENTH_RAB_STARTUP_CONTRACT_REPAIR_20261018
python3 -B VERIFY_RELEASE.py
bash BUILD_FORENSIC.sh
bash RUN_INTEGRATED_SOLO_CAPTURE.sh
```

**주의**: `BUILD_FORENSIC.sh`는 빌드만 수행하며 `ROS build PASS`가 주행 성공을 의미하지 않는다. `RUN_INTEGRATED_SOLO_CAPTURE.sh`에서 `PREFLIGHT_ARTIFACTS.py` / `PREFLIGHT_LIVE.py`가 실제 ROS에서 모두 합격해야 START 요청한다. START 후 481ms R-B 지연은 여전히 미해결. 이번 계측에서 재현 여부를 확인하고 원인을 좁힌다.

원본 NEXT/Canonical/기존 진단 후보는 변경하지 않는다. H2H 인터락 유지. 실패 시 반드시 생성된 증거ZIP을 제공하고 임의로 timeout을 늘리지 않는다.

## 남은 검증

- 이 컨테이너에는 ROS2 Humble이 없어 16패키지 실빌드/ROS 테스트는 확인 불가. Ubuntu에서만 실행 가능하다.
- 패키지의 안전성 합격 여부는 **실제 START 이후 증거**를 분석해야 판정한다.
- PRESTART_LIVE 실패나 외부 노드 기동 타이밍 지연 시 START를 요청하지 않고 ZIP에 증거를 저장한다.
- S0-R은 수정된 코드의 설치 provenance를 신뢰 기점으로 한다. 이 방식은 소스 코드의 암호학적 서명/배포체계가 아니며, 로컬 코드가 변조됐는지는 `VERIFY_RELEASE.py`로 별도 검증한다.

## 남은 검증

- 이 컨테이너에는 ROS2 Humble이 없어 16패키지 실빌드/ROS 테스트는 확인 불가. Ubuntu에서만 실행 가능하다.
- 패키지의 안전성 합격 여부는 **실제 START 이후 증거**를 분석해야 판정한다.
- PRESTART_LIVE 실패나 외부 노드 기동 타이밍 지연 시 START를 요청하지 않고 ZIP에 증거를 저장한다.
- S0-R은 수정된 코드의 설치 provenance를 신뢰 기점으로 한다. 이 방식은 소스 코드의 암호학적 서명/배포체계가 아니며, 로컬 코드가 변조됐는지는 `VERIFY_RELEASE.py`로 별도 검증한다.
