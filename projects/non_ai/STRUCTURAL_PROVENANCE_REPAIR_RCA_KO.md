# RAB Spielberg R-A2 등록 실패 RCA 및 수정 보고서 (2026-10-19)

## 실측 실패
- Ubuntu 구조 수정본 선택 검사: 5개 패키지, 424 등록 테스트, 4 실패, 0 오류/skip.
- 실패 4개 모두 `f1tenth_racing_reference/test/test_spielberg_ra2_registration.py`.
- 예외: `ValueError: candidate registry not inside executing verified stage`.

## 확정된 근본 원인
`contract_guard._trusted_stage_provenance`가 `stage.parent == ~/Downloads`만 허용하여,
`~/Downloads/RAB_STRUCTURAL_VERIFY_XXXX/F1TENTH_RAB_UNIFIED_CONSOLIDATED_IMPLEMENTATION_20261018`
처럼 한 단계 격리해서 푼 정본을 거부하였다. R-A2 CSV 손상이나 지도 해시 불일치 증거는 없다.

## 코드 수정
- `contract_guard._permitted_stage_location`에서 `~/Downloads` 아래 중첩 정본을 허용.
- 허용 정본 이름 `F1TENTH_*`와 기존 `~/F1TENTH_NEXT_WORKSPACE` 규칙 유지.
- 실제 실행 모듈이 바로 그 정본의 `src/.../contract_guard.py`인지 검사 유지.
- stage/소스/설치 경로의 symlink 이탈, 타 정본 경로, 파일명 traversal, 맵/CSV SHA 검증 유지.
- 기존 경로·알고리즘·안전승인 정책의 완화/롤백 없음.

## 교차검증
- 신규 17개 오프라인 테스트 PASS: 정상/중첩 stage, 다른 정본, symlink 조작, 파일 변조, 잘못된 맵 SHA, 실제 Spielberg R-A2 등록.
- 실제 별도 Downloads 격리 디렉터리 안에 production `contract_guard.py`, JSON/CSV, 가상 symlink-install 경로를 복제해 로드: `REAL_NESTED_STAGE_SPIELBERG_POLICY_PASS 2`.
- 실제 Ubuntu ROS 535개 전체 테스트는 아직 실행 전. 앞선 508 PASS는 이 변경본의 승인 증거로 전용하지 않음.

## 유효 범위 및 미완성
이 수정은 Spielberg 후보 레지스트리 **provenance admission**만 다룬다. 고속 주행 권한, 양맵 트래킹, R-A3 SCP 최적화,
Guard 15 ms, GuardAhead 7 ms, H2H end-to-end 완성/실증을 입증하지 않는다.
