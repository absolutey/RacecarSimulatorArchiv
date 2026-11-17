# F1TENTH RAB 실제 구조 구현 후보 — 2026-10-18

## 입력 정본
2차 Reference Hotfix, Ubuntu 17 package/508 registered ROS PASS (사용자 제공 증거). 기존 PASS를 새 코드에 자동 승계하지 않는다.

## 이번 소스 변경 (실행경로)
1. H2H: R-E가 **ABORT를 실제로 발행하기 전에**, 기존 추월 궤적이 충돌 위험으로 부적합해지면 R-D의 신선한 예측으로 _rejoin_geometry + trackability + _temporal_safety를 재계산한다. 검증된 새 MERGE 계획만 R-E→S0-R에 제안, 전부 실패하면 예전처럼 ABORT 유지. 기존 ABORT 래치 이후 새 MERGE로 우회 금지. S0-R은 새 예측 신원/시간/계획 revision 검증하고 Guard는 매 명령 독립 정지증명.
2. R-A/Spielberg: 864점 실제 맵/센터라인 기반 R-A0 최적화, R-A1 동역학 v(s), R-A2 서명식 content-identity 검증 제품을 생성해 설치. 기존 예비 centerline 후보도 분리 보존. 기본 offline baseline을 검증된 A2로 변경. 속도 4.206~20m/s는 '제안'이지 '실제 안전 인가'가 아님. R-A3 sparse solver는 strict feasibility (absolute violation 3.9e-4 > 1e-6)로 실패, 완료로 표기하지 않음.
3. Independent Guard: ROS native-proof 설치 라이브러리와 살아있는 simulator native physics source를 검증한 뒤, 명시 옵션(native_dual_veto_required=true)일 때 기존 Python Red 포함 stop proof와 C++ 정지증명을 **둘 다** 요구. 하나라도 불일치 또는 15ms deadline 지나면 fail-closed. 기존 ROS Python Guard만 actuator final publisher이며 C++ 독립 ROS Guard 본체는 여전히 없음. 기본값 false, 자격검증 전 자동 승격 없음.
4. R-B: 2차에서 구현한 별도 MPC solver-thread/fallback 경계를 그대로 보존. 이번 수정에서 독립 프로세스/7ms WCET를 완료했다고 주장하지 않음.

## 남은 핵심 개발 (코드·실증 구분)
GAP-01 독립 C++ Guard ROS 본체: 미구현. GAP-03/08 15ms·7ms E2E: 미실증. GAP-10 완전 프로세스 격리: 미구현. GAP-14 Spielberg R-A3: 미수렴. GAP-27 ABORT가 이미 래치된 뒤 이동 중 재출발: 금지 (parked 게이트 유지). GAP-30 NMPC: 미구현. H2H 양맵 실주행과 G0~G9 공식 승격: 미실증.

## 테스트 증거
신규 오프라인 R-E 움직이는 MERGE 6개 + 구 Q0 25개 = 31 PASS; Spielberg registry 4 PASS; C++/Python physics 정지증명 1176가지 불일치 0. ROS 종단 검증은 Ubuntu 스크립트 실행 필요. 508은 이전 소스 기준선이며 새 버전 합격 수가 아님.
