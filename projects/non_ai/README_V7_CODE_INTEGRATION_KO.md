# F1TENTH RAB — V7 Native C++ 빌드/배포 및 G0~G9 증거 정합성 수술

## 검증 기준선
- 사용자 Ubuntu: V6 전체 ROS 16/16 packages, **433/433 테스트 PASS**, Applied 별도 7 PASS. 경로/주행/실시간 인증 아님.
- 기존 정상 소스·327→328→338→374→405→433 등록 테스트를 유지한다. 4계층 전체 완성으로 선언하지 않는다.

## 실제 구현 변경
1. `f1tenth_racing_native_proof` 신규 ROS `ament_cmake` 패키지. C++17 shared library `libf1tenth_native_stop.so` 빌드·install. 기존 `native_stop_proof.cpp`를 동일 바이트로 번들하고 CMake configure에서 원본과 SHA256 일치 필수. 복제 소스가 달라지면 빌드 거부. 운영 Guard는 기존 Python 기준이며 자동 native 전환 없음.
2. 설치 바이너리 ABI, null 포인터, NaN/Inf, 잘못된 파라미터, 만료된 deadline, 메모리 생성/해제 등 **23개 native 패키지 테스트**. Test doubles가 아니라 실제 C++ 공유 라이브러리를 ctypes로 호출한다.
3. `RUN_V7_ROS_VERIFY.sh`: 전체 colcon build/test에서 native 엔진 패키지 설치 확인 및 정식 원본과 설치 바이너리 경로 검증. C++/Python parity 추가 실행; parity가 실패하면 Gate FAIL. ROS 등록 총 >=456 (= 기존433 + 신규23), 0 errors/failures/skips 요구.
4. G0~G9 read-only draft assessor: 각 gate/map/run/source별 별도 JSON 실물 증거와 SHA256·metrics 교차결합, 동일 파일 재사용 방지, 숫자 bool/음수/NaN·Inf 거부. 기존 `RAB_G0G9_PROVISIONAL_V1` 영수증은 **V2 정책 아래에서 거부**, 자동 변환하거나 과거 증거를 PASS로 이월하지 않음.

## 독립 환경에서 실제 확인
- C++17 공유 라이브러리 직접 컴파일 및 실제 ABI·오류경계 23 PASS.
- G0~G9 검증기 19 PASS. 합계 42개 선택 집중 테스트 통과. Python/C++ 정지증명 1,176사례 및 GuardAhead 130주기 동등성 검사도 mismatch 0건으로 통과했고, 독립 출력 로그를 함께 포함한다.
- 현재 실행 환경은 ROS Humble이 없으므로 V7 colcon 17개 패키지·최소 456개 Ubuntu 검증 필요.

## 정직한 완료 판정
- GAP-05: native 라이브러리의 ROS 공식 빌드·설치 코드 작성 (기능적 기본 승격/승인 미구현).
- GAP-33: 증거 심사 정합성 향상 (공식 G0~G9 Qualification/promotion 승인 정책 미구현).
- GAP-01/02/03/07/08/09/10/13~16/24/27/30 등 잔여 원래 문제 유지. ROS Guard 전체는 **Python**, Guard 15ms/GuardAhead 7ms E2E 미증명, C++ 강제 적용 금지, H2H 양맵 안전·추월 실증 미완료.

## Ubuntu
`bash RUN_V7_ROS_VERIFY.sh` (격리 워크스페이스에서 실행; 원본 `~/f1tenth_racing` 수정 금지).
