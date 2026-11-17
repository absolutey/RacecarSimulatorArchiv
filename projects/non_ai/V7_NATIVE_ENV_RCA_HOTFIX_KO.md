# F1TENTH RAB V7 Native Test ENV 정정 — RCA / 배포 경계

## 결론
- 기존 2026-10-18 V7 로그에서 전체 457 tests / 24 failures.
- 23개의 f1tenth_racing_native_proof 테스트가 공통으로 ':/home/neo/.../*.so' 또는 ':/home/neo/.../*.cpp' 경로를 따라 실패.
- 원본 CMakeLists.txt의 `ament_add_pytest_test(... APPEND_ENV ...)`는 스칼라 파일 경로에 기존 환경변수 값을 이어 붙이는 잘못된 옵션. `ENV`로만 변경.
- 전체 24개 가운데 나머지 1개는 제공된 출력 구간으로 테스트명과 근본원인을 식별할 수 없음. `DIAGNOSE_V7_TEST_FAILURES.py`로 Ubuntu build의 XML를 읽어 별도 분류.
- 이 보정은 로컬 C++ 증명 계산을 변경하지 않으며 네이티브 Guard 실행기, 15ms/7ms 타이밍, H2H 기능을 완성하지 않음.
- 기존 V6 433/433 PASS는 유지, 원본 V7은 비교 증거로 보존.

## 변경 대상
`src/platform/f1tenth_racing_native_proof/CMakeLists.txt`: `APPEND_ENV` => `ENV` (단일 치환). Python 테스트/Native .cpp/Guard/메시지/launch 불변.
`SOURCE_MANIFEST.json` 및 `RELEASE_HASHES.json`은 변경 내용으로 재생성.

## Ubuntu 절차
1. ZIP SHA256와 압축 무결성 검사, 격리 작업공간으로 해제.
2. `bash RUN_V7_NATIVE_SELECT_VERIFY.sh` : 소스 릴리스 검증, **native 패키지만** 빌드/테스트(23/23 요구).
3. 실제 V7 최초 실패 워크스페이스에서는 `python3 DIAGNOSE_V7_TEST_FAILURES.py /home/neo/Downloads/F1TENTH_RAB_V7_VERIFY_APonTJGH/F1TENTH_RAB_UNIFIED_CONSOLIDATED_IMPLEMENTATION_20261018`로 24번째의 테스트 이름·결과 확인.
4. 이후 전체 V7 게이트를 원하면 `bash RUN_V7_ROS_VERIFY.sh`. 미해결 24번째 실패가 있으면 PASS로 간주하지 말고 해당 소스만 수정할 것.
