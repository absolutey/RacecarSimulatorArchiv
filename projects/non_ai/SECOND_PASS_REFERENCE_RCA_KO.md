# RAB Second Pass — S0-R 테스트 2건 근본 원인 및 수정

- 입력: Ubuntu ROS 등록 테스트 508개 중 2개 실패, 나머지 506개 PASS.
- 실패 위치: `f1tenth_racing_reference.node.NominalActiveReferenceNode.receive_plan()`에서 `plan.valid_until`을 분기 전에 참조.
- 원인: 새로운 ABORT/`BLOCKING_HOLD` `COMPLETE` 회귀 테스트의 `SimpleNamespace` 입력은 `valid_until`이 없는 최소 무효화 이벤트. 두 테스트 모두 분기 전 AttributeError로 중단됨.
- 실제 수정: 유효한(`plan.valid`) override 계획일 때만 유효기한을 읽도록 검사 위치를 분리. 본래 `matching_complete()`의 신원 검사 및 ABORT/`BLOCKING_HOLD`의 fail-closed 처리, active override 만료 거부는 변경하지 않음.
- 수정 부위: `src/platform/f1tenth_racing_reference/f1tenth_racing_reference/node.py` 한 파일. 테스트 Fixture와 safety proof를 완화하지 않음.
- 검증: Python AST 로드·분기 테스트, release hash, Ubuntu 선택 테스트(`f1tenth_racing_reference` 50개 예상) 후 508개 전체 테스트.
- 제외: H2H 실제 폐루프, 15ms/7ms 성능, 네 계층 코드 완성은 이번 수정으로 증명되지 않음.
- 기존 2차 ZIP, V7 및 공식 워크스페이스는 수정하지 않음.
