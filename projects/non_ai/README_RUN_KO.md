# P0 고정 물리스텝 상태결속 수술 — 실행 안내

1. 기존 P0 후보는 변경하지 않는다. 다운로드 ZIP을 `~/Downloads/`에 저장한다.
2. 아래 코드로 SHA 검증 후 새 폴더에 압축 해제한다.
3. `python3 -B VERIFY_RELEASE.py`, `python3 -B TEST_GUARD_TICK_BINDING.py`, `python3 -B TEST_OLD_VS_NEW_GUARD_CERT.py`, `python3 -B REPLAY_P0_TICK_STOP_PROOF.py` 실행.
4. `bash BUILD_FORENSIC.sh`: Golden 1356건 + ROS 전체 16패키지 테스트.
5. `bash RUN_INTEGRATED_SOLO_CAPTURE.sh`: START 전 비정상 참조·Guard 차단을 보존한 Solo 실주행. 결과 `F1TENTH_RAB_P0_TICK_SOLO_*.zip`.

중요: 정지증명과 모든 안전 예산을 유지한다. 단일 #252 재현의 PASS는 실제 Solo 완주 합격을 의미하지 않는다. 실제 Ubuntu 결과를 다시 검토해야 한다.
