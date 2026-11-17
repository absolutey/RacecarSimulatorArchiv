# F1TENTH 개발 재개

- 시작 소스: 2026-10-19 4계층 통합 후보. 경로상 최대 포함이지만 ROS 구동 합격을 의미하지 않음.
- 4TB/USB 원본은 수정하지 않았음.
- build/install/log는 의도적으로 포함하지 않았음. colcon 빌드 시 재생성.
- 검증 우선순위: Map ICCAS/Spielberg, Guard 독립성, 5.0m/s 고정 cap, S0-R -> R-B 적용, R-C/D/E, H2H.
- `python3 -B F1TENTH_MINIMAL_DEV_BOOTSTRAP_20261022.py --mode verify`로 초기 원본 소스 해시를 검사 가능.
- 코드를 편집하면 초기 해시 검증은 달라지므로 이후 변경은 Git에서 관리 권장.
