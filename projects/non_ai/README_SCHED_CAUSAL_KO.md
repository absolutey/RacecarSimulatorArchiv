# F1TENTH P0 — 실시간 실행 원인 계측 후보 (2026-10-18)

**구조 실행 수술에 앞선 원인 판별 게이트입니다.** 최근 원본 Intent #184: Guard 17.778ms wall / 2.595ms thread CPU, CPU 외 15.183ms. 오프라인 60스텝 정지증명 qualified=True였지만 온라인 15ms DEADLINE 거부. GuardAhead 145/1230회 7ms 초과, CPU 자체 초과 10회. START/완주 테스트 PASS 아님.

## 이번에 실제 수정한 소스(세 파일)

- Guard `_certificate`: 계산 전후 **동일 스레드**의 `resource.getrusage(RUSAGE_THREAD)` 문맥전환 카운터를 읽어 인증 실패 스냅샷에 `voluntary_context_switches`, `involuntary_context_switches`를 기록. 인증 성공/실패 조건, 15ms와 물리 StopProof 무변경.
- GuardAhead `choose`: 동일 문맥전환 카운터를 결과에 추가. 변경 없는 후보 순서와 **현재+N+1 전수 증명**, 7ms 유지.
- `final_preflight`: 측정 카운터를 기존 컨트롤러 계산 텔레메트리에 전달. 안전결정에는 사용하지 않음.

새 외부 `/proc/*/task/*/schedstat` 샘플러는 **10ms 표본**을 선택적으로 읽고 `ANALYZE_SCHED_CAUSAL.py`가 첫 Guard 실패와 시간상 가까운 실행/대기 변화량을 연결합니다. 이것만으로 커널 `sched_switch`에 해당하는 정확한 경합 시간을 증명할 수는 없습니다. 의심이 좁혀지면 Ubuntu에서 별도 kernel trace가 필요합니다. 진단 도구는 CPU affinity, priority, realtime policy, ROS control publisher를 변경하지 않습니다.

## Ubuntu 사용법 (기존 후보 폴더 및 NEXT/Canonical 비수정)

```bash
(
set -euo pipefail
cd ~/Downloads
ZIP=F1TENTH_RAB_P0_SCHED_CAUSAL_INSTRUMENTED_20261018.zip
DIR=F1TENTH_RAB_P0_SCHED_CAUSAL_INSTRUMENTED_20261018
# 다운로드 ZIP SHA를 함께 제공한 값과 비교한다.
sha256sum "$ZIP"
test ! -e "$DIR"
unzip -q "$ZIP"
cd "$DIR"
python3 -B VERIFY_RELEASE.py
python3 -B TEST_SCHED_INSTRUMENTATION.py
python3 -B TEST_P0_EXECUTION_PATH.py
bash BUILD_FORENSIC.sh
F1TENTH_SCHED_DIAG=1 F1TENTH_SCHED_SAMPLE_MS=10 bash RUN_INTEGRATED_SOLO_CAPTURE.sh
)
```

결과 `~/Downloads/F1TENTH_RAB_P0_SCHED_SOLO_*.zip` 내부의 `scheduler_trace.jsonl`, `scheduler_attribution.json`, `actor_traces/GUARD_*.jsonl`, `full_topic_trace/compute_full.jsonl`를 교차 검증합니다. **진단 오버헤드가 관측값에 영향을 줄 수 있습니다.** 판정 전에는 원본 Intent #184를 포함한 과거 최신 P0 Solo ZIP과 비교하십시오. 새로운 실행에서 15ms 초과가 없어도 단일 주행으로 최악 시간 보장을 주장하지 않습니다.

## 합격 게이트

- H2H 실행차단과 Guard 15ms/GuardAhead 7ms 유지
- 원본 1,356개 안전 Golden 시험 결과 불변
- Test Tick(14), Execution Path(8), New instrumentation(5) 모두 PASS
- Guard/Pursuit/R-B 콜백 타이밍 기록이 일관되게 수집됨
- 최초 실패 전후 voluntary/involuntary + wait/cpu 변화량으로 **scheduler vs blocking**에 대한 관측 근거 확보. 불확실하면 미확정
- 추후 독립된 '상태 수신과 계산 경로 분리'는 executor 안전성·메시지 freshness·ROS QoS·deadline 증거를 기반으로 설계하고 별도 통합/실주행 시험을 통과해야 함

**모든 스레드의 실행 우선순위 변경, 15/7ms 제한 확대, 무인증 Guard 승인, 임의 5.0m/s 속도 캡 부활 금지.**
