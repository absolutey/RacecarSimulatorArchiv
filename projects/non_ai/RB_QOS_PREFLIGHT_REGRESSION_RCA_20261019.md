# F1TENTH RAB — R-B QoS preflight regression RCA (2026-10-19)

## Evidence and failure

The latest Ubuntu run in `RAB_QOS_VERIFY_naKWbYeI` stopped in
`f1tenth_racing_bringup/test_capture_offline_preflight`: two registered cases
failed. `TEST_UNIFIED_EXECUTION.py` had five AttributeErrors and
`TEST_RUNTIME_CONTRACT_V2.py` had one AttributeError. The new R-B
`receive_state_snapshot()` records three diagnostic-only fields on every
monotonic incoming physics state, but its source-executing offline `Stub`
created before this change lacked them.

## Exact repair

In `TEST_UNIFIED_EXECUTION.py`, initialize the three existing live R-B
attributes under the existing `_state_receipt_lock` abstraction:
`_state_rx_recent_ages_ns=deque(maxlen=8)`, `_state_rx_receipt_count=0`,
`_state_rx_last_receipt_steady_ns=-1`. Do not stub out the callback or
replace the 75ms pre-compute freshness test. Add regression cases verifying
8-sample bounded telemetry and that stale states never emit commands.
`TEST_RUNTIME_CONTRACT_V2.py` imports this `Stub`, so it repairs the dependent
5ms mailbox test without changing its intended semantics.

## Safety / preservation

There are **no changed files under `src/`** compared with the preceding
R-B QoS package. R-B state0 remains BEST_EFFORT/depth 1; independent Guard
subscription and safety authority, 75ms state freshness, Guard 15ms deadline,
and GuardAhead 7ms deadline are all untouched. No safety or H2H qualification
is claimed from these offline tests. The root-level verification wrapper now
runs every offline Solo prerequisite **before** the ROS build and only proceeds
to selected and complete colcon suites after it succeeds.

## Locally verified

- `TEST_UNIFIED_EXECUTION.py`: 11/11
- `TEST_RUNTIME_CONTRACT_V2.py`: 8/8
- `pytest .../test_capture_offline_preflight.py`: 10/10
- Source SHA + exact release inventory + extracted ZIP CRC: checked at packaging.

Ubuntu colcon and live Solo behavior must still be confirmed using the
isolated wrapper; do not equate a START acknowledgment with a completed lap.
