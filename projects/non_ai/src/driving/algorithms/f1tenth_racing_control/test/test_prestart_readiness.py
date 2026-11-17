from f1tenth_racing_control.node import build_prestart_status


def test_prestart_status_is_readiness_only_and_identity_bearing():
    status = build_prestart_status(
        123456789,
        "sha256:test-reference",
        7,
        0.0,
        "pursuit",
    )
    assert status.receipt_time_ns == 123456789
    assert status.mode == "PRESTART_READY"
    assert status.reason == "WAITING_FOR_RACE_START"
    assert status.active_reference_id == "sha256:test-reference"
    assert status.active_revision == 7
    assert status.actual_speed_mps == 0.0
    assert status.primary_status == "NOT_RUN"
    assert status.fallback_active is False
    assert status.controller_kind == "pursuit"
    assert status.local_speed_limit_reason == "PRESTART_READY"
