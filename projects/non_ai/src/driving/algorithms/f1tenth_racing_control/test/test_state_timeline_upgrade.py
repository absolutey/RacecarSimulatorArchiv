"""Pure state-sequence and monotonic budget fault-injection tests (no ROS)."""
from f1tenth_racing_control.state_timeline import (
    StateTimeline, StateReceipt, ReceiptStatus, BindingStatus)
from f1tenth_racing_control.cycle_trace import StageTimer


def state(seq, rx_ns=None, ros_ns=None):
    return StateReceipt(seq, ros_ns if ros_ns is not None else seq * 25_000_000,
                        rx_ns if rx_ns is not None else seq * 25_000_000)


def test_exact_future_tick_never_preapproved():
    timeline = StateTimeline(depth=16)
    assert timeline.append(state(832)) == ReceiptStatus.INSERTED
    assert timeline.binding(833, 833 * 25_000_000, 833 * 25_000_000,
                            833 * 25_000_000) == BindingStatus.PENDING_ONE_FUTURE
    assert timeline.append(state(833)) == ReceiptStatus.INSERTED
    assert timeline.binding(833, 833 * 25_000_000, 833 * 25_000_000,
                            833 * 25_000_000) == BindingStatus.EXACT_LATEST


def test_gap_832_to_835_cannot_retroactively_authorize_833():
    ledger = StateTimeline()
    ledger.append(state(832))
    assert ledger.append(state(835)) == ReceiptStatus.GAP_OBSERVED
    assert ledger.missing_ranges == [(833, 834)]
    assert ledger.binding(833, 833 * 25_000_000, 835 * 25_000_000,
                          835 * 25_000_000) == BindingStatus.MISSING
    assert ledger.total_received == 2 and ledger.gap_count == 1


def test_old_snapshot_is_not_new_actuation_authority():
    ledger = StateTimeline(depth=16)
    ledger.append(state(1))
    ledger.append(state(2))
    assert ledger.exact(1, 25_000_000) is not None
    assert ledger.binding(1, 25_000_000, 50_000_000, 50_000_000) == BindingStatus.HISTORICAL_NO_AUTHORITY


def test_physics_stamp_spoof_rejected():
    ledger = StateTimeline()
    ledger.append(state(1))
    assert ledger.binding(1, 25_000_001, 25_000_000, 25_000_000) == BindingStatus.STAMP_MISMATCH


def test_ros_and_steady_expiry_separate():
    ledger = StateTimeline(max_age_ns=100_000_000)
    ledger.append(state(1, rx_ns=100, ros_ns=1_000))
    assert ledger.binding(1, 1_000, 100_002_000, 100) == BindingStatus.STALE_ROS
    assert ledger.binding(1, 1_000, 1_000, 100_000_101) == BindingStatus.STALE_STEADY


def test_negative_clock_difference_rejected():
    ledger = StateTimeline()
    ledger.append(state(1, rx_ns=1_000, ros_ns=1_000))
    assert ledger.binding(1, 1_000, 999, 2_000) == BindingStatus.BAD_TIME


def test_sequence_and_stamp_regressions_preserve_ledger():
    ledger = StateTimeline()
    ledger.append(state(1, 25, 25))
    assert ledger.append(state(1, 30, 30)) == ReceiptStatus.SEQUENCE_REGRESSION
    assert ledger.append(state(2, 31, 24)) == ReceiptStatus.STAMP_REGRESSION
    assert ledger.last_sequence == 1 and ledger.total_received == 1


def test_bounded_retention():
    ledger = StateTimeline(depth=2)
    for seq in (1, 2, 3):
        assert ledger.append(state(seq)) == ReceiptStatus.INSERTED
    assert ledger.exact(1, 25_000_000) is None
    assert ledger.exact(3, 75_000_000) is not None


def test_monotonic_budget_receipt():
    readings = iter([100, 106, 200, 208])
    timer = StageTimer('guard_ahead', 7, now=lambda: next(readings))
    assert timer.start().finish().within_budget
    r = timer.start().finish()
    assert r.elapsed_ns == 8 and not r.within_budget


def test_invalid_depth_never_accepted():
    for depth in (0, 1, 257):
        try:
            StateTimeline(depth=depth)
        except ValueError:
            pass
        else:
            raise AssertionError(depth)
