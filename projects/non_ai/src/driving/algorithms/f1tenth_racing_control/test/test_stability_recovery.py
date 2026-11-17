import pytest

from f1tenth_racing_control.stability import CausalEntryDetector, rate_limited


def test_rate_limited_obeys_rate_bound():
    assert rate_limited(0.0, 1.0, 4.0, 0.025) == pytest.approx(0.1)
    assert rate_limited(0.0, -1.0, 4.0, 0.025) == pytest.approx(-0.1)


def test_causal_entry_triggers_immediate_nonpositive_margin():
    detector = CausalEntryDetector()
    assert detector.update(0.0, 0.8, 0.8) is False
    assert detector.update(0.025, 0.5, -0.01) is True


def test_causal_entry_uses_past_only_100ms_slope():
    detector = CausalEntryDetector()
    assert detector.update(0.00, 0.8, 0.8) is False
    assert detector.update(0.05, 0.7, 0.7) is False
    assert detector.update(0.10, 0.3, 0.3) is True


def test_causal_entry_does_not_trigger_flat_positive_margin():
    detector = CausalEntryDetector()
    for index in range(8):
        assert detector.update(index * 0.025, 0.7, 0.8) is False
