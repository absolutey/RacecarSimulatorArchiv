"""ABORT recovery may not recycle a pre-ABORT Red prediction."""
from f1tenth_racing_overtaking.post_abort_freshness import accept_post_abort_red_observation as admit


def test_no_release_normal_prediction_contract_unchanged():
    assert admit(10, None, 11)

def test_new_red_after_release_accepted():
    assert admit(1_000_000_001, 1_000_000_000, 1_010_000_000)

def test_exact_same_timestamp_does_not_count_as_new_observation():
    assert not admit(1_000_000_000, 1_000_000_000, 1_010_000_000)

def test_preabort_and_future_replayed_observations_rejected():
    assert not admit(999_999_999, 1_000_000_000, 1_010_000_000)
    assert not admit(1_100_000_000, 1_000_000_000, 1_010_000_000)

def test_stale_after_release_and_corrupt_type_rejected():
    assert not admit(1_000_000_001, 1_000_000_000, 1_500_000_000)
    assert not admit('1001', 1000, 1010)
