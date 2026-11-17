import importlib.util
from pathlib import Path


path = Path(__file__).parents[1] / "scripts/race_coordinator.py"
spec = importlib.util.spec_from_file_location("race_coordinator", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
readiness_reason = module.readiness_reason
opponent_state_ready = module.opponent_state_ready
prediction_ready = module.prediction_ready


def test_h2h_readiness_requires_every_fresh_product_boundary_and_one_controller():
    now, freshness = 1_000_000_000, 750_000_000
    names = ("STATE", "REFERENCE", "OPPONENT_STATE", "PREDICTION", "PLANNER")
    receipts = {name: now for name in names}
    valid = {name: True for name in names}
    assert readiness_reason(now, receipts, valid, 1, freshness) == "READY"
    for name in names:
        missing = dict(receipts)
        missing.pop(name)
        assert readiness_reason(now, missing, valid, 1, freshness) == f"WAITING_FOR_{name}"
    invalid = dict(valid)
    invalid["PREDICTION"] = False
    assert readiness_reason(now, receipts, invalid, 1, freshness) == "WAITING_FOR_PREDICTION"
    stale = dict(receipts)
    stale["PLANNER"] = now - freshness - 1
    assert readiness_reason(now, stale, valid, 1, freshness) == "WAITING_FOR_PLANNER"
    stale_reference = dict(receipts)
    stale_reference["REFERENCE"] = 0
    assert readiness_reason(now, stale_reference, valid, 1, freshness) == "READY"
    assert readiness_reason(now, receipts, valid, 0, freshness) == "WAITING_FOR_CONTROLLER"
    assert readiness_reason(now, receipts, valid, 2, freshness) == "WAITING_FOR_CONTROLLER"


def test_stationary_tentative_track_is_ready_only_after_bounded_sensor_warmup():
    class Message:
        valid = False
        lifecycle = source_lifecycle = "TENTATIVE"
        hit_count = source_hit_count = 2
        confidence = source_confidence = 2.0 / 3.0
        measurement_age_s = source_age_s = 0.0

    message = Message()
    assert opponent_state_ready(message)
    assert prediction_ready(message)
    message.hit_count = message.source_hit_count = 1
    assert not opponent_state_ready(message)
    assert not prediction_ready(message)
    message.hit_count = message.source_hit_count = 2
    message.measurement_age_s = message.source_age_s = 0.6
    assert not opponent_state_ready(message)
    assert not prediction_ready(message)


def test_h2h_readiness_requires_sustained_pipeline_and_state_cadence():
    now, freshness = 1_000_000_000, 750_000_000
    names = ("STATE", "REFERENCE", "OPPONENT_STATE", "PREDICTION", "PLANNER")
    receipts = {name: now for name in names}
    valid = {name: True for name in names}
    histories = {
        "STATE": [now - (9 - index) * 25_000_000 for index in range(10)],
        "OPPONENT_STATE": [now - 200_000_000, now - 100_000_000, now],
        "PREDICTION": [now - 200_000_000, now - 100_000_000, now],
        "PLANNER": [now - 200_000_000, now - 100_000_000, now],
    }
    assert readiness_reason(now, receipts, valid, 1, freshness, histories) == "READY"
    slow = dict(histories)
    slow["STATE"] = [now - (9 - index) * 50_000_000 for index in range(10)]
    assert readiness_reason(now, receipts, valid, 1, freshness, slow) == "WAITING_FOR_STATE_CADENCE"
    sparse = dict(histories)
    sparse["PREDICTION"] = sparse["PREDICTION"][:2]
    assert readiness_reason(now, receipts, valid, 1, freshness, sparse) == "WAITING_FOR_PREDICTION_CADENCE"
