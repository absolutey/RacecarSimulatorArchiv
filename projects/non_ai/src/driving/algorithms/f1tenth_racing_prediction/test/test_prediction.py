import math

import numpy as np
import pytest

from f1tenth_racing_geometry import ClosedTrackGeometry
from f1tenth_racing_prediction import AnalyticImmPredictor, OpponentMotion, TrackCvPredictor
from f1tenth_racing_prediction.core import directional_variances


def circle_geometry():
    return ClosedTrackGeometry((math.cos(i * math.pi / 8), math.sin(i * math.pi / 8)) for i in range(16))


def motion():
    return OpponentMotion(1.0, 1.0, 0.0, 0.0, 2.0, np.diag([0.01, 0.01, 0.04, 0.04]))


def test_track_cv_wraps_seam_and_grows_covariance():
    predictor = TrackCvPredictor(circle_geometry())
    samples = predictor.predict(motion(), (0.25, 0.5, 1.0))
    assert all(0.0 <= sample.s_m < predictor.geometry.length for sample in samples)
    assert np.trace(samples[-1].covariance_xy) > np.trace(samples[0].covariance_xy)
    assert math.hypot(samples[-1].x_m, samples[-1].y_m) == pytest.approx(1.0, abs=0.08)


def test_track_cv_does_not_turn_along_track_process_noise_into_lateral_motion():
    predictor = TrackCvPredictor(circle_geometry())
    samples = predictor.predict(motion(), (0.1, 1.0))
    first = predictor.geometry.from_frenet(samples[0].s_m, samples[0].d_m)
    last = predictor.geometry.from_frenet(samples[-1].s_m, samples[-1].d_m)
    first_normal = np.array([-math.sin(first.heading), math.cos(first.heading)])
    last_normal = np.array([-math.sin(last.heading), math.cos(last.heading)])
    assert math.isclose(
        float(first_normal @ samples[0].covariance_xy @ first_normal),
        float(last_normal @ samples[-1].covariance_xy @ last_normal),
        rel_tol=1e-9,
    )
    along, across = directional_variances(samples[-1].covariance_xy, last.heading)
    assert along > across


def test_analytic_imm_is_deterministic_and_covariance_positive():
    geometry = circle_geometry()
    first = AnalyticImmPredictor(geometry, [[0.95, 0.05], [0.1, 0.9]]).predict(motion(), (0.5, 1.0))
    second = AnalyticImmPredictor(geometry, [[0.95, 0.05], [0.1, 0.9]]).predict(motion(), (0.5, 1.0))
    assert [(sample.x_m, sample.y_m) for sample in first] == [(sample.x_m, sample.y_m) for sample in second]
    assert all(np.array_equal(a.covariance_xy, b.covariance_xy) for a, b in zip(first, second))
    assert all(np.all(np.linalg.eigvalsh(sample.covariance_xy) > 0.0) for sample in first)


def test_invalid_prediction_inputs_fail_explicitly():
    predictor = TrackCvPredictor(circle_geometry())
    with pytest.raises(ValueError):
        predictor.predict(motion(), (0.5, 0.5))
    with pytest.raises(ValueError):
        AnalyticImmPredictor(circle_geometry(), [[1, 1], [0, 1]])
    invalid = OpponentMotion(1.0, 1.0, 0.0, 0.0, 2.0, -np.eye(4))
    with pytest.raises(ValueError):
        predictor.predict(invalid, (0.5,))


def test_track_cv_supports_extended_horizon_time_grid():
    predictor = TrackCvPredictor(circle_geometry())
    times = tuple(0.1 * step for step in range(1, 21))
    samples = predictor.predict(motion(), times)

    assert len(samples) == 20
    assert samples[-1].dt_s == pytest.approx(2.0)
    assert all(
        0.0 <= sample.s_m < predictor.geometry.length
        for sample in samples
    )
    assert all(
        np.all(np.linalg.eigvalsh(sample.covariance_xy) >= -1e-9)
        for sample in samples
    )
    assert np.trace(samples[-1].covariance_xy) > np.trace(samples[9].covariance_xy)


def test_prediction_source_accepts_fresh_tentative_two_hit_source():
    from f1tenth_racing_prediction.core import prediction_source_is_eligible
    assert prediction_source_is_eligible(
        False, "TENTATIVE", 2, 0.6, 0.0, maximum_age_s=0.5
    )


def test_prediction_source_rejects_weak_tentative_source():
    from f1tenth_racing_prediction.core import prediction_source_is_eligible
    assert not prediction_source_is_eligible(
        False, "TENTATIVE", 1, 0.6, 0.0, maximum_age_s=0.5
    )
    assert not prediction_source_is_eligible(
        False, "TENTATIVE", 2, 0.59, 0.0, maximum_age_s=0.5
    )
    assert not prediction_source_is_eligible(
        False, "TENTATIVE", 2, 0.6, 0.51, maximum_age_s=0.5
    )


def test_prediction_source_preserves_confirmed_valid_path():
    from f1tenth_racing_prediction.core import prediction_source_is_eligible
    assert prediction_source_is_eligible(
        True, "TRACKED", 3, 1.0, 0.0, maximum_age_s=0.5
    )
    assert not prediction_source_is_eligible(
        False, "TRACKED", 2, 0.6, 0.0, maximum_age_s=0.5
    )
