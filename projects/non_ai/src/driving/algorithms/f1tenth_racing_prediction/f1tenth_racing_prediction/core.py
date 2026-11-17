from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class OpponentMotion:
    time_s: float
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
    covariance: np.ndarray


@dataclass(frozen=True)
class PredictionSample:
    dt_s: float
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
    heading_rad: float
    s_m: float
    d_m: float
    covariance_xy: np.ndarray


def prediction_source_is_eligible(
    valid,
    lifecycle,
    hit_count,
    confidence,
    measurement_age_s,
    maximum_age_s=0.5,
    minimum_tentative_hits=2,
    minimum_tentative_confidence=0.6,
):
    """Return whether R-D may predict from a fresh R-C source.

    Confirmed/valid R-C tracks remain eligible exactly as before.
    A TENTATIVE source is eligible for early prediction only when it has
    at least two hits, >=0.6 confidence, and <=0.5 s measurement age.
    Track/frame identity and numerical/covariance safety remain enforced
    by the node and predictor.
    """
    values = (
        hit_count,
        confidence,
        measurement_age_s,
        maximum_age_s,
        minimum_tentative_hits,
        minimum_tentative_confidence,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return False
    if measurement_age_s < 0.0 or measurement_age_s > maximum_age_s:
        return False
    if valid:
        return True
    return (
        lifecycle == "TENTATIVE"
        and hit_count >= minimum_tentative_hits
        and confidence >= minimum_tentative_confidence
    )

def directional_variances(covariance_xy, heading_rad):
    covariance = np.asarray(covariance_xy, dtype=float)
    tangent = np.array([math.cos(heading_rad), math.sin(heading_rad)])
    normal = np.array([-tangent[1], tangent[0]])
    return float(tangent @ covariance @ tangent), float(normal @ covariance @ normal)


def _propagated_covariance(covariance, dt_s, acceleration_variance):
    covariance = np.asarray(covariance, dtype=float)
    if covariance.shape != (4, 4) or not np.all(np.isfinite(covariance)):
        raise ValueError("finite 4x4 covariance required")
    if not np.allclose(covariance, covariance.T) or np.min(np.linalg.eigvalsh(covariance)) < -1e-9:
        raise ValueError("covariance must be symmetric positive semidefinite")
    f = np.array([[1, 0, dt_s, 0], [0, 1, 0, dt_s], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
    g = np.array([[0.5 * dt_s ** 2, 0], [0, 0.5 * dt_s ** 2], [dt_s, 0], [0, dt_s]])
    return f @ covariance @ f.T + acceleration_variance * g @ g.T


def _track_covariance(covariance, dt_s, acceleration_variance, source_heading, target_heading):
    covariance = np.asarray(covariance, dtype=float)
    _propagated_covariance(covariance, dt_s, acceleration_variance)  # shared validation
    tangent = np.array([math.cos(source_heading), math.sin(source_heading)])
    normal = np.array([-tangent[1], tangent[0]])
    along = np.r_[tangent, dt_s * tangent]
    across = np.r_[normal, np.zeros(2)]
    along_variance = float(along @ covariance @ along + 0.25 * acceleration_variance * dt_s ** 4)
    across_variance = float(across @ covariance @ across)
    cross = float(along @ covariance @ across)
    target_tangent = np.array([math.cos(target_heading), math.sin(target_heading)])
    target_normal = np.array([-target_tangent[1], target_tangent[0]])
    basis = np.column_stack((target_tangent, target_normal))
    return basis @ np.array([[along_variance, cross], [cross, across_variance]]) @ basis.T


class TrackCvPredictor:
    def __init__(self, geometry, acceleration_variance=2.0):
        self.geometry = geometry
        self.acceleration_variance = float(acceleration_variance)
        if self.acceleration_variance < 0.0:
            raise ValueError("acceleration variance must be nonnegative")

    def predict(self, motion, times):
        values = tuple(float(value) for value in times)
        if not values or values[0] <= 0.0 or any(a >= b for a, b in zip(values, values[1:])):
            raise ValueError("strictly increasing positive prediction times required")
        if not all(math.isfinite(value) for value in vars(motion).values() if np.isscalar(value)):
            raise ValueError("finite opponent motion required")
        projection = self.geometry.project(motion.x_m, motion.y_m)
        tangent = np.array([math.cos(projection.heading), math.sin(projection.heading)])
        along_speed = float(tangent @ np.array([motion.vx_mps, motion.vy_mps]))
        samples = []
        for dt_s in values:
            s_m = self.geometry.wrap_s(projection.s + along_speed * dt_s)
            point = self.geometry.from_frenet(s_m, projection.d)
            vx, vy = along_speed * math.cos(point.heading), along_speed * math.sin(point.heading)
            samples.append(PredictionSample(
                dt_s, point.x, point.y, vx, vy, point.heading, s_m, projection.d,
                _track_covariance(
                    motion.covariance, dt_s, self.acceleration_variance,
                    projection.heading, point.heading,
                ),
            ))
        return tuple(samples)


class AnalyticImmPredictor:
    """Two-mode track-following/map-CV analytic mixture with Markov weights."""

    def __init__(self, geometry, transition, acceleration_variance=2.0):
        self.geometry = geometry
        self.track = TrackCvPredictor(geometry, acceleration_variance)
        self.transition = np.asarray(transition, dtype=float)
        if self.transition.shape != (2, 2) or np.any(self.transition < 0.0) or not np.allclose(self.transition.sum(axis=1), 1.0):
            raise ValueError("IMM transition must be a 2x2 row-stochastic matrix")
        self.probabilities = np.array([0.7, 0.3])
        self.previous = None

    def _map_cv(self, motion, times):
        samples = []
        for dt_s in times:
            x_m, y_m = motion.x_m + motion.vx_mps * dt_s, motion.y_m + motion.vy_mps * dt_s
            projection = self.geometry.project(x_m, y_m)
            samples.append(PredictionSample(
                dt_s, x_m, y_m, motion.vx_mps, motion.vy_mps,
                math.atan2(motion.vy_mps, motion.vx_mps), projection.s, projection.d,
                _propagated_covariance(motion.covariance, dt_s, self.track.acceleration_variance)[:2, :2],
            ))
        return tuple(samples)

    def predict(self, motion, times):
        times = tuple(float(value) for value in times)
        track_samples = self.track.predict(motion, times)
        map_samples = self._map_cv(motion, times)
        prior = self.transition.T @ self.probabilities
        if self.previous is not None:
            dt_s = motion.time_s - self.previous.time_s
            if dt_s > 0.0:
                expected = (self.track.predict(self.previous, (dt_s,))[0], self._map_cv(self.previous, (dt_s,))[0])
                errors = np.array([
                    (motion.x_m - sample.x_m) ** 2 + (motion.y_m - sample.y_m) ** 2
                    for sample in expected
                ])
                likelihood = np.exp(-0.5 * errors / 0.25)
                posterior = prior * likelihood
                self.probabilities = posterior / posterior.sum() if posterior.sum() > 0.0 else prior
        else:
            self.probabilities = prior
        self.previous = motion
        mixed = []
        for track, straight in zip(track_samples, map_samples):
            means = np.array([[track.x_m, track.y_m], [straight.x_m, straight.y_m]])
            mean = self.probabilities @ means
            covariance = np.zeros((2, 2))
            for weight, sample, sample_mean in zip(self.probabilities, (track, straight), means):
                delta = sample_mean - mean
                covariance += weight * (sample.covariance_xy + np.outer(delta, delta))
            projection = self.geometry.project(*mean)
            velocity = self.probabilities @ np.array([[track.vx_mps, track.vy_mps], [straight.vx_mps, straight.vy_mps]])
            mixed.append(PredictionSample(
                track.dt_s, mean[0], mean[1], velocity[0], velocity[1],
                math.atan2(velocity[1], velocity[0]), projection.s, projection.d, covariance,
            ))
        return tuple(mixed)
