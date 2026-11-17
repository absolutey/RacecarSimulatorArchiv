from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class SpeedEnvelopeConfig:
    convergence_tolerance_m_s: float
    curvature_epsilon_1_m: float
    maximum_iterations: int

    def __post_init__(self):
        values = (self.convergence_tolerance_m_s, self.curvature_epsilon_1_m)
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("speed-envelope tolerances must be finite and positive")
        if self.maximum_iterations < 1:
            raise ValueError("maximum_iterations must be positive")


@dataclass(frozen=True)
class BaselineSpeedProfile:
    station_s: tuple
    station_period: float
    segment_length: tuple
    curvature: tuple
    curvature_speed_ceiling: tuple
    target_speed: tuple
    implied_longitudinal_acceleration: tuple
    limiting_constraint: tuple
    global_speed_ceiling: float
    lateral_acceleration_ceiling: float
    acceleration_limit: float
    braking_limit: float
    iterations: int
    convergence_delta: float
    predicted_lap_time: float


def _finite_positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _arrays(station_s, points, curvature):
    station_s = np.asarray(station_s, dtype=float)
    points = np.asarray(points, dtype=float)
    curvature = np.asarray(curvature, dtype=float)
    count = len(station_s)
    invalid = any((
        count < 3,
        points.shape != (count, 2),
        curvature.shape != (count,),
        not np.all(np.isfinite(station_s)),
        not np.all(np.isfinite(points)),
        not np.all(np.isfinite(curvature)),
        np.any(np.diff(station_s) <= 0.0),
    ))
    if invalid:
        raise ValueError("matching finite closed-path samples and increasing s are required")
    following = np.roll(points, -1, axis=0)
    segment_length = np.linalg.norm(following - points, axis=1)
    if np.any(segment_length <= 0.0):
        raise ValueError("closed path contains a nonpositive segment length")
    return station_s, points, curvature, segment_length


def _labels(speed, local_ceiling, global_max, acceleration, braking, segment_length, tolerance):
    count = len(speed)
    labels = []
    for index in range(count):
        previous = (index - 1) % count
        following = (index + 1) % count
        active = []
        if abs(speed[index] - global_max) <= tolerance:
            active.append("GLOBAL_SPEED")
        if abs(speed[index] - local_ceiling[index]) <= tolerance and local_ceiling[index] < global_max - tolerance:
            active.append("LATERAL_CURVATURE")
        forward_reachable = math.sqrt(speed[previous] ** 2 + 2.0 * acceleration * segment_length[previous])
        if abs(speed[index] - forward_reachable) <= tolerance:
            active.append("FORWARD_ACCELERATION")
        braking_reachable = math.sqrt(speed[following] ** 2 + 2.0 * braking * segment_length[index])
        if abs(speed[index] - braking_reachable) <= tolerance:
            active.append("BACKWARD_BRAKING")
        labels.append(active[0] if len(active) == 1 else "MULTIPLE" if active else "UNCLASSIFIED")
    return tuple(labels)


def generate_speed_profile(
    station_s,
    points,
    curvature,
    global_speed_ceiling,
    acceleration_limit,
    braking_limit,
    lateral_acceleration_ceiling,
    config,
    station_period=None,
):
    station_s, _, curvature, segment_length = _arrays(station_s, points, curvature)
    station_period = float(station_period) if station_period is not None else float(station_s[-1] + segment_length[-1])
    if not math.isfinite(station_period) or station_period <= station_s[-1]:
        raise ValueError("station period must be finite and greater than the last station")
    global_speed_ceiling = _finite_positive(global_speed_ceiling, "global speed ceiling")
    acceleration_limit = _finite_positive(acceleration_limit, "acceleration limit")
    braking_limit = _finite_positive(braking_limit, "braking limit")
    lateral_acceleration_ceiling = _finite_positive(
        lateral_acceleration_ceiling, "lateral acceleration ceiling"
    )

    absolute_curvature = np.maximum.reduce((
        np.roll(np.abs(curvature), 1),
        np.abs(curvature),
        np.roll(np.abs(curvature), -1),
    ))
    local_ceiling = np.full(len(curvature), global_speed_ceiling)
    curved = absolute_curvature > config.curvature_epsilon_1_m
    local_ceiling[curved] = np.minimum(
        global_speed_ceiling,
        np.sqrt(lateral_acceleration_ceiling / absolute_curvature[curved]),
    )
    speed = local_ceiling.copy()
    convergence_delta = math.inf

    for iteration in range(1, config.maximum_iterations + 1):
        previous = speed.copy()
        for index in range(len(speed)):
            following = (index + 1) % len(speed)
            reachable = math.sqrt(speed[index] ** 2 + 2.0 * acceleration_limit * segment_length[index])
            speed[following] = min(speed[following], reachable)
        for index in range(len(speed) - 1, -1, -1):
            following = (index + 1) % len(speed)
            reachable = math.sqrt(speed[following] ** 2 + 2.0 * braking_limit * segment_length[index])
            speed[index] = min(speed[index], reachable)
        convergence_delta = float(np.max(np.abs(speed - previous)))
        if convergence_delta <= config.convergence_tolerance_m_s:
            break
    else:
        raise RuntimeError("periodic speed envelope did not converge")

    following_speed = np.roll(speed, -1)
    implied_acceleration = (following_speed ** 2 - speed ** 2) / (2.0 * segment_length)
    lap_time = float(np.sum(2.0 * segment_length / (speed + following_speed)))
    labels = _labels(
        speed,
        local_ceiling,
        global_speed_ceiling,
        acceleration_limit,
        braking_limit,
        segment_length,
        max(config.convergence_tolerance_m_s * 10.0, 1.0e-9),
    )
    return BaselineSpeedProfile(
        tuple(map(float, station_s)),
        station_period,
        tuple(map(float, segment_length)),
        tuple(map(float, curvature)),
        tuple(map(float, local_ceiling)),
        tuple(map(float, speed)),
        tuple(map(float, implied_acceleration)),
        labels,
        global_speed_ceiling,
        lateral_acceleration_ceiling,
        acceleration_limit,
        braking_limit,
        iteration,
        convergence_delta,
        lap_time,
    )
