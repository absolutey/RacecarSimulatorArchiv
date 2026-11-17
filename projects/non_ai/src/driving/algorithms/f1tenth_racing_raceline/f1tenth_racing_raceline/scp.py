from dataclasses import dataclass
import math

import numpy as np

from .foundation_adapter import validate_line
from .raceline import BaselineRacingLine, _headings, curvature_metrics
from .speed_adapter import solve_speed_problem, validate_dense


@dataclass(frozen=True)
class ScpConfig:
    maximum_scp_iterations: int
    maximum_admm_iterations: int
    admm_penalty: float
    admm_tolerance: float
    finite_difference_step_m: float
    initial_offset_trust_region_m: float
    minimum_offset_trust_region_m: float
    offset_proximal_weight: float
    speed_squared_proximal_weight: float
    offset_smoothness_weight: float
    minimum_time_improvement_s: float
    use_sparse_solver: bool = False
    use_joint_speed_profile: bool = False

    def __post_init__(self):
        if self.maximum_scp_iterations < 1 or self.maximum_admm_iterations < 1:
            raise ValueError("SCP/ADMM iteration limits must be positive")
        values = tuple(value for key, value in self.__dict__.items()
                       if "maximum_" not in key and not isinstance(value, bool))
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("SCP numeric configuration must be finite and positive")


@dataclass(frozen=True)
class ScpResult:
    line: BaselineRacingLine
    speed_profile: object
    initial_predicted_time_s: float
    final_predicted_time_s: float
    scp_iterations: int
    accepted_steps: int
    convex_iterations: tuple
    termination_reason: str
    validation: dict
    dense_speed_validation: dict
    diagnostics: dict = None


def solve_convex_qp(hessian, gradient, constraints, bounds_low, bounds_high, config, initial):
    """Solve one convex QP with deterministic two-split ADMM."""
    hessian = np.asarray(hessian, dtype=float)
    gradient = np.asarray(gradient, dtype=float)
    constraints = np.asarray(constraints[0], dtype=float), np.asarray(constraints[1], dtype=float)
    low, high = np.asarray(bounds_low, dtype=float), np.asarray(bounds_high, dtype=float)
    x = np.clip(np.asarray(initial, dtype=float), low, high)
    a, b = constraints
    rho = config.admm_penalty
    system = hessian + rho * (a.T @ a) + rho * np.eye(len(x))
    inverse = np.linalg.inv(system)
    z = np.minimum(a @ x, b)
    y = x.copy()
    dual_z = np.zeros(len(b))
    dual_y = np.zeros(len(x))
    for iteration in range(1, config.maximum_admm_iterations + 1):
        x = inverse @ (-gradient + rho * a.T @ (z - dual_z) + rho * (y - dual_y))
        previous_z, previous_y = z, y
        z = np.minimum(a @ x + dual_z, b)
        y = np.clip(x + dual_y, low, high)
        dual_z += a @ x - z
        dual_y += x - y
        primal = max(np.max(np.abs(a @ x - z)), np.max(np.abs(x - y)))
        dual = rho * max(
            np.max(np.abs(a.T @ (z - previous_z))),
            np.max(np.abs(y - previous_y)),
        )
        primal_scale = max(1.0, np.max(np.abs(a @ x)), np.max(np.abs(z)), np.max(np.abs(x)), np.max(np.abs(y)))
        dual_scale = max(1.0, np.max(np.abs(gradient)), np.max(np.abs(rho * a.T @ dual_z)), np.max(np.abs(rho * dual_y)))
        if max(primal / primal_scale, dual / dual_scale) <= config.admm_tolerance:
            return y, iteration
    raise RuntimeError(
        f"convex ADMM did not converge: primal={primal}/{primal_scale}, "
        f"dual={dual}/{dual_scale}"
    )


def _geometry(corridor, offsets):
    points = corridor.points_for_offsets(offsets)
    metrics = curvature_metrics(points)
    following = np.roll(points, -1, axis=0)
    return points, np.asarray(metrics["curvature"]), np.linalg.norm(following - points, axis=1), metrics


def _jacobians(corridor, offsets, epsilon):
    _, curvature, lengths, _ = _geometry(corridor, offsets)
    curvature_jacobian = np.empty((len(offsets), len(offsets)))
    length_jacobian = np.empty_like(curvature_jacobian)
    for index in range(len(offsets)):
        perturbed = offsets.copy()
        direction = 1.0 if offsets[index] + epsilon <= corridor.upper_offsets[index] else -1.0
        perturbed[index] += direction * epsilon
        _, changed_curvature, changed_lengths, _ = _geometry(corridor, perturbed)
        curvature_jacobian[:, index] = (changed_curvature - curvature) / (direction * epsilon)
        length_jacobian[:, index] = (changed_lengths - lengths) / (direction * epsilon)
    return curvature, lengths, curvature_jacobian, length_jacobian


def _time_gradient(lengths, length_jacobian, speed_squared):
    speeds = np.sqrt(np.maximum(speed_squared, 1e-6))
    following = np.roll(speeds, -1)
    length_weights = 2.0 / (speeds + following)
    gradient_d = length_weights @ length_jacobian
    gradient_w = np.zeros(len(speeds))
    for index in range(len(speeds)):
        following_index = (index + 1) % len(speeds)
        denominator = speeds[index] + speeds[following_index]
        common = -lengths[index] / (denominator * denominator)
        gradient_w[index] += common / speeds[index]
        gradient_w[following_index] += common / speeds[following_index]
    return np.concatenate((gradient_d, gradient_w))


def _constraints(offsets, speed_squared, curvature, lengths, curvature_jacobian,
                 length_jacobian, lateral_acceleration, acceleration, braking):
    count = len(offsets)
    x0 = np.concatenate((offsets, speed_squared))
    rows, limits = [], []

    def linearized(g0, gradient):
        rows.append(gradient)
        limits.append(-g0 + gradient @ x0)

    for index in range(count):
        for sign in (-1.0, 1.0):
            gradient = np.zeros(2 * count)
            gradient[:count] = sign * speed_squared[index] * curvature_jacobian[index]
            gradient[count + index] = sign * curvature[index]
            linearized(sign * curvature[index] * speed_squared[index] - lateral_acceleration, gradient)
        following = (index + 1) % count
        forward = np.zeros(2 * count)
        forward[:count] = -2.0 * acceleration * length_jacobian[index]
        forward[count + index], forward[count + following] = -1.0, 1.0
        linearized(speed_squared[following] - speed_squared[index] - 2.0 * acceleration * lengths[index], forward)
        reverse = np.zeros(2 * count)
        reverse[:count] = -2.0 * braking * length_jacobian[index]
        reverse[count + index], reverse[count + following] = 1.0, -1.0
        linearized(speed_squared[index] - speed_squared[following] - 2.0 * braking * lengths[index], reverse)
    return np.asarray(rows), np.asarray(limits)


def _line(corridor, offsets, initial_objective, iterations, reason):
    points, _, _, metrics = _geometry(corridor, offsets)
    return BaselineRacingLine(
        corridor.station_s,
        tuple(map(float, points[:, 0])),
        tuple(map(float, points[:, 1])),
        tuple(map(float, _headings(points))),
        tuple(map(float, metrics["curvature"])),
        tuple(map(float, offsets)),
        initial_objective,
        metrics["objective"],
        metrics["objective"],
        metrics["length"],
        iterations,
        iterations,
        reason,
    )


def solve_scp(corridor, initial_line, speed_problem, line_problem, config):
    # Optional SciPy import: frozen legacy SCP remains usable without SciPy.
    if config.use_sparse_solver or config.use_joint_speed_profile:
        from .sparse_joint import (solve_sparse_qp, geometry_sparse_jacobian,
            sparse_linear_constraints, sparse_hessian, candidate_speed_profile,
            jerk_feasible_initial_profile)
    offsets = np.asarray(initial_line.lateral_offset, dtype=float)
    initial_profile = solve_speed_problem(speed_problem)
    if config.use_joint_speed_profile:
        profile, phase_i = jerk_feasible_initial_profile(
            initial_profile, jerk_limit=speed_problem["limits"]["jerk_m_s3"],
            dense_samples=speed_problem["optimizer_data"]["dense_samples_per_segment"])
        phase_i['raw_envelope_predicted_time_s'] = initial_profile.predicted_lap_time
        phase_i['feasible_seed_predicted_time_s'] = profile.predicted_lap_time
    else:
        profile, phase_i = initial_profile, None
    speed_squared = np.square(profile.target_speed)
    trust = config.initial_offset_trust_region_m
    accepted = 0
    convex_iterations = []
    rejects = {}
    trials = 0
    def rejected(reason):
        rejects[reason] = rejects.get(reason, 0) + 1
    initial_objective = curvature_metrics(initial_line.points)["objective"]
    reason = "MAXIMUM_SCP_ITERATIONS"
    second = None if config.use_sparse_solver else (np.roll(np.eye(len(offsets)), -1, axis=0) - 2.0 * np.eye(len(offsets)) + np.roll(np.eye(len(offsets)), 1, axis=0))

    for scp_iteration in range(1, config.maximum_scp_iterations + 1):
        if config.use_sparse_solver:
            curvature, lengths, curvature_jacobian, length_jacobian = geometry_sparse_jacobian(
                corridor, offsets, config.finite_difference_step_m, _geometry)
        else:
            curvature, lengths, curvature_jacobian, length_jacobian = _jacobians(
                corridor, offsets, config.finite_difference_step_m)
        count = len(offsets)
        proximal = np.concatenate((
            np.full(count, config.offset_proximal_weight),
            np.full(count, config.speed_squared_proximal_weight),
        ))
        if config.use_sparse_solver:
            hessian = sparse_hessian(count, config.offset_proximal_weight,
                                     config.speed_squared_proximal_weight,
                                     config.offset_smoothness_weight)
        else:
            hessian = np.diag(proximal)
            hessian[:count, :count] += 2.0 * config.offset_smoothness_weight * (second.T @ second)
        gradient = _time_gradient(lengths, length_jacobian, speed_squared) - proximal * np.concatenate((offsets, speed_squared))
        constraint_builder = sparse_linear_constraints if config.use_sparse_solver else _constraints
        args = (offsets, speed_squared, curvature, lengths, curvature_jacobian,
                length_jacobian, profile.lateral_acceleration_ceiling,
                profile.acceleration_limit, profile.braking_limit)
        if config.use_joint_speed_profile:
            a, b = constraint_builder(*args, jerk_limit=speed_problem["limits"]["jerk_m_s3"])
        else:
            a, b = constraint_builder(*args)
        low = np.concatenate((
            np.maximum(np.asarray(corridor.lower_offsets), offsets - trust),
            np.full(count, 1e-4),
        ))
        high = np.concatenate((
            np.minimum(np.asarray(corridor.upper_offsets), offsets + trust),
            np.full(count, profile.global_speed_ceiling ** 2),
        ))
        optimizer = solve_sparse_qp if config.use_sparse_solver else solve_convex_qp
        candidate, qp_iterations = optimizer(
            hessian, gradient, (a, b), low, high, config, np.concatenate((offsets, speed_squared))
        )
        convex_iterations.append(qp_iterations)
        accepted_step = False
        for alpha in (2.0 ** (-i) for i in range(16 if config.use_joint_speed_profile else 4)):
            trials += 1
            trial_offsets = offsets + alpha * (candidate[:count] - offsets)
            trial_line = _line(corridor, trial_offsets, initial_objective, scp_iteration, "SCP_TRIAL")
            try:
                validate_line(trial_line, line_problem)
                trial_profile = solve_speed_problem(speed_problem, {
                    "station_s": trial_line.station_s,
                    "points": trial_line.points,
                    "curvature": trial_line.curvature,
                    "station_period": corridor.geometry.length,
                })
            except (RuntimeError, ValueError) as error:
                rejected('geometry_or_envelope:' + str(error)[:140])
                continue
            if config.use_joint_speed_profile:
                try:
                    # The regenerated envelope is a constraint ceiling, NOT the selected speed.
                    joint_v2 = speed_squared + alpha * (candidate[count:] - speed_squared)
                    joint_profile = candidate_speed_profile(
                        trial_profile, joint_v2, jerk_limit=speed_problem["limits"]["jerk_m_s3"])
                    validate_dense(joint_profile, joint_profile.station_period,
                                   speed_problem["optimizer_data"]["dense_samples_per_segment"])
                except (RuntimeError, ValueError) as error:
                    rejected('joint_candidate:' + str(error)[:140])
                    continue
                trial_profile = joint_profile
            if trial_profile.predicted_lap_time < profile.predicted_lap_time - config.minimum_time_improvement_s:
                offsets = trial_offsets
                profile = trial_profile
                speed_squared = np.square(profile.target_speed)
                trust = min(config.initial_offset_trust_region_m, trust * 1.25)
                accepted += 1
                accepted_step = True
                break
            rejected('no_predicted_time_improvement')
        if not accepted_step:
            trust *= 0.5
            if trust < config.minimum_offset_trust_region_m:
                reason = "TRUST_REGION_EXHAUSTED"
                break

    final_line = _line(corridor, offsets, initial_objective, scp_iteration, reason)
    validation = validate_line(final_line, line_problem)
    dense = validate_dense(profile, profile.station_period, speed_problem["optimizer_data"]["dense_samples_per_segment"])
    return ScpResult(
        final_line, profile,
        (phase_i['feasible_seed_predicted_time_s'] if phase_i is not None
         else initial_profile.predicted_lap_time),
        profile.predicted_lap_time, scp_iteration, accepted,
        tuple(convex_iterations), reason, validation, dense,
        {'phase_i': phase_i, 'candidate_trials': trials, 'rejection_counts': rejects,
         'joint_qp_accepted_steps': accepted},
    )
