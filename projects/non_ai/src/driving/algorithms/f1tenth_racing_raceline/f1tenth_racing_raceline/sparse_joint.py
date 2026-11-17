"""Sparse trust-region QP and joint speed feasibility, offline only.

This module does NOT produce runtime approval. It solves the same two-split
ADMM subproblem as the dense prototype using sparse factorization. Every
accepted solution must be checked against non-linear corridor and speed limits.
"""
from dataclasses import replace
import math

import numpy as np
from scipy import sparse
from scipy.sparse.linalg import splu


def _near_feasible_sparse_projection(a, b, low, high, y, *, max_change=1.0e-4):
    """Bounded sparse half-space projection; ALWAYS verify original QP.

    Hildreth-style dual-coordinate correction for a near-feasible ADMM iterate.
    Unlike the previous augmented LP, it does NOT re-optimize 1729 variables
    with HiGHS. Original Ax <= b, original low/high, and |x-y| <= 1e-4 are all
    explicit inequalities. A failed numerical correction NEVER grants success.
    This only repairs minute feasibility error in OFFLINE SCP, not the QP KKT
    check, nonlinear SCP validation, or Guard safety certificate.
    """
    a = sparse.csr_matrix(a)
    b = np.asarray(b, dtype=float)
    y = np.asarray(y, dtype=float)
    low = np.asarray(low, dtype=float)
    high = np.asarray(high, dtype=float)
    n = len(y)
    if (a.shape != (len(b),n) or low.shape != y.shape or high.shape != y.shape
            or not np.isfinite(max_change) or max_change <= 0.0
            or np.any(low > high) or not all(np.all(np.isfinite(v))
            for v in (a.data,b,y,low,high))):
        raise ValueError('invalid sparse correction input')

    # The box on x below is the intersection of ORIGINAL variable bounds and
    # an unmodified 1e-4 maximum absolute correction around ADMM candidate y.
    lo = np.maximum(low, y - max_change)
    hi = np.minimum(high, y + max_change)
    if np.any(lo > hi):
        raise RuntimeError('no correction within original bounds and maximum change')
    eye = sparse.eye(n, format='csr')
    halfspaces = sparse.vstack((a, eye, -eye), format='csr')
    rhs = np.r_[b, hi, -lo]
    # Hildreth's cyclic dual-coordinate method for the nearest feasible point:
    # minimize 0.5*||x-y||_2^2 subject to original half-space inequalities.
    # Every accepted returned point is checked again against ORIGINAL A,b.
    rownorm = np.asarray(halfspaces.multiply(halfspaces).sum(axis=1)).ravel()
    if np.any(rownorm <= 0.0):
        # A zero row is a valid, redundant constraint iff 0 <= its rhs.
        zeros = rownorm <= 0.0
        if np.any(rhs[zeros] < 0.0):
            raise RuntimeError('infeasible zero-normal inequality')
        rownorm[zeros] = 1.0
    dual = np.zeros(halfspaces.shape[0], dtype=float)
    x = y.copy()
    data, indices, ptr = halfspaces.data, halfspaces.indices, halfspaces.indptr
    absolute_target = 1.0e-6   # Match original QP feasibility gate; no constraint waiver
    max_passes = 450
    for sweep in range(max_passes):
        residual = halfspaces @ x - rhs
        worst = float(np.max(residual))
        if not np.isfinite(worst) or not np.all(np.isfinite(x)):
            raise RuntimeError('numerically unstable sparse projection')
        if worst <= absolute_target:
            break
        # Visit violated rows and rows with nonzero multipliers. Previously
        # active constraints may need correction release after another row moves.
        active = np.flatnonzero((residual > 0.0) | (dual > 1.0e-18))
        if len(active) == 0:
            raise RuntimeError('sparse projection stagnated without active rows')
        for i in active:
            start, end = int(ptr[i]), int(ptr[i+1])
            js = indices[start:end]
            values = data[start:end]
            violation = float(np.dot(values,x[js]) - rhs[i])
            step = max(-dual[i], violation / rownorm[i])
            if step != 0.0:
                dual[i] += step
                x[js] -= step * values
    else:
        worst = float(np.max(halfspaces @ x - rhs))
        if not np.isfinite(worst) or worst > 1.0e-4:
            raise RuntimeError(
                f'sparse projection not near-feasible after {max_passes} passes: '
                f'violation={worst:.9g}')
        # Hildreth can stall on nearly parallel, differently scaled rows.
        # Solve a bounded FEASIBILITY problem in a 1e-6 box around its last
        # iterate, intersected with the UNCHANGED original variable bounds
        # and the pre-existing 1e-4 maximum ADMM correction. No relaxation.
        # The original full Ax <= b is checked again below, as are the outer
        # objective/stationarity and nonlinear SCP acceptance gates.
        from scipy.optimize import linprog
        local_radius = 1.0e-6
        local_lo = np.maximum(lo, x - local_radius)
        local_hi = np.minimum(hi, x + local_radius)
        if np.any(local_lo > local_hi):
            raise RuntimeError('sparse precision correction box empty')
        lp = linprog(np.zeros(n, dtype=float), A_ub=a, b_ub=b,
                     bounds=list(zip(local_lo, local_hi)), method='highs',
                     options={'time_limit': 3.0})
        if not lp.success or lp.x is None or not np.all(np.isfinite(lp.x)):
            raise RuntimeError('sparse local feasibility refinement BLOCKED: '
                               + str(lp.message))
        polished = np.asarray(lp.x, dtype=float)
        if (np.max(np.abs(polished - x)) > local_radius + 1.0e-9 or
                np.max(np.abs(polished - y)) > max_change + 1.0e-9 or
                max(0.0, float(np.max(a @ polished - b)),
                    float(np.max(low - polished)),
                    float(np.max(polished - high))) > absolute_target):
            raise RuntimeError('sparse local refinement failed ORIGINAL constraints')
        x = polished

    distance = float(np.max(np.abs(x-y)))
    original_violation = max(0., float(np.max(a @ x - b)),
                             float(np.max(low-x)), float(np.max(x-high)))
    if not (np.all(np.isfinite(x)) and distance <= max_change + 1.0e-9
            and original_violation <= 1.0e-6):
        raise RuntimeError('sparse projection rejected by original QP constraints: '
            f'distance={distance:.9g} abs_violation={original_violation:.9g}')
    return x, distance, original_violation


def solve_sparse_qp(hessian, gradient, constraints, bounds_low, bounds_high, config, initial):
    """Bounded adaptive-rho sparse ADMM; fixed original iteration/tolerance.

    Keeps the original objective and constraints. Does not grant offline or
    runtime qualification by itself: the SCP outer loop still has to validate
    its full nonlinear candidate. Numeric feasibility is independently checked
    before returning a QP step.
    """
    h = sparse.csc_matrix(hessian)
    a = sparse.csc_matrix(constraints[0])
    b = np.asarray(constraints[1], dtype=float)
    gradient = np.asarray(gradient, dtype=float)
    low = np.asarray(bounds_low, dtype=float)
    high = np.asarray(bounds_high, dtype=float)
    x = np.clip(np.asarray(initial, dtype=float), low, high)
    if (h.shape != (len(x), len(x)) or a.shape != (len(b), len(x)) or
            gradient.shape != x.shape or low.shape != x.shape or high.shape != x.shape or
            not all(np.all(np.isfinite(v)) for v in (h.data, a.data, b, gradient, low, high, x)) or
            np.any(low > high)):
        raise ValueError('invalid sparse QP dimensions/bounds/numerics')
    rho = float(config.admm_penalty)
    if not math.isfinite(rho) or rho <= 0:
        raise ValueError('invalid sparse ADMM penalty')
    ata = (a.T @ a).tocsc()
    ident = sparse.eye(len(x), format='csc')
    def factor(penalty):
        return splu((h + penalty * (ata + ident)).tocsc())
    lu = factor(rho)
    z = np.minimum(a @ x, b)
    y = x.copy()
    dz = np.zeros(len(b))
    dy = np.zeros(len(x))
    tol = float(config.admm_tolerance)
    if not math.isfinite(tol) or tol <= 0.0:
        raise ValueError('invalid sparse ADMM tolerance')
    for iteration in range(1, config.maximum_admm_iterations + 1):
        x = lu.solve(-gradient + rho * (a.T @ (z - dz)) + rho * (y - dy))
        prev_z = z.copy()
        prev_y = y.copy()
        ax = a @ x
        z = np.minimum(ax + dz, b)
        y = np.clip(x + dy, low, high)
        dz += ax - z
        dy += x - y
        residual = max(float(np.max(np.abs(ax - z))), float(np.max(np.abs(x - y))))
        dual = rho * max(float(np.max(np.abs(a.T @ (z - prev_z)))),
                         float(np.max(np.abs(y - prev_y))))
        pscale = max(1.0, float(np.max(np.abs(ax))), float(np.max(np.abs(z))),
                     float(np.max(np.abs(x))), float(np.max(np.abs(y))))
        dscale = max(1.0, float(np.max(np.abs(gradient))),
                     float(np.max(np.abs(rho * (a.T @ dz)))), float(np.max(np.abs(rho * dy))))
        pn, dn = residual / pscale, dual / dscale
        if max(pn, dn) <= tol:
            violation = max(0.0, float(np.max(a @ y - b)),
                            float(np.max(low - y)), float(np.max(y - high)))
            # A normalized ADMM stopping rule alone cannot approve a
            # physically infeasible speed/corridor candidate.
            if violation <= 1.0e-6 and np.all(np.isfinite(y)):
                return y, iteration
        # Residual balancing is numerical preconditioning, not a relaxation
        # of any speed limit, SCP trust bound or original stopping tolerance.
        if iteration % 25 == 0 and iteration < config.maximum_admm_iterations:
            newrho = rho
            if pn > 8.0 * dn:
                newrho = min(rho * 2.0, config.admm_penalty * 1024.0)
            elif dn > 8.0 * pn:
                newrho = max(rho * 0.5, config.admm_penalty / 1024.0)
            if newrho != rho:
                # Preserve unscaled Lagrange multipliers: lambda=rho*dual.
                dz *= rho / newrho
                dy *= rho / newrho
                rho = newrho
                lu = factor(rho)
    violation = max(0.0, float(np.max(a @ y - b)),
                    float(np.max(low - y)), float(np.max(y - high)))
    # Recover ONLY the last few micrometres of feasibility from an otherwise
    # already-converged offline ADMM iterate. Never loosen tol or constraints.
    if max(pn, dn) <= tol and 1.0e-6 < violation <= 1.0e-4 and np.all(np.isfinite(y)):
        try:
            repaired, change, repaired_violation = _near_feasible_sparse_projection(
                a, b, low, high, y, max_change=1.0e-4)
            # Independently certify approximate KKT stationarity using the
            # unscaled ADMM duals and actual Hessian/gradient at corrected x.
            stationarity = h @ repaired + gradient + a.T @ (rho * dz) + rho * dy
            station_scale = max(1.0, float(np.max(np.abs(gradient))),
                                float(np.max(np.abs(h @ repaired))),
                                float(np.max(np.abs(a.T @ (rho * dz)))),
                                float(np.max(np.abs(rho * dy))))
            station_norm = float(np.max(np.abs(stationarity))) / station_scale
            old_obj = 0.5 * float(initial @ (h @ initial)) + float(gradient @ initial)
            new_obj = 0.5 * float(repaired @ (h @ repaired)) + float(gradient @ repaired)
            if (station_norm <= tol and np.all(np.isfinite(repaired)) and
                    new_obj <= old_obj + max(1e-8, abs(old_obj) * 1e-8) and
                    repaired_violation <= 1.0e-6):
                return repaired, iteration
            raise RuntimeError(
                f'polished iterate not qualified: stationarity={station_norm}/{tol}, '
                f'objective={new_obj}/{old_obj}, correction_inf={change}, '
                f'absolute_constraint_violation={repaired_violation}')
        except RuntimeError as polish_exc:
            raise RuntimeError(f'sparse QP precision repair BLOCKED: {polish_exc}') from polish_exc
    raise RuntimeError(f'sparse ADMM not converged: primal={residual}/{pscale}, '
                       f'dual={dual}/{dscale}, absolute_constraint_violation={violation}, '
                       f'last_rho={rho}')


def geometry_sparse_jacobian(corridor, offsets, epsilon, geometry_fn):
    """Finite differencing, retaining only exact local curvature/segment stencils.

    Curvature at i uses vertices i-1,i,i+1 and segment i uses i,i+1.
    """
    offsets = np.asarray(offsets, dtype=float)
    n = len(offsets)
    _, curvature, lengths, _ = geometry_fn(corridor, offsets)
    cr, cc, cv, lr, lc, lv = [], [], [], [], [], []
    for j in range(n):
        delta = np.zeros(n)
        direction = 1.0 if offsets[j] + epsilon <= corridor.upper_offsets[j] else -1.0
        delta[j] = direction * epsilon
        _, altered_c, altered_l, _ = geometry_fn(corridor, offsets + delta)
        for i in ((j - 1) % n, j, (j + 1) % n):
            value = (altered_c[i] - curvature[i]) / (direction * epsilon)
            if value != 0.0:
                cr.append(i); cc.append(j); cv.append(value)
        for i in ((j - 1) % n, j):
            value = (altered_l[i] - lengths[i]) / (direction * epsilon)
            if value != 0.0:
                lr.append(i); lc.append(j); lv.append(value)
    cjac = sparse.csr_matrix((cv, (cr, cc)), shape=(n, n))
    ljac = sparse.csr_matrix((lv, (lr, lc)), shape=(n, n))
    return curvature, lengths, cjac, ljac



def sparse_linear_constraints(offsets, speed_squared, curvature, lengths, cjac, ljac,
                              lateral_acceleration, acceleration, braking, *, jerk_limit=None):
    """Linearize the SAME three-neighbor conservative lateral envelope as R-A1.

    This uses all adjacent curvature constraints rather than choosing just one
    nonsmooth max-active branch. Jerk rows freeze the positive physical time
    intervals at the current feasible SCP iterate; the exact nonlinear jerk
    formula is still checked before accepting a step.
    """
    n = len(offsets)
    current = np.r_[offsets, speed_squared]
    rows, cols, vals, limits = [], [], [], []

    def insert(entries, constant):
        row = len(limits)
        dot = 0.0
        for col, value in entries:
            if value != 0.0:
                rows.append(row); cols.append(col); vals.append(value)
                dot += value * current[col]
        limits.append(-constant + dot)

    for i in range(n):
        lx = ljac.getrow(i)
        for neighbor in ((i - 1) % n, i, (i + 1) % n):
            cx = cjac.getrow(neighbor)
            for sign in (-1.0, 1.0):
                entries = [(int(c), sign * speed_squared[i] * v)
                           for c, v in zip(cx.indices, cx.data)]
                entries.append((n + i, sign * curvature[neighbor]))
                insert(entries, sign * curvature[neighbor] * speed_squared[i] - lateral_acceleration)
        k = (i + 1) % n
        insert([(int(c), -2.0 * acceleration * v) for c, v in zip(lx.indices, lx.data)] +
               [(n + i, -1.0), (n + k, 1.0)],
               speed_squared[k] - speed_squared[i] - 2.0 * acceleration * lengths[i])
        insert([(int(c), -2.0 * braking * v) for c, v in zip(lx.indices, lx.data)] +
               [(n + i, 1.0), (n + k, -1.0)],
               speed_squared[i] - speed_squared[k] - 2.0 * braking * lengths[i])

    if jerk_limit is not None:
        if not np.isfinite(jerk_limit) or jerk_limit <= 0.0:
            raise ValueError('jerk limit must be finite and positive')
        speeds = np.sqrt(speed_squared)
        if np.any(speeds <= 0) or np.any(np.asarray(lengths) <= 0):
            raise ValueError('nonpositive speed or segment length in jerk constraints')
        dt = 2.0 * lengths / (speeds + np.roll(speeds, -1))
        for i in range(n):
            j, k = (i + 1) % n, (i + 2) % n
            a = 1.0 / (2.0 * lengths[i])
            b = 1.0 / (2.0 * lengths[j])
            delta = (speed_squared[k] - speed_squared[j]) * b - (speed_squared[j] - speed_squared[i]) * a
            frozen_dt = 0.5 * (dt[i] + dt[j])
            for sign in (-1.0, 1.0):
                insert([(n + i, sign * a), (n + j, -sign * (a+b)), (n + k, sign * b)],
                       sign * delta - jerk_limit * frozen_dt)
    return sparse.csr_matrix((vals, (rows, cols)), shape=(len(limits), 2 * n)), np.array(limits)


def jerk_feasible_initial_profile(reference, *, jerk_limit, dense_samples=20):
    """Phase I: produce a safe squared-speed seed without modifying R-A1.

    Convex interpolation in squared speed between a constant-speed feasible
    anchor and the original periodic envelope maintains all envelope and
    longitudinal constraints. Binary-search jerk feasibility, then retain
    some jerk slack for phase II. Never treat this seed as a joint QP step.
    """
    from .speed_adapter import validate_dense
    ceiling = np.asarray(reference.target_speed, dtype=float)
    if ceiling.ndim != 1 or not np.all(np.isfinite(ceiling)) or min(ceiling) <= 0:
        raise ValueError('invalid initial envelope')
    seed_speed = float(min(ceiling) * 0.90)
    low_w = np.full(len(ceiling), seed_speed ** 2)
    target_w = np.square(ceiling)
    def trial(alpha):
        w = low_w + alpha * (target_w - low_w)
        candidate = candidate_speed_profile(reference, w, jerk_limit=jerk_limit)
        validate_dense(candidate, candidate.station_period, dense_samples)
        return candidate
    best = trial(0.0)  # Guarded feasible anchor, not an implicit speed waiver.
    try:
        return trial(1.0), {'phase_i_alpha': 1.0, 'phase_i_seed_speed_mps': seed_speed}
    except (ValueError, RuntimeError):
        pass
    lo, hi = 0.0, 1.0
    for _ in range(38):
        mid = (lo + hi) / 2.0
        try:
            trial(mid)
            lo = mid
        except (ValueError, RuntimeError):
            hi = mid
    # Stay inside, rather than precisely on, a nonlinear jerk active bound.
    feasible_alpha = max(0.0, lo * 0.90)
    best = trial(feasible_alpha)
    return best, {'phase_i_alpha': feasible_alpha, 'phase_i_seed_speed_mps': seed_speed}


def sparse_hessian(n, offset_weight, speed_weight, smoothness):
    identity = sparse.eye(n, format='csr')
    d2 = (sparse.diags([np.ones(n - 1), -2 * np.ones(n), np.ones(n - 1)],
                       [-1, 0, 1], format='lil'))
    d2[0, n-1] = 1.0
    d2[n-1, 0] = 1.0
    h1 = offset_weight * identity + 2.0 * smoothness * (d2.T @ d2)
    return sparse.block_diag((h1, speed_weight * identity), format='csc')


def candidate_speed_profile(reference, squared_speed, *, jerk_limit):
    """Adopt QP speed (never silently replace by a separately optimized profile).

    Feasibility uses the conservative regenerated envelope ONLY as an upper
    bound; it is not the returned profile. Failure raises; caller shrinks trust.
    """
    v2 = np.asarray(squared_speed, dtype=float)
    n = len(reference.target_speed)
    if (v2.shape != (n,) or not np.all(np.isfinite(v2)) or
            np.any(v2 <= 1e-6)):
        raise ValueError('invalid joint speed vector')
    speeds = np.sqrt(v2)
    local = np.asarray(reference.curvature_speed_ceiling)
    ceiling = np.asarray(reference.target_speed)
    if np.any(speeds > np.minimum(local, ceiling) + 1e-7):
        raise ValueError('joint speed exceeds conservative curvature/acceleration envelope')
    lengths = np.asarray(reference.segment_length)
    forward = np.roll(v2, -1)
    accel = (forward - v2) / (2.0 * lengths)
    if (np.max(accel) > reference.acceleration_limit + 1e-7 or
            np.min(accel) < -reference.braking_limit - 1e-7):
        raise ValueError('joint speed longitudinal constraint violation')
    segment_dt = 2.0 * lengths / (speeds + np.roll(speeds, -1))
    jerk = (np.roll(accel, -1) - accel) / np.maximum(1e-6, 0.5 * (segment_dt + np.roll(segment_dt, -1)))
    if (not np.isfinite(jerk_limit) or jerk_limit <= 0.0 or
            np.max(np.abs(jerk)) > jerk_limit + 1e-6):
        raise ValueError('joint speed jerk violation')
    t = float(np.sum(segment_dt))
    if not math.isfinite(t):
        raise ValueError('invalid joint predicted lap time')
    return replace(reference,
                   target_speed=tuple(map(float, speeds)),
                   implied_longitudinal_acceleration=tuple(map(float, accel)),
                   limiting_constraint=tuple('SCP_JOINT_SOLUTION' for _ in range(n)),
                   predicted_lap_time=t)
