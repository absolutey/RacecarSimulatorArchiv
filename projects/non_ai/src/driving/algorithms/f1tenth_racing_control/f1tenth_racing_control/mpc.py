from dataclasses import dataclass
import math
import time

import numpy as np

from f1tenth_racing_geometry import interpolate_periodic_scalar, wrap_angle

from f1tenth_racing_sim.physics.vehicle import Dynamics, State

from .core import EgoState, clamp


@dataclass(frozen=True)
class MpcConfig:
    horizon_steps: int
    timestep_s: float
    maximum_solver_iterations: int
    solver_penalty: float
    solver_tolerance: float
    primary_deadline_s: float
    lateral_error_weight: float
    longitudinal_error_weight: float
    heading_error_weight: float
    speed_error_weight: float
    slip_weight: float
    yaw_rate_weight: float
    steering_state_weight: float
    acceleration_state_weight: float
    steering_input_weight: float
    acceleration_input_weight: float
    steering_rate_weight: float
    acceleration_rate_weight: float
    preview_distance_m: float
    preview_step_m: float
    preview_braking_mps2: float
    lateral_acceleration_limit_mps2: float
    lateral_error_limit_m: float
    slip_angle_limit_rad: float
    fallback_recoverable_lateral_error_m: float
    fallback_recoverable_heading_error_rad: float

    def __post_init__(self):
        if self.horizon_steps < 2 or self.maximum_solver_iterations < 1:
            raise ValueError("MPC horizon and iteration limit must be positive")
        values = tuple(value for value in self.__dict__.values() if not isinstance(value, int))
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("MPC values must be finite and positive")


@dataclass(frozen=True)
class MpcResult:
    steering_rad: float
    acceleration_mps2: float
    solve_time_s: float
    iterations: int
    status: str
    target_speed_mps: float
    speed_limit_reason: str
    objective: float
    max_constraint_residual: float
    raw_dynamic_limit_mps: float
    backward_reachable_limit_mps: float
    execution_envelope_mps: float
    effective_preview_distance_m: float
    required_braking_distance_m: float


@dataclass(frozen=True)
class SpeedEnvelopePoint:
    r_a_speed_mps: float
    raw_dynamic_limit_mps: float
    backward_reachable_limit_mps: float
    execution_envelope_mps: float
    effective_preview_distance_m: float
    required_braking_distance_m: float
    limit_reason: str


@dataclass(frozen=True)
class RecoverabilityResult:
    recoverable: bool
    reason: str
    initial_margin: float
    terminal_margin: float
    terminal_state: tuple


class ExecutionSpeedEnvelope:
    """Downward-only cyclic reachability envelope over the supplied reference."""

    def __init__(self, reference, speed_cap_mps, lateral_acceleration_mps2,
                 braking_mps2, acceleration_mps2, minimum_preview_m):
        self.reference = reference
        self.speed_cap_mps = speed_cap_mps
        self.braking_mps2 = braking_mps2
        self.acceleration_mps2 = acceleration_mps2
        self.reference = reference
        self.minimum_preview_m = minimum_preview_m
        self.stations = reference.stations
        samples = reference.samples
        if (not samples or not all(math.isfinite(float(v)) and float(v)>0.0
                for v in (speed_cap_mps, lateral_acceleration_mps2,
                          braking_mps2, acceleration_mps2, minimum_preview_m))):
            raise ValueError('MPC_ENVELOPE_PARAMETERS_INVALID')
        if any(not all(math.isfinite(float(v)) for v in (
                sample.target_speed_mps, sample.curvature_per_m,
                sample.ds_to_next_m, sample.s_m)) or
                float(sample.target_speed_mps)<0.0 or
                float(sample.ds_to_next_m)<=0.0 for sample in samples):
            raise ValueError('MPC_REFERENCE_NONFINITE_OR_INVALID')
        self.r_a = tuple(min(float(sample.target_speed_mps), speed_cap_mps)
                         for sample in samples)
        self.raw = tuple(min(
            self.r_a[index],
            math.sqrt(lateral_acceleration_mps2 /
                      max(abs(float(sample.curvature_per_m)), 1e-9)),
        ) for index, sample in enumerate(samples))
        backward = list(self.raw)
        self._relax_backward(backward, samples, braking_mps2)
        self.backward = tuple(backward)
        final = list(backward)
        self._relax_forward(final, samples, acceleration_mps2)
        self.final = tuple(final)
        self._raw_min = min(self.raw)
        if any(value < -1e-12 or value > upper + 1e-9
               for value, upper in zip(self.final, self.r_a)):
            raise ValueError("execution envelope violates R-A speed authority")

    @staticmethod
    def _relax_backward(values, samples, deceleration):
        count = len(values)
        for _ in range(count):
            changed = False
            for index in range(count - 1, -1, -1):
                following = (index + 1) % count
                reachable = math.sqrt(values[following] ** 2 +
                                      2.0 * deceleration * samples[index].ds_to_next_m)
                if reachable + 1e-12 < values[index]:
                    values[index] = reachable
                    changed = True
            if not changed:
                break

    @staticmethod
    def _relax_forward(values, samples, acceleration):
        count = len(values)
        for _ in range(count):
            changed = False
            for index in range(count):
                following = (index + 1) % count
                reachable = math.sqrt(values[index] ** 2 +
                                      2.0 * acceleration * samples[index].ds_to_next_m)
                if reachable + 1e-12 < values[following]:
                    values[following] = reachable
                    changed = True
            if not changed:
                break

    def _scalar(self, values, station):
        return interpolate_periodic_scalar(
            self.stations, values, station, self.reference.station_period_m,
        )

    def execution_limit_at(self, station):
        """Exact scalar-only view of at(station).execution_envelope_mps.

        B0 and pursuit require the already-computed final v(s) only; the
        full at() additionally scans the entire cyclic map to report
        explanatory braking preview distances. Never approximate v(s),
        alter R-A authority, or skip a stopping certificate here.
        """
        return self._scalar(self.final, station)

    def acceleration_at(self, station):
        # Longitudinal acceleration consistent with executable v(s).
        station = float(station) % self.reference.station_period_m
        stations = self.reference.stations
        index = len(stations) - 1
        for candidate in range(len(stations) - 1):
            if stations[candidate] <= station < stations[candidate + 1]:
                index = candidate
                break
        following = (index + 1) % len(self.reference.samples)
        ds = max(float(self.reference.samples[index].ds_to_next_m), 1e-9)
        v0 = float(self.final[index])
        v1 = float(self.final[following])
        acceleration = (v1 * v1 - v0 * v0) / (2.0 * ds)
        return max(-self.braking_mps2, min(self.acceleration_mps2, acceleration))

    def at(self, station):
        r_a = self._scalar(self.r_a, station)
        raw = self._scalar(self.raw, station)
        backward = self._scalar(self.backward, station)
        final = self._scalar(self.final, station)
        required = 0.0
        # If every raw speed is at least this local R-A ceiling,
        # the physical braking distance is exactly zero everywhere.
        # Otherwise use the original exhaustive, conservative preview.
        if r_a > self._raw_min:
            period = self.reference.station_period_m
            for sample, future_limit in zip(self.reference.samples, self.raw):
                distance = (sample.s_m - station) % period
                braking_distance = max(0.0, (r_a * r_a - future_limit * future_limit) /
                                       (2.0 * self.braking_mps2))
                if braking_distance + 1e-9 >= distance:
                    required = max(required, braking_distance)
        if final + 1e-9 < backward:
            reason = "ACCELERATION_REACHABILITY"
        elif backward + 1e-9 < raw:
            reason = "BRAKING_REACHABILITY"
        elif raw + 1e-9 < r_a:
            reason = "LATERAL_DYNAMIC_LIMIT"
        else:
            reason = "R_A_LIMIT"
        return SpeedEnvelopePoint(
            r_a, raw, backward, final, max(self.minimum_preview_m, required),
            required, reason,
        )


class EnvelopeBoundReference:
    """Pursuit-compatible view whose speed scalar is bounded by one envelope."""

    def __init__(self, reference, envelope):
        self.reference = reference
        self.envelope = envelope
        self.geometry = reference.geometry
        self.station_period_m = reference.station_period_m
        # Preserve the PursuitReference iteration contract. The bounded
        # scalar() method continues to enforce the execution envelope.
        self.samples = reference.samples

    def station_at_physical(self, physical_sample):
        return self.reference.station_at_physical(physical_sample)

    def scalar(self, field, station):
        value = self.reference.scalar(field, station)
        if field == "target_speed_mps":
            return min(value, self.envelope.execution_limit_at(station))
        if field == "target_accel_mps2":
            return self.envelope.acceleration_at(station)
        return value

    def angle(self, field, station):
        return self.reference.angle(field, station)


class PrimaryAdoptionGate:
    """Time hysteresis derived from one MPC prediction horizon."""

    def __init__(self, horizon_duration_s):
        self.horizon_duration_s = horizon_duration_s
        self.reset()

    def reset(self):
        self.state = "FALLBACK"
        self.feasible_since_s = None
        self.infeasible_since_s = None

    def update(self, feasible, now_s):
        if feasible:
            self.infeasible_since_s = None
            if self.state == "PRIMARY":
                return True, "PRIMARY_CONTINUATION", self.horizon_duration_s
            if self.feasible_since_s is None:
                self.feasible_since_s = now_s
            duration = now_s - self.feasible_since_s
            if duration + 1e-12 >= self.horizon_duration_s:
                self.state = "PRIMARY"
                return True, "PRIMARY_ADMITTED", duration
            self.state = "ADMITTING"
            return False, "PRIMARY_ADMISSION_PENDING", duration
        self.feasible_since_s = None
        if self.state == "PRIMARY":
            if self.infeasible_since_s is None:
                self.infeasible_since_s = now_s
            duration = now_s - self.infeasible_since_s
            if duration + 1e-12 < self.horizon_duration_s:
                return False, "PRIMARY_CONTINUATION_GAP", duration
        self.state = "FALLBACK"
        self.infeasible_since_s = now_s
        return False, "FALLBACK_REQUIRED", 0.0


class B0RecoverabilityGovernor:
    """Accept primary only when the installed B0 policy preserves its safe tube."""

    def __init__(self, mpc, fallback_controller, stopping_checker=None):
        self.mpc = mpc
        self.fallback_controller = fallback_controller
        self.stopping_checker = stopping_checker
        self.fallback_supervisor = None

    def _margin(self, reference, envelope, vector):
        # Preserve the original two-field public/internal B0 contract.
        margin, component, _station, _admissible = self._margin_detail(
            reference, envelope, vector)
        return margin, component

    def _margin_detail(self, reference, envelope, vector):
        # Single geometric projection shared by the B0 margin and fallback
        # headroom. Same formulas, limit reasons, and ordered tie-breaks.
        projection = reference.geometry.project(float(vector[0]), float(vector[1]))
        sample = reference.geometry.interpolate(projection.s)
        station = reference.station_at_physical(sample)
        speed = max(float(vector[3]), 0.0)
        admissible_speed = envelope.execution_limit_at(station)
        values = {
            "LATERAL_ERROR": (
                self.mpc.config.fallback_recoverable_lateral_error_m - abs(projection.d)
            ) / self.mpc.config.fallback_recoverable_lateral_error_m,
            "HEADING_ERROR": (
                self.mpc.config.fallback_recoverable_heading_error_rad -
                abs(wrap_angle(reference.angle("heading_rad", station) - float(vector[2])))
            ) / self.mpc.config.fallback_recoverable_heading_error_rad,
            "SLIP": (
                self.mpc.config.slip_angle_limit_rad - abs(float(vector[4]))
            ) / self.mpc.config.slip_angle_limit_rad,
            "YAW_RATE": (
                self.mpc.config.lateral_acceleration_limit_mps2 / max(speed, 0.5) -
                abs(float(vector[5]))
            ) / (self.mpc.config.lateral_acceleration_limit_mps2 / max(speed, 0.5)),
            "SPEED_ENVELOPE": (
                admissible_speed - speed
            ) / max(admissible_speed, 0.5),
        }
        reason = min(values, key=values.get)
        return values[reason], reason, station, admissible_speed

    def evaluate(self, reference, envelope, state_message, primary_control,
                 deadline_ns=None):
        # The solver deadline does not bound B0. The whole B0 rollout and
        # terminal swept-map stop proof must also be budgeted. Exhaustion is
        # NOT a recoverability certificate and NEVER causes primary adoption.
        b0_started_ns = time.perf_counter_ns()
        self.last_profile = {
            "phase": "ENTRY", "elapsed_us": 0.0,
            "rollout_steps": 0, "stop_check_attempted": False,
            "stop_check_us": 0.0,
        }
        def mark(phase):
            self.last_profile["phase"] = phase
            self.last_profile["elapsed_us"] = (
                time.perf_counter_ns() - b0_started_ns) / 1000.0
        vector = self.mpc.state_vector(state_message)
        def expired():
            return deadline_ns is not None and time.perf_counter_ns() >= deadline_ns
        def unproven(state, initial=math.nan, terminal=math.nan):
            mark("DEADLINE_UNPROVEN")
            return RecoverabilityResult(
                False, "B0_DEADLINE_UNPROVEN", initial, terminal,
                tuple(float(value) for value in state),
            )
        if expired():
            return unproven(vector)
        bounded = EnvelopeBoundReference(reference, envelope)
        previous = tuple(float(value) for value in primary_control)
        vector = self.mpc._step(vector, previous, vector[2])
        if expired():
            return unproven(vector)
        initial_margin, limiting = self._margin(reference, envelope, vector)
        if expired():
            return unproven(vector, initial_margin, initial_margin)
        if initial_margin < 0.0:
            mark("PRIMARY_NEXT_SAFETY_REJECT")
            return RecoverabilityResult(
                False, f"PRIMARY_NEXT_{limiting}", initial_margin, initial_margin,
                tuple(float(value) for value in vector),
            )
        terminal_margin = initial_margin
        # B0 must predict the installed startup fallback control law.
        # Copy only mutable phase bits; real supervisor remains untouched.
        from copy import copy
        from types import SimpleNamespace
        shadow = (copy(self.fallback_supervisor)
                  if self.fallback_supervisor is not None else None)
        mark("B0_ROLLOUT")
        for _ in range(self.mpc.config.horizon_steps):
            self.last_profile["rollout_steps"] += 1
            if expired():
                return unproven(vector, initial_margin, terminal_margin)
            state = EgoState(float(vector[0]), float(vector[1]), float(vector[2]), float(vector[3]))
            decision = self.fallback_controller.compute(
                bounded, state, previous[0], previous[1], self.mpc.config.timestep_s,
            )
            if expired():
                return unproven(vector, initial_margin, terminal_margin)
            if shadow is not None:
                decision, _ = shadow.shape(
                    SimpleNamespace(v=float(vector[3])), decision, previous,
                    self.mpc.config.timestep_s,
                )
                if expired():
                    return unproven(vector, initial_margin, terminal_margin)
            previous = (decision.steering_rad, decision.acceleration_mps2)
            vector = self.mpc._step(vector, previous, vector[2])
            if expired():
                return unproven(vector, initial_margin, terminal_margin)
            terminal_margin, limiting = self._margin(reference, envelope, vector)
            if expired():
                return unproven(vector, initial_margin, terminal_margin)
            if terminal_margin < 0.0:
                mark("B0_ROLLOUT_SAFETY_REJECT")
                return RecoverabilityResult(
                    False, f"B0_ROLLOUT_{limiting}", initial_margin, terminal_margin,
                    tuple(float(value) for value in vector),
                )
        # Nondecreasing normalized margin is NOT a recoverability proof.
        # The exact evolved physical state must have a locally certified
        # stopping rollout under the same model and swept map geometry.
        if expired():
            return unproven(vector, initial_margin, terminal_margin)
        certified=False
        mark("B0_STOP_CHECK")
        stop_started_ns = time.perf_counter_ns()
        if self.stopping_checker is not None:
            self.last_profile["stop_check_attempted"] = True
            try:
                certified=bool(self.stopping_checker(self.mpc._state(vector),
                                                     previous[0],previous[1]))
            except (ArithmeticError, ValueError, TypeError, RuntimeError):
                certified=False
        self.last_profile["stop_check_us"] = (
            time.perf_counter_ns() - stop_started_ns) / 1000.0
        # A proof completed AFTER its allocated deadline is not admissible.
        # The Guard's independent certificate remains unchanged.
        if expired():
            return unproven(vector, initial_margin, terminal_margin)
        mark("B0_CERTIFIED" if certified else "B0_STOP_UNPROVEN")
        recoverable = certified
        return RecoverabilityResult(
            recoverable,
            "B0_STOP_CERTIFIED" if recoverable
            else "B0_STOP_UNPROVEN_OR_FAILED",
            initial_margin, terminal_margin, tuple(float(value) for value in vector),
        )


class MpcCycleBudgetExceeded(RuntimeError):
    """Wall-clock budget exhaustion, not a proof of QP infeasibility."""
    def __init__(self, stage, iterations, started_ns):
        self.stage = str(stage)
        self.iterations = int(iterations)
        self.elapsed_ns = max(0, time.perf_counter_ns() - started_ns)
        super().__init__("MPC_CYCLE_BUDGET_EXCEEDED")


def _qp(hessian, gradient, a, b, low, high, initial, config,
        deadline_ns=None, started_ns=None):
    # Deadline is an additional admission constraint; no inequalities are
    # relaxed and no unconverged QP output is returned as PRIMARY_VALID.
    started_ns = time.perf_counter_ns() if started_ns is None else started_ns
    def within(stage, iterations):
        if deadline_ns is not None and time.perf_counter_ns() >= deadline_ns:
            raise MpcCycleBudgetExceeded(stage, iterations, started_ns)
    within("QP_SETUP", 0)
    rho = config.solver_penalty
    x = np.clip(initial, low, high)
    transpose = a.T
    system_inverse = np.linalg.inv(hessian + rho * (transpose @ a) + rho * np.eye(len(x)))
    within("QP_FACTOR", 0)
    z, y = np.minimum(a @ x, b), x.copy()
    uz, uy = np.zeros(len(b)), np.zeros(len(x))
    for iteration in range(1, config.maximum_solver_iterations + 1):
        within("QP_ITERATION_START", iteration - 1)
        x = system_inverse @ (-gradient + rho * transpose @ (z - uz) + rho * (y - uy))
        old_z, old_y = z, y
        ax = a @ x
        z = np.minimum(ax + uz, b)
        y = np.clip(x + uy, low, high)
        uz += ax - z
        uy += x - y
        primal = max(np.max(np.abs(ax - z)), np.max(np.abs(x - y)))
        dual = rho * max(np.max(np.abs(transpose @ (z - old_z))), np.max(np.abs(y - old_y)))
        scale = max(1.0, np.max(np.abs(x)), np.max(np.abs(ax)))
        # The physical constraint contract is absolute, not normalized by
        # control/state units. Reject candidate solutions with unqualified
        # inequalities even if scaled ADMM residuals appear small.
        absolute_inequality = max(0.0, float(np.max(a @ y - b)))
        absolute_bounds = max(0.0, float(np.max(low - y)), float(np.max(y - high)))
        if (max(primal, dual) / scale <= config.solver_tolerance and
                max(absolute_inequality, absolute_bounds) <= config.solver_tolerance):
            within("QP_ACCEPT", iteration)
            return y, iteration
    # Exhaustion does not prove that the constrained QP is infeasible.
    # Distinguish numerical/nonconvergence from genuinely unsatisfiable bounds.
    within("QP_ITERATION_LIMIT", config.maximum_solver_iterations)
    raise RuntimeError("MPC_QP_ITERATION_LIMIT")


class LtvDynamicBicycleMpc:
    STATE_SIZE = 8
    INPUT_SIZE = 2

    def __init__(self, dynamics_config, limits, config):
        self.dynamics_config = dynamics_config
        self.dynamics = Dynamics(dynamics_config)
        self.limits = limits
        self.config = config
        self.warm_start = np.zeros(config.horizon_steps * self.INPUT_SIZE)
        self._envelope_key = None
        self._envelope = None

    def speed_envelope(self, reference, speed_cap_mps):
        key = (id(reference), speed_cap_mps)
        if key != self._envelope_key:
            self._envelope = ExecutionSpeedEnvelope(
                reference, speed_cap_mps,
                self.config.lateral_acceleration_limit_mps2,
                self.config.preview_braking_mps2,
                self.limits["acceleration_max_m_s2"],
                self.config.preview_distance_m,
            )
            self._envelope_key = key
        return self._envelope

    @staticmethod
    def state_vector(message):
        return np.array([
            message.px, message.py, message.yaw, message.v,
            message.slip_angle, message.omega, message.steer, message.accel,
        ], dtype=float)

    @staticmethod
    def _state(vector):
        return State(
            x=vector[0], y=vector[1], yaw=vector[2], speed=vector[3],
            slip=vector[4], yaw_rate=vector[5], steering=vector[6],
            requested_acceleration=vector[7], acceleration=vector[7],
            vx=vector[3] * math.cos(vector[4]), vy=vector[3] * math.sin(vector[4]),
        )

    def _step(self, vector, control, yaw_anchor):
        state = self.dynamics.step(
            self._state(vector), float(control[0]), float(control[1]), self.config.timestep_s
        )
        yaw = state.yaw
        while yaw - yaw_anchor > math.pi:
            yaw -= 2.0 * math.pi
        while yaw - yaw_anchor < -math.pi:
            yaw += 2.0 * math.pi
        return np.array([
            state.x, state.y, yaw, state.speed, state.slip,
            state.yaw_rate, state.steering, state.requested_acceleration,
        ])

    def _linearize(self, state, control, deadline_ns=None, started_ns=None, step_index=0):
        # Never claim that a finished but tardy linearization was timely.
        def check(phase):
            if deadline_ns is not None and time.perf_counter_ns() >= deadline_ns:
                raise MpcCycleBudgetExceeded(
                    'MPC_LINEARIZE_STEP_%d_%s' % (step_index, phase), 0,
                    started_ns if started_ns is not None else time.perf_counter_ns())
        check('ENTRY')
        base = self._step(state, control, state[2])
        check('BASE')
        a = np.empty((self.STATE_SIZE, self.STATE_SIZE))
        b = np.empty((self.STATE_SIZE, self.INPUT_SIZE))
        for index in range(self.STATE_SIZE):
            epsilon = 1e-5 * max(1.0, abs(state[index]))
            changed = state.copy(); changed[index] += epsilon
            a[:, index] = (self._step(changed, control, base[2]) - base) / epsilon
            check('STATE_%d' % index)
        for index in range(self.INPUT_SIZE):
            epsilon = 1e-5 * max(1.0, abs(control[index]))
            changed = control.copy(); changed[index] += epsilon
            b[:, index] = (self._step(state, changed, base[2]) - base) / epsilon
            check('CONTROL_%d' % index)
        affine = base - a @ state - b @ control
        check('COMPLETE')
        return a, b, affine

    def _local_speed_limit(self, reference, station, speed_cap_mps):
        point = self.speed_envelope(reference, speed_cap_mps).at(station)
        return point.execution_envelope_mps, point.limit_reason

    def _reference_horizon(self, reference, current_station, speed_cap_mps, yaw_anchor):
        refs, controls = [], []
        station = current_station
        previous_heading = reference.angle("heading_rad", station)
        while previous_heading - yaw_anchor > math.pi:
            previous_heading -= 2.0 * math.pi
        while previous_heading - yaw_anchor < -math.pi:
            previous_heading += 2.0 * math.pi
        wheelbase = self.dynamics_config["dynamic_wheelbase_m"]
        envelope = self.speed_envelope(reference, speed_cap_mps)
        for _ in range(self.config.horizon_steps):
            # Pure v(s) lookup: the old at() computed a full-map explanatory
            # braking preview for each horizon step though it was discarded.
            speed = envelope.execution_limit_at(station)
            station = (station + speed * self.config.timestep_s) % reference.station_period_m
            heading = reference.angle("heading_rad", station)
            while heading - previous_heading > math.pi:
                heading -= 2.0 * math.pi
            while heading - previous_heading < -math.pi:
                heading += 2.0 * math.pi
            curvature = reference.scalar("curvature_per_m", station)
            x = reference.scalar("x_m", station)
            y = reference.scalar("y_m", station)
            steering = math.atan(wheelbase * curvature)
            acceleration = envelope.acceleration_at(station)
            refs.append(np.array([x, y, heading, speed, 0.0, speed * curvature, steering, acceleration]))
            controls.append(np.array([steering, acceleration]))
            previous_heading = heading
        return refs, controls

    def solve(self, reference, state_message, current_station, previous_control, speed_cap_mps,
              deadline_ns=None):
        started_ns = time.perf_counter_ns()
        started_cpu_ns = time.process_time_ns()
        self.last_microstage_times = {}
        def trace(phase):
            self.last_microstage_times[phase] = {
                'wall_us': (time.perf_counter_ns()-started_ns)/1000.0,
                'cpu_us': (time.process_time_ns()-started_cpu_ns)/1000.0,
            }
        trace('MPC_ENTRY')
        if deadline_ns is not None and started_ns >= deadline_ns:
            raise MpcCycleBudgetExceeded("MPC_ENTRY", 0, started_ns)
        state = self.state_vector(state_message)
        if not np.all(np.isfinite(state)):
            raise ValueError("MPC_INVALID_STATE")
        refs, nominal_controls = self._reference_horizon(
            reference, current_station, speed_cap_mps, state[2]
        )
        trace('MPC_REFERENCE_HORIZON_COMPLETE')
        if deadline_ns is not None and time.perf_counter_ns() >= deadline_ns:
            raise MpcCycleBudgetExceeded('MPC_REFERENCE_HORIZON', 0, started_ns)
        horizon, nx, nu = self.config.horizon_steps, self.STATE_SIZE, self.INPUT_SIZE
        mappings, offsets = [], []
        mapping = np.zeros((nx, horizon * nu))
        offset = state.copy()
        envelope_point = self.speed_envelope(reference, speed_cap_mps).at(current_station)
        local_speed, speed_reason = (
            envelope_point.execution_envelope_mps, envelope_point.limit_reason,
        )
        hessian = np.zeros((horizon * nu, horizon * nu))
        gradient = np.zeros(horizon * nu)
        rows, limits = [], []
        for step in range(horizon):
            if deadline_ns is not None and time.perf_counter_ns() >= deadline_ns:
                raise MpcCycleBudgetExceeded('MPC_BEFORE_LINEARIZE_%d' % step, 0, started_ns)
            a, b, affine = self._linearize(refs[step], nominal_controls[step],
                deadline_ns=deadline_ns, started_ns=started_ns, step_index=step)
            trace('MPC_LINEARIZE_%d_COMPLETE' % step)
            mapping = a @ mapping
            mapping[:, step * nu:(step + 1) * nu] += b
            offset = a @ offset + affine
            mappings.append(mapping.copy()); offsets.append(offset.copy())
            heading = refs[step][2]
            tangent = np.array([math.cos(heading), math.sin(heading)])
            normal = np.array([-tangent[1], tangent[0]])
            q = np.diag([
                self.config.longitudinal_error_weight,
                self.config.lateral_error_weight,
                self.config.heading_error_weight,
                self.config.speed_error_weight,
                self.config.slip_weight,
                self.config.yaw_rate_weight,
                self.config.steering_state_weight,
                self.config.acceleration_state_weight,
            ])
            q[:2, :2] = self.config.longitudinal_error_weight * np.outer(tangent, tangent) + self.config.lateral_error_weight * np.outer(normal, normal)
            error = offset - refs[step]
            hessian += 2.0 * mapping.T @ q @ mapping
            gradient += 2.0 * mapping.T @ q @ error
            normal_mapping = normal @ mapping[:2]
            normal_offset = normal @ (offset[:2] - refs[step][:2])
            rows.extend((normal_mapping, -normal_mapping))
            limits.extend((self.config.lateral_error_limit_m - normal_offset,
                           self.config.lateral_error_limit_m + normal_offset))
            if deadline_ns is not None and time.perf_counter_ns() >= deadline_ns:
                raise MpcCycleBudgetExceeded('MPC_MATRIX_ASSEMBLY_%d' % step, 0, started_ns)
            for state_index, bound in (
                (4, self.config.slip_angle_limit_rad),
                (5, self.config.lateral_acceleration_limit_mps2 /
                    max(refs[step][3], 0.5)),
            ):
                rows.extend((mapping[state_index], -mapping[state_index]))
                limits.extend((bound - offset[state_index], bound + offset[state_index]))
        r = np.tile(np.array([self.config.steering_input_weight, self.config.acceleration_input_weight]), horizon)
        nominal = np.concatenate(nominal_controls)
        hessian += 2.0 * np.diag(r)
        gradient -= 2.0 * r * nominal
        previous = np.asarray(previous_control, dtype=float)
        rate_limits = np.array([
            self.limits["steering_rate_abs_max_rad_s"] * self.config.timestep_s,
            self.limits["jerk_abs_max_m_s3"] * self.config.timestep_s,
        ])
        rate_weights = np.array([self.config.steering_rate_weight, self.config.acceleration_rate_weight])
        for step in range(horizon):
            for channel in range(nu):
                index = step * nu + channel
                difference = np.zeros(horizon * nu); difference[index] = 1.0
                reference_value = previous[channel] if step == 0 else 0.0
                if step > 0:
                    difference[index - nu] = -1.0
                rows.extend((difference, -difference))
                limits.extend((rate_limits[channel] + reference_value, rate_limits[channel] - reference_value))
                hessian += 2.0 * rate_weights[channel] * np.outer(difference, difference)
                gradient -= 2.0 * rate_weights[channel] * reference_value * difference
        low = np.tile(np.array([-self.limits["steering_angle_abs_max_rad"], -self.limits["deceleration_max_m_s2"]]), horizon)
        high = np.tile(np.array([self.limits["steering_angle_abs_max_rad"], self.limits["acceleration_max_m_s2"]]), horizon)
        trace('MPC_QP_ASSEMBLY_COMPLETE')
        if deadline_ns is not None and time.perf_counter_ns() >= deadline_ns:
            raise MpcCycleBudgetExceeded("MPC_QP_ASSEMBLY", 0, started_ns)
        solution, iterations = _qp(
            hessian + 1e-8 * np.eye(horizon * nu), gradient,
            np.asarray(rows), np.asarray(limits), low, high,
            np.clip(self.warm_start, low, high), self.config,
            deadline_ns=deadline_ns, started_ns=started_ns,
        )
        trace('MPC_QP_COMPLETE')
        if not np.all(np.isfinite(solution)):
            raise ValueError("MPC_INVALID_SOLUTION")
        # ADMM's *scaled* stopping test alone is insufficient to certify
        # absolute physical inequalities: historical telemetry contained
        # PRIMARY_VALID rows with residual > solver_tolerance.
        inequality_violation = max(
            0.0,
            float(np.max(np.asarray(rows) @ solution - np.asarray(limits))),
        )
        bound_violation = max(
            0.0,
            float(np.max(low - solution)),
            float(np.max(solution - high)),
        )
        if max(inequality_violation, bound_violation) > self.config.solver_tolerance:
            raise RuntimeError("MPC_QP_CONSTRAINT_RESIDUAL")
        if deadline_ns is not None and time.perf_counter_ns() >= deadline_ns:
            raise MpcCycleBudgetExceeded("MPC_POST_QP", iterations, started_ns)
        # An already-late primary never seeds the next physical tick.
        # Otherwise a solver outside its admission window contaminates the
        # next run even though its command is explicitly unqualified.
        elapsed = (time.perf_counter_ns() - started_ns) * 1e-9
        if elapsed <= self.config.primary_deadline_s:
            self.warm_start = np.concatenate((solution[nu:], solution[-nu:]))
        steering = clamp(
            float(solution[0]),
            previous[0] - rate_limits[0],
            previous[0] + rate_limits[0],
        )
        acceleration = clamp(
            float(solution[1]),
            previous[1] - rate_limits[1],
            previous[1] + rate_limits[1],
        )
        objective = float(0.5 * solution @ hessian @ solution + gradient @ solution)
        residual = max(0.0, float(np.max(np.asarray(rows) @ solution - np.asarray(limits))))
        status = "PRIMARY_LATE" if elapsed > self.config.primary_deadline_s else "PRIMARY_VALID"
        return MpcResult(
            steering, acceleration, elapsed, iterations, status, local_speed,
            speed_reason, objective, residual,
            envelope_point.raw_dynamic_limit_mps,
            envelope_point.backward_reachable_limit_mps,
            envelope_point.execution_envelope_mps,
            envelope_point.effective_preview_distance_m,
            envelope_point.required_braking_distance_m,
        )


class NonlinearDynamicBicycleMpc(LtvDynamicBicycleMpc):
    """Constrained full-dynamics candidate solver for B2 research selection.

    Unlike the linearized QP controller, every optimization evaluation rolls
    out the simulator's nonlinear dynamic bicycle model. Candidate admission
    requires hard bounds on state, controls, rates and Frenet path errors.
    A failed/incomplete/late solve raises and the existing R-B fallback plus
    independent Guard retain all actuator authority. Not a realtime claim.
    """
    formulation_id = 'NMPC_NONLINEAR_DYNAMICS_SLSQP_V1'

    def solve(self, reference, state_message, current_station, previous_control,
              speed_cap_mps, deadline_ns=None):
        from scipy.optimize import minimize
        begin = time.perf_counter_ns()

        def check_budget(stage):
            if deadline_ns is not None and time.perf_counter_ns() >= deadline_ns:
                raise MpcCycleBudgetExceeded(stage, 0, begin)

        check_budget('NMPC_ENTRY')
        measured = self.state_vector(state_message)
        if not np.isfinite(measured).all():
            raise ValueError('NMPC_INVALID_MEASURED_STATE')
        previous = np.asarray(previous_control, dtype=float)
        if previous.shape != (2,) or not np.isfinite(previous).all():
            raise ValueError('NMPC_INVALID_PREVIOUS_CONTROL')
        if not math.isfinite(speed_cap_mps) or speed_cap_mps <= 0.:
            raise ValueError('NMPC_SPEED_AUTHORITY_INVALID')
        refs, control_refs = self._reference_horizon(
            reference, current_station, speed_cap_mps, measured[2])
        envelope = self.speed_envelope(reference, speed_cap_mps).at(current_station)
        n = self.config.horizon_steps
        dt = self.config.timestep_s
        limits = self.limits
        lo = np.tile([-limits['steering_angle_abs_max_rad'],
                      -limits['deceleration_max_m_s2']], n)
        hi = np.tile([limits['steering_angle_abs_max_rad'],
                      limits['acceleration_max_m_s2']], n)
        rate = np.array([limits['steering_rate_abs_max_rad_s']*dt,
                         limits['jerk_abs_max_m_s3']*dt])
        # Re-derive complete trajectories from input on every evaluation.
        # Never reuse linearized or precomputed dynamics as a safety witness.
        def rollout(vector):
            check_budget('NMPC_NONLINEAR_ROLLOUT')
            u = vector.reshape(n, 2)
            states = []
            current = measured.copy()
            for control in u:
                current = self._step(current, control, current[2])
                if not np.isfinite(current).all():
                    raise ValueError('NMPC_NONFINITE_ROLLOUT')
                states.append(current)
            return u, states

        def residual(vector):
            u, states = rollout(vector)
            constraints = []
            prior = previous
            for k,(cmd, actual) in enumerate(zip(u,states)):
                ref = refs[k]
                heading = ref[2]
                normal = np.array([-math.sin(heading), math.cos(heading)])
                lateral = float(normal @ (actual[:2]-ref[:2]))
                lateral_acceleration = abs(float(actual[3]*actual[5]))
                constraints.extend((
                    self.config.lateral_error_limit_m-abs(lateral),
                    self.config.slip_angle_limit_rad-abs(float(actual[4])),
                    self.config.lateral_acceleration_limit_mps2-lateral_acceleration,
                    speed_cap_mps-abs(float(actual[3])),
                    float(actual[3]),  # Racing Ego cannot reverse to satisfy pass objective.
                    rate[0]-abs(float(cmd[0]-prior[0])),
                    rate[1]-abs(float(cmd[1]-prior[1])),
                ))
                prior = cmd
            return np.asarray(constraints)

        def objective(vector):
            u, states = rollout(vector)
            result = 0.
            prior = previous
            for k,(cmd,actual) in enumerate(zip(u,states)):
                ref = refs[k]
                heading = ref[2]
                tangent = np.array([math.cos(heading),math.sin(heading)])
                normal = np.array([-tangent[1],tangent[0]])
                delta = actual[:2]-ref[:2]
                result += self.config.longitudinal_error_weight*float(tangent@delta)**2
                result += self.config.lateral_error_weight*float(normal@delta)**2
                for i,w in ((2,self.config.heading_error_weight),
                            (3,self.config.speed_error_weight),
                            (4,self.config.slip_weight),
                            (5,self.config.yaw_rate_weight),
                            (6,self.config.steering_state_weight),
                            (7,self.config.acceleration_state_weight)):
                    result += w*float(actual[i]-ref[i])**2
                result += self.config.steering_input_weight*(cmd[0]-control_refs[k][0])**2
                result += self.config.acceleration_input_weight*(cmd[1]-control_refs[k][1])**2
                result += self.config.steering_rate_weight*(cmd[0]-prior[0])**2
                result += self.config.acceleration_rate_weight*(cmd[1]-prior[1])**2
                prior = cmd
            return float(result)

        initial = np.clip(self.warm_start.copy(), lo, hi)
        # The initial guess must meet control-rate bounds; warm starts from a
        # previous physical tick are not automatically admissible.
        for k in range(n):
            for j in range(2):
                i = 2*k+j
                previous_value = previous[j] if k == 0 else initial[i-2]
                initial[i] = clamp(initial[i],previous_value-rate[j],
                                   previous_value+rate[j])
        check_budget('NMPC_BEFORE_OPTIMIZE')
        result = minimize(objective, initial, method='SLSQP',
                          bounds=list(zip(lo,hi)),
                          constraints=({'type':'ineq','fun':residual},),
                          options={'ftol':min(self.config.solver_tolerance,1e-4),
                                   'maxiter':self.config.maximum_solver_iterations,
                                   'disp':False})
        check_budget('NMPC_AFTER_OPTIMIZE')
        candidate = np.asarray(result.x, dtype=float)
        if not np.isfinite(candidate).all() or not bool(result.success):
            raise RuntimeError('NMPC_SOLVER_NOT_CONVERGED')
        exact = residual(candidate)
        if (not np.isfinite(exact).all() or
                float(np.min(exact)) < -self.config.solver_tolerance or
                np.any(candidate<lo-self.config.solver_tolerance) or
                np.any(candidate>hi+self.config.solver_tolerance)):
            raise RuntimeError('NMPC_NONLINEAR_CONSTRAINT_UNPROVEN')
        check_budget('NMPC_FINAL_ADMISSION')
        elapsed = (time.perf_counter_ns()-begin)*1e-9
        if elapsed > self.config.primary_deadline_s:
            # No warm-start advance for a late, unqualified result.
            raise MpcCycleBudgetExceeded('NMPC_PRIMARY_DEADLINE',int(result.nit),begin)
        self.warm_start = np.concatenate((candidate[2:],candidate[-2:]))
        return MpcResult(float(candidate[0]),float(candidate[1]),elapsed,
                         int(result.nit),'PRIMARY_VALID',
                         envelope.execution_envelope_mps,envelope.limit_reason,
                         float(result.fun),max(0.,-float(np.min(exact))),
                         envelope.raw_dynamic_limit_mps,
                         envelope.backward_reachable_limit_mps,
                         envelope.execution_envelope_mps,
                         envelope.effective_preview_distance_m,
                         envelope.required_braking_distance_m)
