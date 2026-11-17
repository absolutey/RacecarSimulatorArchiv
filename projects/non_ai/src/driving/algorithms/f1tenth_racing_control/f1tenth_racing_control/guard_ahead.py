"""Bounded, fail-closed pre-admission of ALL R-B proposals against the
same installed StoppingEnvelope used by the independent actuator Guard.

A positive result is a *proposal pre-check*, not actuation authority: the
Guard always independently validates a fresh physics state and can reject.
Never use this for issuing a high-speed qualification or bypassing Guard.
"""
from dataclasses import dataclass, replace
import math
import time

from f1tenth_racing_sim.physics.vehicle import State


@dataclass(frozen=True)
class GuardAheadResult:
    steering_rad: float
    acceleration_mps2: float
    reason: str
    proved: bool
    checks: int
    elapsed_us: float
    current_stop_proof_us: float = 0.0
    next_stop_proof_us: float = 0.0
    stopped_proof_attempts: int = 0
    cpu_elapsed_us: float = 0.0
    voluntary_context_switches: int = 0
    involuntary_context_switches: int = 0
    scheduler_gap_us: float = 0.0
    budget_left_at_start_us: float = 0.0


class GuardAheadAdmission:
    """Select an actuator-rate-compliant command with full-stop proof.

    Rejects candidates unless a certificate holds both for the observed
    state and one model-predicted extra physics tick (Guard may see N+1).
    This N+1 projection is *not* a guarantee: Guard retains final authority.
    CPU cost is bounded against the caller's 25ms full cycle budget.
    """

    PHYSICS_DT_S = 0.025
    MAX_CYCLE_NS = 25_000_000
    RESERVE_NS = 1_000_000
    MAX_EXTRA_NS = 7_000_000

    def __init__(self, stopping_envelope, dynamics_limits):
        self.stop = stopping_envelope
        self.limits = dynamics_limits

    @staticmethod
    def state_from_car(m):
        return State(
            x=float(m.px), y=float(m.py), yaw=float(m.yaw),
            speed=float(m.v), vx=float(m.vx), vy=float(m.vy),
            yaw_rate=float(m.omega), acceleration=float(m.a),
            ax=float(m.ax), ay=float(m.ay), slip=float(m.slip_angle),
            requested_acceleration=float(m.accel), jerk=float(m.jerk),
            steering=float(m.steer), steering_rate=float(m.steer_vel),
        )

    def _admissible(self, initial, steer, accel):
        # Reject ALL non-finite measured and predicted physics fields before
        # consulting the stopping model. Even a buggy/custom proof backend
        # returning qualified=True cannot bless a poisoned CarState.
        vals=(float(steer),float(accel),*(float(v) for v in vars(initial).values()))
        if not all(math.isfinite(x) for x in vals):return False
        limits=self.limits
        dt=self.PHYSICS_DT_S
        return (
            abs(steer) <= float(limits['steering_angle_abs_max_rad'])
            and abs(steer-initial.steering) <= float(limits['steering_rate_abs_max_rad_s'])*dt+1e-9
            and abs(accel-initial.requested_acceleration) <= float(limits['jerk_abs_max_m_s3'])*dt+1e-9
            and -float(limits['deceleration_max_m_s2']) <= accel <= float(limits['acceleration_max_m_s2'])
        )

    def _certificate(self, initial, steer, accel, *, deadline_ns=None):
        z=replace(initial)
        self.stop.dynamics.step(z,steer,accel,self.PHYSICS_DT_S)
        if not all(math.isfinite(float(v)) for v in vars(z).values()):
            return False
        if deadline_ns is not None and time.perf_counter_ns() >= deadline_ns:
            return False
        # Test doubles/legacy models may expose evaluate() only; the
        # installed StoppingEnvelope exposes a fail-closed bounded variant.
        if deadline_ns is not None and hasattr(self.stop,'evaluate_bounded'):
            cert=self.stop.evaluate_bounded(z,steer,accel,deadline_ns)
        else:
            cert=self.stop.evaluate(z,steer,accel)
        return bool(cert.qualified)

    def _next_state(self, current):
        # CarState carries the physically applied steering and acceleration,
        # rather than relying on an unacknowledged previous R-B intent.
        nxt=replace(current)
        self.stop.dynamics.step(nxt,current.steering,
                                current.requested_acceleration,self.PHYSICS_DT_S)
        return nxt

    def choose(self, msg, steer, accel, cycle_started_ns, prefer_applied_steer=False):
        t0=time.perf_counter_ns()
        cpu_started=time.thread_time_ns()
        import resource
        _ctx_start=resource.getrusage(resource.RUSAGE_THREAD)
        checks=0
        # Non-authorizing budget facts: detect lack of room for a full
        # current+next proof before doing any work. No deadline is relaxed.
        initial_budget_ns=min(self.MAX_EXTRA_NS,
            cycle_started_ns+self.MAX_CYCLE_NS-self.RESERVE_NS-t0)
        current_proof_ns=0
        next_proof_ns=0
        failed_candidates=0
        def done(s,a,why,ok):
            # If the CFS scheduler preempts the thread after the final check,
            # authorization must still fail closed at the response boundary.
            # No deadline extension, no shortened stop proof.
            if ok and (time.perf_counter_ns()-t0 > self.MAX_EXTRA_NS or
                       time.perf_counter_ns()-cycle_started_ns > self.MAX_CYCLE_NS-self.RESERVE_NS):
                s,a,why,ok=steer,-2.0,'GUARD_AHEAD_RESPONSE_DEADLINE_UNPROVEN',False
            _ctx_end=resource.getrusage(resource.RUSAGE_THREAD)
            elapsed_us=(time.perf_counter_ns()-t0)/1000.0
            cpu_us=(time.thread_time_ns()-cpu_started)/1000.0
            return GuardAheadResult(float(s),float(a),why,bool(ok),checks,
                                    elapsed_us,
                                    current_proof_ns/1000.0, next_proof_ns/1000.0,
                                    failed_candidates, cpu_us,
                                    max(0,_ctx_end.ru_nvcsw-_ctx_start.ru_nvcsw),
                                    max(0,_ctx_end.ru_nivcsw-_ctx_start.ru_nivcsw),
                                    max(0.0,elapsed_us-cpu_us),
                                    max(0.0,initial_budget_ns/1000.0))
        # Both controller 25ms reserve and internal 7ms limit apply to
        # the SAME exact full stop proofs; a deadline only returns UNPROVEN.
        proof_deadline_ns=min(t0+self.MAX_EXTRA_NS,
                              cycle_started_ns+self.MAX_CYCLE_NS-self.RESERVE_NS)
        if initial_budget_ns<=0:
            return done(steer,-2.0,'GUARD_AHEAD_NO_TIME_SLOT',False)
        try:
            current=self.state_from_car(msg)
            next_state=self._next_state(current)
            limits=self.limits
            # Prefer the full request. If it cannot stop safely, reduce
            # acceleration while retaining trajectory-following steering.
            # Explore a rate-constrained current-steering backup only when
            # necessary. No map-specific pose or speed magic constants.
            # Startup steering handoff has already finished above the
            # fallback's 0.75 m/s blend threshold. At that point trying a
            # speculative +4 m/s^2 certificate first can consume the entire
            # 7 ms budget before the known-safe moderated candidate is tried.
            # This changes *priority* only, not the admissibility predicates
            # nor either independent stop certificate. Below the blend speed,
            # retain the original request first when it is non-escalating;
            # when an already positive applied acceleration would be raised,
            # prioritize the jerk-valid moderated candidate. The original
            # request still receives its full certificate when considered.
            raw=float(accel)
            moderate=min(raw,2.0)
            # With an already positive physically applied acceleration, a
            # nominal +2.0 proposal may violate the 25ms jerk-rate contract.
            # Include its nearest admissible projection instead of skipping
            # directly from an unsafe primary to complete braking. This is
            # only a candidate: BOTH stop certificates are still mandatory.
            max_delta=float(limits['jerk_abs_max_m_s3'])*self.PHYSICS_DT_S
            admissible_moderate=max(
                current.requested_acceleration-max_delta,
                min(moderate,current.requested_acceleration+max_delta),
            )
            if raw>2.0 and (abs(current.speed)>=0.75 or
                            raw>current.requested_acceleration+1e-9):
                accels=list(dict.fromkeys([
                    admissible_moderate,raw,moderate,min(raw,0.0),-2.0]))
            else:
                accels=list(dict.fromkeys([
                    raw,admissible_moderate,moderate,min(raw,0.0),-2.0]))
            steers=list(dict.fromkeys([float(steer),float(current.steering)]))
            # V4 incident: the STABILIZE braking angle collided in the stop
            # corridor while holding physically-applied steering was certified.
            # With a nonpositive request probe steering alternatives at the SAME
            # acceleration before changing longitudinal braking. This is NOT a
            # bypass: each proposal must pass both current and N+1 certificates.
            if raw <= 0.0:
                # In STABILIZE/REJOIN the physical steering is a trusted
                # rate-compliant candidate, NOT a trusted safety certificate.
                # Probe its stop envelope first to keep the recovery candidate
                # inside the 7ms wall-clock proof budget whenever possible.
                if prefer_applied_steer:
                    steers=list(dict.fromkeys([float(current.steering),float(steer)]))
                candidates = list(dict.fromkeys(
                    [(s, raw) for s in steers] +
                    [(s, a) for a in accels for s in steers if a != raw]
                ))
            else:
                candidates = [(s, a) for s in steers for a in accels]
            for s, a in candidates:
                    now=time.perf_counter_ns()
                    if (now-t0 > self.MAX_EXTRA_NS or
                        now-cycle_started_ns > self.MAX_CYCLE_NS-self.RESERVE_NS):
                        return done(steer,-2.0,'GUARD_AHEAD_BUDGET_UNPROVEN',False)
                    if not self._admissible(current,s,a):continue
                    if not self._admissible(next_state,s,a):continue
                    if hasattr(self.stop,'evaluate_dual'):
                        # Native proof runs BOTH unchanged 25ms->full-stop
                        # rollouts under the SAME 7ms budget, with ONE C ABI
                        # call. No cached results, no shortened stop horizon.
                        # Activation requires independent per-map parity.
                        proof_start_ns=time.perf_counter_ns()
                        both,current_ok,next_ok,ran_next=self.stop.evaluate_dual(
                            current,s,a,current.steering,
                            current.requested_acceleration,proof_deadline_ns)
                        current_proof_ns+=time.perf_counter_ns()-proof_start_ns
                        checks+=1+int(ran_next)
                        if not both:failed_candidates+=1
                        if (time.perf_counter_ns()-t0>self.MAX_EXTRA_NS or
                                time.perf_counter_ns()-cycle_started_ns>
                                self.MAX_CYCLE_NS-self.RESERVE_NS):
                            return done(steer,-2.0,'GUARD_AHEAD_BUDGET_UNPROVEN',False)
                        if both:
                            return done(s,a,'GUARD_AHEAD_CURRENT_AND_NEXT_TICK',True)
                        continue
                    checks+=1
                    proof_start_ns=time.perf_counter_ns()
                    current_ok=self._certificate(current,s,a,deadline_ns=proof_deadline_ns)
                    current_proof_ns+=time.perf_counter_ns()-proof_start_ns
                    if not current_ok:
                        failed_candidates+=1
                        continue
                    if (time.perf_counter_ns()-t0 > self.MAX_EXTRA_NS or
                        time.perf_counter_ns()-cycle_started_ns > self.MAX_CYCLE_NS-self.RESERVE_NS):
                        return done(steer,-2.0,'GUARD_AHEAD_BUDGET_UNPROVEN',False)
                    checks+=1
                    proof_start_ns=time.perf_counter_ns()
                    next_ok=self._certificate(next_state,s,a,deadline_ns=proof_deadline_ns)
                    next_proof_ns+=time.perf_counter_ns()-proof_start_ns
                    if not next_ok:
                        failed_candidates+=1
                    if (time.perf_counter_ns()-t0 > self.MAX_EXTRA_NS or
                        time.perf_counter_ns()-cycle_started_ns > self.MAX_CYCLE_NS-self.RESERVE_NS):
                        return done(steer,-2.0,'GUARD_AHEAD_BUDGET_UNPROVEN',False)
                    if next_ok:
                        return done(s,a,'GUARD_AHEAD_CURRENT_AND_NEXT_TICK',True)
            return done(steer,-2.0,'GUARD_AHEAD_NO_QUALIFIED_COMMAND',False)
        except (ArithmeticError,ValueError,OverflowError,TypeError,KeyError,AttributeError) as exc:
            return done(steer,-2.0,'GUARD_AHEAD_MODEL_UNPROVEN:'+type(exc).__name__,False)
