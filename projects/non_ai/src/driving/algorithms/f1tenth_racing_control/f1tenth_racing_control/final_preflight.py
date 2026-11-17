"""Fail-closed admission of the FINAL controller proposal, before ActuatorIntent.

This is a proposal pre-check, never the independent Actuator Guard authority.
No speed cap, collision threshold, or Guard certificate is loosened.
"""
from dataclasses import dataclass, replace
import math
import time
from types import SimpleNamespace


@dataclass(frozen=True)
class FinalPreflightOutcome:
    decision: object
    reason: str
    changed: bool
    rejected: bool
    # Read-only timing facts; never affect preflight/Guard authority.
    trace: object = None


def admit_final_command(decision, *, message, state_finite, actual_speed_mps,
                        reference_valid, cycle_started_ns, reserve_ns,
                        previous_steering, dt_s, guard_ahead, controller):
    """Pre-prove the full stop corridor for EVERY final steering/accel pair.

    A negative acceleration is NOT proof of clearance: a turning vehicle can
    still strike a wall while braking. The independent Guard validates again.
    """
    original_mode = str(decision.mode)
    try:
        finite = all(math.isfinite(float(v)) for v in
                     (decision.steering_rad, decision.acceleration_mps2))
    except (TypeError, ValueError, OverflowError):
        finite = False
    if not finite or not reference_valid or not state_finite:
        result = SimpleNamespace(proved=False, reason='FINAL_PREFLIGHT_INVALID_CONTEXT')
    elif time.perf_counter_ns() - cycle_started_ns > 25_000_000 - reserve_ns:
        result = SimpleNamespace(proved=False, reason='CYCLE_RESERVE_EXHAUSTED')
    else:
        try:
            # Keep the legacy four-argument certification interface for
            # standard modes. The priority hint is needed ONLY by the V5
            # STABILIZE/REJOIN corridor recovery and is part of the newer
            # GuardAheadAdmission interface.
            if original_mode in ('STABILIZE', 'REJOIN'):
                result = guard_ahead.choose(
                    message, decision.steering_rad,
                    decision.acceleration_mps2, cycle_started_ns,
                    prefer_applied_steer=True,
                )
            else:
                result = guard_ahead.choose(
                    message, decision.steering_rad,
                    decision.acceleration_mps2, cycle_started_ns,
                )
        except (ArithmeticError, ValueError, TypeError, OverflowError,
                KeyError, AttributeError):
            result = SimpleNamespace(proved=False, reason='FINAL_PREFLIGHT_EXCEPTION')

    # A returned certificate must never silently turn a braking/holding
    # proposal into propulsion, and must never contain non-finite commands.
    # This guards the PRE-flight handoff; independent Guard is still final.
    if result.proved:
        try:
            certified_steer = float(result.steering_rad)
            certified_accel = float(result.acceleration_mps2)
            certificate_output_valid = (
                math.isfinite(certified_steer)
                and math.isfinite(certified_accel)
                and not (float(decision.acceleration_mps2) <= 0.0
                         and certified_accel > 0.0)
            )
        except (ValueError, TypeError, OverflowError, AttributeError):
            certificate_output_valid = False
        if not certificate_output_valid:
            result = SimpleNamespace(
                proved=False, reason='FINAL_PREFLIGHT_UNSAFE_CERT_OUTPUT'
            )

    # Report expensive stop certificates even when they are rejected.
    trace = {
        'candidate_checks': int(getattr(result, 'checks', 0)),
        'candidate_elapsed_us': float(getattr(result, 'elapsed_us', 0.0)),
        'candidate_cpu_elapsed_us': float(getattr(result, 'cpu_elapsed_us', 0.0)),
        'candidate_scheduler_gap_us': float(getattr(result, 'scheduler_gap_us', 0.0)),
        'candidate_budget_left_at_start_us': float(getattr(result, 'budget_left_at_start_us', 0.0)),
        'current_stop_proof_us': float(getattr(result, 'current_stop_proof_us', 0.0)),
        'next_stop_proof_us': float(getattr(result, 'next_stop_proof_us', 0.0)),
        'failed_candidates': int(getattr(result, 'stopped_proof_attempts', 0)),
        'voluntary_context_switches': int(getattr(result, 'voluntary_context_switches', 0)),
        'involuntary_context_switches': int(getattr(result, 'involuntary_context_switches', 0)),
    }
    if result.proved:
        same = (result.steering_rad == decision.steering_rad and
                result.acceleration_mps2 == decision.acceleration_mps2)
        if same:
            return FinalPreflightOutcome(decision, result.reason, False, False, trace)
        # State/receipt must not continue to claim Primary commanded the
        # moderated acceleration. Subsequent callback code must not mutate
        # the proved command before publication.
        moderated = replace(
            decision,
            steering_rad=result.steering_rad,
            acceleration_mps2=result.acceleration_mps2,
            mode='FALLBACK_FINAL_PREFLIGHT_MODERATED',
            reason='FINAL_PREFLIGHT_MODERATED_FROM:'+original_mode+':'+result.reason,
            degradation_active=True,
        )
        return FinalPreflightOutcome(moderated, result.reason, True, False, trace)

    # No certificate means NO uncertified command (including braking).
    # The controller's degraded brake is only a fail-closed proposal; the
    # independent Guard may still reject its path and stop the car.
    braking = controller.degraded(previous_steering,
                                  'STOP_PREFLIGHT_UNPROVEN:'+result.reason,
                                  actual_speed_mps, dt_s)
    braking = replace(braking, mode='FALLBACK_STOP_PREFLIGHT_UNPROVEN',
                      target_speed_mps=0.0)
    if not (math.isfinite(float(braking.acceleration_mps2)) and
            braking.acceleration_mps2 <= 0.0):
        raise RuntimeError('DEGRADED_CONTROLLER_PRODUCED_UNSAFE_ACCELERATION')
    return FinalPreflightOutcome(braking, result.reason, True, True, trace)
