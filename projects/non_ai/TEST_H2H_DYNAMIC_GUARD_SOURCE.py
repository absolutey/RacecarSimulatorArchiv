#!/usr/bin/env python3
"""Offline safety-contract inspection for the current dynamic-Red Guard.

Tests MUST NOT restore the retired unconditional H2H rejection marker just
for string matching. This checker verifies the actual fail-closed branches.
It is NOT runtime collision or timing certification.
"""

def assert_h2h_dynamic_guard_source(source):
    required = (
        "if not self.solo and not self.red_guard_enabled:",
        "H2H_REQUIRES_INDEPENDENT_DYNAMIC_RED_GUARD",
        "if not self.solo:",
        "H2H_RED_PREDICTION_MISSING_OR_STALE",
        "RedDynamicWitness(self.red_prediction",
        "witness.collision_free(z,z1,0.0,0.025)",
        "return witness.collision_free(before,after,",
        "H2H_RED_IMMEDIATE_SWEEP_REJECTED",
        "H2H_RED_WITNESS_UNPROVEN",
        "if dynamic_checker is not None:",
        "return False  # no legacy unbounded/non-dynamic fallback in H2H",
        "if elapsed_ns>15000000:",
        "GUARD_15MS_RESPONSE_DEADLINE",
    )
    missing=[marker for marker in required if marker not in source]
    if missing:
        raise AssertionError('H2H_GUARD_FAIL_CLOSED_CONTRACT_MISSING:'+repr(missing))
