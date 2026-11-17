from f1tenth_racing_overtaking.successor_core import (
    CONTRACT,
    InternalState,
    Topology,
    WireState,
)
from f1tenth_racing_overtaking.successor_node import (
    NON_OVERRIDE_INTERNAL_STATES,
    WIRE_STATES_BY_INTERNAL,
)


def test_successor_q0_is_not_runtime_enabled():
    assert CONTRACT.runtime_enabled is False
    assert CONTRACT.schema_version == 0


def test_owner_boundaries_are_single_owner():
    assert CONTRACT.prediction_owner == "R-D"
    assert CONTRACT.active_reference_owner == "S0-R"
    assert CONTRACT.controller_owner == "R-B"
    assert CONTRACT.nominal_reference_owner == "R-A2"


def test_topology_set_is_finite_and_explicit():
    assert CONTRACT.candidate_topologies == (
        Topology.LEFT_EARLY,
        Topology.LEFT_LATE,
        Topology.RIGHT_EARLY,
        Topology.RIGHT_LATE,
        Topology.FOLLOW,
    )


def test_internal_frozen_states_do_not_expand_wire_schema():
    assert WIRE_STATES_BY_INTERNAL[InternalState.PASS_COMMIT] == frozenset({
        WireState.COMMITTED_PASS,
    })
    assert WIRE_STATES_BY_INTERNAL[InternalState.FROZEN_PASS] == frozenset({
        WireState.COMMITTED_PASS,
        WireState.PASSING,
        WireState.ABORT,
    })
    assert WIRE_STATES_BY_INTERNAL[InternalState.PASS_CONFIRMED] == frozenset({
        WireState.MERGE,
    })
    assert WIRE_STATES_BY_INTERNAL[InternalState.FROZEN_REJOIN] == frozenset({
        WireState.MERGE,
        WireState.ABORT,
    })
    assert InternalState.TRAIL in NON_OVERRIDE_INTERNAL_STATES


def test_no_literal_frozen_wire_states_exist():
    emitted = {
        state.value
        for states in WIRE_STATES_BY_INTERNAL.values()
        for state in states
    }
    assert "FROZEN_PASS" not in emitted
    assert "FROZEN_REJOIN" not in emitted
    assert "PASS_CONFIRMED" not in emitted


def test_existing_wire_states_only():
    emitted = {
        state.value
        for states in WIRE_STATES_BY_INTERNAL.values()
        for state in states
    }
    assert emitted <= {
        "COMMITTED_PASS",
        "PASSING",
        "MERGE",
        "ABORT",
        "BLOCKING_HOLD",
        "COMPLETE",
    }
