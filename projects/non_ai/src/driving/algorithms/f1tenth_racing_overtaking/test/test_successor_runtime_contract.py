from dataclasses import replace
from types import SimpleNamespace

import pytest

from f1tenth_racing_overtaking.successor_node import (
    runtime_deadlines,
    split_time,
)


def plan(committed_at_s=10.0):
    return SimpleNamespace(committed_at_s=committed_at_s)


def test_normal_committed_deadline_never_exceeds_fixed_continuation():
    valid, continuation = runtime_deadlines(
        plan(),
        "PUBLISH_OVERRIDE",
        now_s=24.9,
        plan_validity_s=0.5,
        continuation_timeout_s=15.0,
        planning_period_s=0.2,
    )
    assert continuation == pytest.approx(25.0)
    assert valid == pytest.approx(25.0)


def test_expired_committed_plan_is_not_publishable():
    assert runtime_deadlines(
        plan(),
        "PUBLISH_OVERRIDE",
        now_s=25.0,
        plan_validity_s=0.5,
        continuation_timeout_s=15.0,
        planning_period_s=0.2,
    ) is None


def test_emergency_abort_has_exactly_one_planning_period_lease():
    valid, continuation = runtime_deadlines(
        plan(),
        "PUBLISH_ABORT_DEGRADE",
        now_s=12.0,
        plan_validity_s=0.5,
        continuation_timeout_s=15.0,
        planning_period_s=0.2,
    )
    assert valid == pytest.approx(12.2)
    assert continuation == pytest.approx(12.2)


def test_split_time_normalizes_rounding():
    assert split_time(12.25) == (12, 250_000_000)
    assert split_time(1.9999999996) == (2, 0)

def test_pass_wire_lease_is_capped_by_pass_timeout():
    timed = SimpleNamespace(
        committed_at_s=10.0,
        state="COMMITTED_PASS",
    )
    valid, continuation = runtime_deadlines(
        timed,
        "PUBLISH_OVERRIDE",
        now_s=10.4,
        plan_validity_s=0.5,
        continuation_timeout_s=15.0,
        planning_period_s=0.2,
        pass_timeout_s=1.0,
    )
    assert continuation == pytest.approx(11.0)
    assert valid == pytest.approx(10.9)


def test_pass_wire_lease_is_not_publishable_after_pass_timeout():
    timed = SimpleNamespace(
        committed_at_s=10.0,
        state="PASSING",
    )
    assert runtime_deadlines(
        timed,
        "PUBLISH_OVERRIDE",
        now_s=11.0,
        plan_validity_s=0.5,
        continuation_timeout_s=15.0,
        planning_period_s=0.2,
        pass_timeout_s=1.0,
    ) is None


def test_merge_keeps_general_continuation_authority():
    timed = SimpleNamespace(
        committed_at_s=10.0,
        state="MERGE",
    )
    valid, continuation = runtime_deadlines(
        timed,
        "PUBLISH_OVERRIDE",
        now_s=10.4,
        plan_validity_s=0.5,
        continuation_timeout_s=15.0,
        planning_period_s=0.2,
        pass_timeout_s=1.0,
    )
    assert continuation == pytest.approx(25.0)
    assert valid == pytest.approx(10.9)

