"""Fail-closed boundary; never creates vehicle commands."""
from __future__ import annotations
from dataclasses import dataclass
from .contracts import DriverProposal, VehicleState, MAX_SPEED_MPS

@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    reason: str

class IndependentSafetyBoundary:
    def __init__(self):
        self.armed=False  # No code path can arm without future verified integration
    def evaluate(self, state: VehicleState, proposal: DriverProposal | None,
                 *, dynamic_model_certified:bool=False, guard_receipt:bool=False)->SafetyDecision:
        if proposal is None:
            return SafetyDecision(False,'NO_PROPOSAL')
        if not dynamic_model_certified:
            return SafetyDecision(False,'ORIGINAL_DYNAMICS_NOT_CERTIFIED')
        if state.speed_mps > MAX_SPEED_MPS:
            return SafetyDecision(False,'SPEED_LIMIT_EXCEEDED')
        if not guard_receipt:
            return SafetyDecision(False,'INDEPENDENT_GUARD_MISSING')
        if not self.armed:
            return SafetyDecision(False,'NOT_ARMED_RESEARCH_ONLY')
        return SafetyDecision(False,'NO_ACTUATOR_OWNERSHIP')
