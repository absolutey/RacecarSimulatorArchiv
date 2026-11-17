"""Versioned, implementation-independent contracts: planning != actuation."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple
import math

MAX_SPEED_MPS = 20.0
SCHEMA_VERSION = 'dr-v2.1'

class ContractError(ValueError):
    pass

def finite(*values: float) -> None:
    if not all(math.isfinite(x) for x in values):
        raise ContractError('NONFINITE_INPUT')

@dataclass(frozen=True)
class VehicleState:
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float
    stamp_s: float
    def __post_init__(self):
        finite(self.x_m, self.y_m, self.yaw_rad, self.speed_mps, self.stamp_s)
        if self.speed_mps < 0 or self.speed_mps > MAX_SPEED_MPS:
            raise ContractError('SPEED_OUT_OF_BOUNDS')

@dataclass(frozen=True)
class MapIdentity:
    name: str
    image_sha256: str
    yaml_sha256: str
    resolution_m: float
    origin_xyyaw: Tuple[float, float, float]
    def __post_init__(self):
        if not self.name or len(self.image_sha256) != 64 or len(self.yaml_sha256) != 64:
            raise ContractError('MAP_IDENTITY_INVALID')
        finite(self.resolution_m, *self.origin_xyyaw)
        if self.resolution_m <= 0:
            raise ContractError('MAP_RESOLUTION_INVALID')

@dataclass(frozen=True)
class PlannerTarget:
    lateral_offset_m: float
    speed_mps: float
    target_curvature_inv_m: float
    def __post_init__(self):
        finite(self.lateral_offset_m, self.speed_mps, self.target_curvature_inv_m)
        if not (0.0 <= self.speed_mps <= MAX_SPEED_MPS):
            raise ContractError('TARGET_SPEED_OUT_OF_BOUNDS')

@dataclass(frozen=True)
class DriverProposal:
    steering_rad: float
    acceleration_mps2: float
    stamp_s: float
    policy_sha256: str
    def __post_init__(self):
        finite(self.steering_rad, self.acceleration_mps2, self.stamp_s)
        if len(self.policy_sha256) != 64:
            raise ContractError('POLICY_IDENTITY_MISSING')

@dataclass(frozen=True)
class EpisodeContract:
    map_hash: str
    start_x_m: float
    start_y_m: float
    start_yaw_rad: float
    initial_speed_mps: float = 0.0
    def __post_init__(self):
        if len(self.map_hash) != 64:
            raise ContractError('MAP_HASH_REQUIRED')
        finite(self.start_x_m, self.start_y_m, self.start_yaw_rad, self.initial_speed_mps)
        if self.initial_speed_mps != 0.0:
            raise ContractError('EPISODE_MUST_START_AT_REST')
