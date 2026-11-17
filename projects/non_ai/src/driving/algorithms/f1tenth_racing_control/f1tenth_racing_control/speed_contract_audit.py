"""Non-authoritative, conservative speed/horizon audit (NEVER an approval)."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class HorizonFinding:
    speed_mps: float
    nominal_braking_seconds: float
    nominal_distance_m: float
    proof_horizon_s: float
    within_nominal_horizon: bool


def audit_nominal_stop_horizon(speed_mps, brake_mps2=2.0, reaction_s=0.25, proof_horizon_s=8.0):
    """A planning diagnostic; yaw/slip/obstacles/jerk require full Guard proof.

    Returns no positive safety certificate and must never be used by Guard to
    admit a command. A nominal calculation cannot qualify a high-speed cap.
    """
    v=float(speed_mps); b=float(brake_mps2); r=float(reaction_s); h=float(proof_horizon_s)
    if not all(math.isfinite(x) for x in (v,b,r,h)) or v < 0 or b<=0 or r<0 or h<=0:
        raise ValueError('INVALID_NOMINAL_HORIZON_PARAMETERS')
    return HorizonFinding(v, v/b, v*r + v*v/(2*b),h,r+v/b <= h)
