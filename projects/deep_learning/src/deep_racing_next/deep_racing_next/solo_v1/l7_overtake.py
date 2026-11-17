"""L7: opponent relative state / tactical candidate. SOLO never publishes or arms."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Opponent:
    range_m:float
    rel_speed_mps:float
    lateral_m:float
    confidence:float
    def __post_init__(self):
        if not all(math.isfinite(x) for x in (self.range_m,self.rel_speed_mps,self.lateral_m,self.confidence)) or self.range_m<0 or not 0<=self.confidence<=1:
            raise ValueError('INVALID_OPPONENT')

def suggest_overtake(opponent:Opponent|None,*,solo_certified=False,tracking_certified=False,prediction_certified=False):
    if not all((solo_certified,tracking_certified,prediction_certified)):
        return {'admissible':False,'reason':'OVERTAKE_MODE_DISABLED_UNTIL_ALL_GATES_PASS','proposal':None}
    if opponent is None or opponent.confidence<.95:
        return {'admissible':False,'reason':'NO_QUALIFIED_OPPONENT','proposal':None}
    # Still requires dynamic collision prediction and independent guard before control; no actuation.
    return {'admissible':False,'reason':'TACTICAL_COLLISION_CHECK_NOT_IMPLEMENTED','proposal':None}
