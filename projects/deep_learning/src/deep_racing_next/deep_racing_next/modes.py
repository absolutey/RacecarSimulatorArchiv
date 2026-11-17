"""Solo first; overtake is a separate capability gate."""
from dataclasses import dataclass
from enum import Enum
from .contracts import ContractError

class Mode(Enum):
    SOLO_TRAIN='solo_train'
    SOLO_EVAL='solo_eval'
    OVERTAKE='overtake'

@dataclass(frozen=True)
class ModeRequest:
    mode:Mode
    solo_certified:bool=False
    opponent_stack_certified:bool=False
    def assert_admissible(self):
        if self.mode==Mode.OVERTAKE and (not self.solo_certified or not self.opponent_stack_certified):
            raise ContractError('OVERTAKE_NOT_CERTIFIED')
