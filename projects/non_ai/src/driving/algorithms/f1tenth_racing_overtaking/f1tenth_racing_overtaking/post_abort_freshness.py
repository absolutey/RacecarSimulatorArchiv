"""Post-ABORT Red epoch fence, separate from the existing Red collision proof.

A planner may resume only after S0-R's three-tick parked recovery and a NEW
post-recovery Red measurement. A replayed pre-ABORT prediction cannot seed a
second pass even if its old confidence and horizon still look adequate.
"""
import math


def accept_post_abort_red_observation(source_ns, release_ns, now_ns):
    if release_ns is None:
        return True
    if any(type(x) is not int for x in (source_ns,release_ns,now_ns)):
        return False
    return (release_ns >= 0 and 0 < source_ns-release_ns and
            0 <= now_ns-source_ns <= 350_000_000)
