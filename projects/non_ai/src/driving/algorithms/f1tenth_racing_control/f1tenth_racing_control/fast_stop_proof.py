"""Pinned R-B stop proof using the *shared* simulator implementation.

The previous V5.3 adapter copied ``StoppingEnvelope.evaluate`` and pinned
obsolete simulator hashes.  Updating the simulator in the realtime surgery
made the controller exit before ROS subscriptions were created.  This class
checks a fixed, audited source identity but inherits the full, optimized
single-authority rollout.  It must not introduce a second copy of the stop
proof, nor bypass any final/independent Guard certificate.
"""
from hashlib import sha256
from pathlib import Path

from f1tenth_racing_sim.safety import stopping_envelope as _stop_source
from f1tenth_racing_sim.world import swept_collision as _sweep_source
from f1tenth_racing_sim.safety.stopping_envelope import StoppingEnvelope

# Build-time verified identities for the *current* simulator model.
PINNED_STOP_SOURCE_SHA256 = '13f6ed05446ed4438076345a0db37f819472c7ca16f38041c3d71ffbab4fb151'
PINNED_SWEEP_SOURCE_SHA256 = 'd377dd42becaeecc76b2f3caf06a2bebe69258a83282018d468a7268364f899c'


def _verified_source_hash(mod, expected, reason):
    try:
        path = Path(mod.__file__).resolve(strict=True)
        actual = sha256(path.read_bytes()).hexdigest()
    except (OSError, TypeError, ValueError) as exc:
        raise RuntimeError(reason) from exc
    if actual != expected:
        raise RuntimeError(reason)


class CertifiedFreeStoppingEnvelope(StoppingEnvelope):
    """Controller-side adapter; shared full stopping rollout is authoritative."""

    def __init__(self, *args, **kwargs):
        _verified_source_hash(_stop_source, PINNED_STOP_SOURCE_SHA256,
                              'V53_STOP_SOURCE_UNPINNED_FAIL_CLOSED')
        _verified_source_hash(_sweep_source, PINNED_SWEEP_SOURCE_SHA256,
                              'V53_SWEEP_SOURCE_UNPINNED_FAIL_CLOSED')
        super().__init__(*args, **kwargs)
