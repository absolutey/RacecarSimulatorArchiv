"""Reference execution identity policy, kept independent of ROS imports."""
import math


def authorize_active_reference(message, *, requested_cap_mps,
                               qualified_trajectory_id, expected_track_id,
                               expected_map_sha256, trusted_high_speed,
                               simulation_online_bound_mps=None):
    try: cap=float(requested_cap_mps)
    except (TypeError,ValueError) as exc:raise ValueError("nonfinite requested cap") from exc
    if not math.isfinite(cap) or cap<=0:
        raise ValueError("nonfinite requested cap")
    if message.source_kind not in ("NOMINAL", "OVERTAKE_OVERRIDE"):
        raise ValueError("unknown reference source")
    if message.track_id != expected_track_id or message.map_image_sha256 != expected_map_sha256:
        raise ValueError("reference map/track mismatch")
    if cap>5.8:
        if trusted_high_speed and message.source_kind == "NOMINAL":
            if not qualified_trajectory_id or message.source_trajectory_id != qualified_trajectory_id:
                raise ValueError("actual nominal trajectory differs from qualified trajectory")
        else:
            # Overrides may be proposed within SIM physics bounds, but are
            # never promoted by this check. Guard and Red occupancy remain
            # mandatory independent actuation authorities.
            # Explicit simulation-only online certificate route; never claim
            # independent signed high-speed qualification from this branch.
            limit = simulation_online_bound_mps
            if (limit is None or not math.isfinite(float(limit)) or
                    cap>float(limit) or not message.source_trajectory_id):
                raise ValueError("simulation online speed bound not qualified")
            samples = getattr(message,"samples",())
            if len(samples)<3 or message.sample_count!=len(samples):
                raise ValueError("simulation reference speed samples invalid")
            if any(not math.isfinite(float(s.target_speed_mps)) or
                   not 0<=float(s.target_speed_mps)<=float(limit) for s in samples):
                raise ValueError("simulation speed plan exceeds vehicle physics")
    return True
