"""Ego actuation-source/lease/proof binding; no ROS import or DDS-security claim."""
import math

def ros_ns(stamp):
    return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)

def verify_guarded_actuation(command, *, now_ros_ns, physics_sequence,
                              prior_intent_sequence, run_epoch, track_id,
                              map_sha256, producer, actual_speed_mps,
                              steer_limit_rad, accel_limit_mps2, brake_limit_mps2):
    """Fail-closed semantic validation before the simulator command lease refresh.

    This binds provenance in ONE ROS message; no cross-topic join/ordering.
    It does not independently verify the Guard's stopping certificate,
    authenticate DDS identities, or guarantee delivery/OS deadlines.
    """
    try:
        stamp=ros_ns(command.stamp)
        expiry=ros_ns(command.expires_ros_stamp)
        now=int(now_ros_ns)
        seq=int(command.intent_sequence)
        src_seq=int(command.source_state_sequence)
        steer=float(command.steering_rad)
        acceleration=float(command.acceleration_mps2)
        speed=float(actual_speed_mps)
        if (not run_epoch or not producer or not track_id or not map_sha256 or
                command.run_epoch!=run_epoch or command.source_id!=producer or
                command.track_id!=track_id or command.map_sha256!=map_sha256):
            return False,'GUARD_ATOMIC_IDENTITY_MISMATCH'
        if not all(math.isfinite(float(v)) for v in
                (steer,acceleration,speed,steer_limit_rad,accel_limit_mps2,brake_limit_mps2)):
            return False,'GUARD_NONFINITE'
        if (min(steer_limit_rad,accel_limit_mps2,brake_limit_mps2)<=0 or
                not (0<stamp<=now<=expiry) or now-stamp>100_000_000 or
                expiry-stamp>100_000_000 or src_seq>int(physics_sequence) or
                int(physics_sequence)-src_seq>4):
            return False,'GUARD_LEASE_OR_STATE_EXPIRED'
        if abs(steer)>steer_limit_rad or not -brake_limit_mps2<=acceleration<=accel_limit_mps2:
            return False,'GUARD_ACTUATOR_LIMIT'
        if command.authority_mode=='CERTIFIED':
            if (not command.stop_certificate_verified or seq<=int(prior_intent_sequence) or
                    seq<=0 or not command.active_reference_id):
                return False,'GUARD_PROOF_OR_REPLAY'
            return True,'CERTIFIED'
        if command.authority_mode=='MINIMUM_RISK_BRAKE':
            # This is an UNQUALIFIED minimum-risk request, never a stop proof.
            # The Guard decided a braking direction using its last CarState,
            # but the simulator may have crossed zero by delivery. Checking
            # this *stale* direction against current speed used to reject
            # emergency messages without invoking deadman immediately.
            # Require a strictly neutral-steer, bounded, discrete intent,
            # but derive the actual braking sign from *local* physics at
            # every integration tick (minimum_risk_brake_at_tick).
            # This cannot authorize CERTIFIED, propulsion or any stale lease.
            if (command.stop_certificate_verified or abs(steer)>1e-9 or
                    not any(abs(acceleration - x)<=1e-8
                            for x in (-2.0, 0.0, 2.0))):
                return False,'GUARD_INVALID_EMERGENCY_BRAKE'
            return True,'MINIMUM_RISK_BRAKE'
        return False,'GUARD_UNKNOWN_AUTHORITY_MODE'
    except (TypeError,ValueError,OverflowError,AttributeError):
        return False,'GUARD_MALFORMED_COMMAND'


def minimum_risk_brake_at_tick(actual_speed_mps, deceleration_limit_mps2,
                               physics_dt_s, *, requested_brake_mps2=2.0):
    """Simulator-only emergency actuation, recomputed under physics lock.

    The Guard's older velocity sign is never used for actuation. Magnitude is
    bounded by the original 2.0 m/s^2 request, model limit, and the remaining
    distance to zero velocity in *this* step. It cannot accelerate, reverse,
    or elevate a minimum-risk request into a certified stopping corridor.
    """
    values=(actual_speed_mps,deceleration_limit_mps2,physics_dt_s,
            requested_brake_mps2)
    if not all(math.isfinite(float(x)) for x in values):
        raise ValueError('NONFINITE_EMERGENCY_BRAKE_INPUT')
    if deceleration_limit_mps2<=0 or physics_dt_s<=0 or requested_brake_mps2<=0:
        raise ValueError('INVALID_EMERGENCY_BRAKE_LIMIT')
    speed=float(actual_speed_mps)
    if speed==0.0:
        return 0.0
    magnitude=min(float(requested_brake_mps2),float(deceleration_limit_mps2),
                  abs(speed)/float(physics_dt_s))
    return -math.copysign(magnitude,speed)
