from dataclasses import dataclass

from f1tenth_racing_control.dynamic_admission import PostBootstrapDynamicAdmission


@dataclass(frozen=True)
class Decision:
    steering_rad: float = 0.2
    acceleration_mps2: float = 4.0
    mode: str = "PRIMARY_VALID"
    reason: str = "PROBE"
    target_speed_mps: float = 12.0
    degradation_active: bool = False


class Entry:
    trend_margin_ceiling = 0.6
    time_to_zero_threshold_s = 0.15
    trend_window_s = 0.10


class Corridor:
    def __init__(self, value=1.0):
        self.value = value

    def clearance(self, px, py, yaw):
        return self.value


class Stability:
    def __init__(self):
        self.entry = Entry()
        self.corridor = Corridor()
        self.slip = 0.9
        self.yaw = 0.9

    def _dynamic_margins(self, message):
        return self.slip, self.yaw


class Message:
    px = 0.0
    py = 0.0
    yaw = 0.0
    v = 4.0


def owner():
    stability = Stability()
    return (
        PostBootstrapDynamicAdmission(
            stability,
            degraded_deceleration_mps2=2.0,
            jerk_limit_mps3=100.0,
            minimum_speed_mps=1.25,
            corridor_guard_m=0.30,
        ),
        stability,
    )


def feed(o, s, values, decision=Decision()):
    result = None
    previous = 4.0
    for t, slip, yaw in values:
        s.slip = slip
        s.yaw = yaw
        result = o.update(Message(), decision, previous, t, 0.025)
        previous = result.decision.acceleration_mps2
    return result


def test_safe_flat_margin_keeps_original_command():
    o, s = owner()
    result = feed(o, s, [(0.0, .9, .9), (.1, .9, .9)])
    assert not result.active
    assert result.decision == Decision()


def test_taper_occurs_before_existing_recovery_horizon():
    o, s = owner()
    # yaw: 0.60 -> 0.40 in 0.10 s => T_zero = 0.20 s.
    result = feed(o, s, [(0.0, .9, .60), (.1, .9, .40)])
    assert result.active
    assert result.mode == "DYNAMIC_ADMISSION_TAPER"
    assert 0.0 < result.decision.acceleration_mps2 < 4.0
    assert result.time_to_zero_s < 0.30
    assert result.time_to_zero_s > 0.15


def test_brake_occurs_at_reactive_trigger_horizon():
    o, s = owner()
    # yaw: 0.60 -> 0.20 in 0.10 s => T_zero = 0.05 s.
    result = feed(o, s, [(0.0, .9, .60), (.1, .9, .20)])
    assert result.active
    assert result.mode == "DYNAMIC_ADMISSION_BRAKE"
    assert result.decision.acceleration_mps2 < 4.0
    assert result.time_to_zero_s <= 0.15


def test_existing_negative_acceleration_is_never_weakened():
    o, s = owner()
    negative = Decision(acceleration_mps2=-3.0)
    result = feed(o, s, [(0.0, .9, .60), (.1, .9, .20)], negative)
    assert not result.active
    assert result.decision.acceleration_mps2 == -3.0


def test_steering_is_never_changed():
    o, s = owner()
    result = feed(o, s, [(0.0, .9, .60), (.1, .9, .20)])
    assert result.decision.steering_rad == Decision().steering_rad


def test_corridor_reserve_brakes_primary_too():
    o, s = owner()
    s.corridor.value = 0.29
    result = feed(o, s, [(0.0, .9, .9), (.1, .9, .9)])
    assert result.active
    assert result.mode == "DYNAMIC_ADMISSION_BRAKE"
    assert result.reason == "POST_BOOTSTRAP_CORRIDOR_RESERVE"


def test_below_startup_release_speed_does_not_steal_startup_owner():
    o, s = owner()
    m = Message()
    m.v = 1.0
    s.yaw = 0.1
    o.update(m, Decision(), 4.0, 0.0, 0.025)
    result = o.update(m, Decision(), 4.0, 0.1, 0.025)
    assert not result.active
    assert result.reason == "STARTUP_OWNER_ACTIVE"


def test_reset_discards_trend_history():
    o, s = owner()
    s.yaw = 0.6
    o.update(Message(), Decision(), 4.0, 0.0, 0.025)
    o.reset()
    s.yaw = 0.2
    result = o.update(Message(), Decision(), 4.0, 0.1, 0.025)
    assert not result.active
