import math


def lateral_force(tire, slip_angle):
    return tire["pacejka_d_n"] * math.sin(
        tire["pacejka_c"] * math.atan(tire["pacejka_b"] * slip_angle)
    )
