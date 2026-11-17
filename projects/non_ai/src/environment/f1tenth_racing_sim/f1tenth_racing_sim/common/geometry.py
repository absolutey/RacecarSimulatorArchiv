import math


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
