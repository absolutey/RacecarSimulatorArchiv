import csv
import math


def load_centerline(path):
    with open(path, newline="") as stream:
        return list(csv.DictReader(stream))


def pose_at_s(centerline, distance):
    points = [(float(row["x_m"]), float(row["y_m"])) for row in centerline]
    lengths = [
        math.hypot(
            points[(i + 1) % len(points)][0] - point[0],
            points[(i + 1) % len(points)][1] - point[1],
        )
        for i, point in enumerate(points)
    ]
    remaining = distance % sum(lengths)
    for i, length in enumerate(lengths):
        if remaining <= length:
            x0, y0 = points[i]
            x1, y1 = points[(i + 1) % len(points)]
            ratio = remaining / length
            return (
                x0 + ratio * (x1 - x0),
                y0 + ratio * (y1 - y0),
                math.atan2(y1 - y0, x1 - x0),
            )
        remaining -= length
    raise AssertionError("closed centerline distance resolution failed")
