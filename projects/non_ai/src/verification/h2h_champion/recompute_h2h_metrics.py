#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from f1tenth_racing_geometry import ClosedTrackGeometry, load_centerline_csv


def corners(state, half=0.2):
    _, x, y, yaw = state
    c, s = math.cos(yaw), math.sin(yaw)
    return tuple((x + c * dx - s * dy, y + s * dx + c * dy)
                 for dx, dy in ((-half, -half), (half, -half), (half, half), (-half, half)))


def polygon_distance(first, second):
    def project(shape, axis):
        values = [x * axis[0] + y * axis[1] for x, y in shape]
        return min(values), max(values)

    separated = False
    for shape in (first, second):
        for index in range(2):
            a, b = shape[index], shape[index + 1]
            axis = (a[1] - b[1], b[0] - a[0])
            left, right = project(first, axis), project(second, axis)
            separated |= left[1] < right[0] or right[1] < left[0]
    if not separated:
        return 0.0

    def point_edge(point, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        scale = dx * dx + dy * dy
        ratio = max(0.0, min(1.0, ((point[0] - a[0]) * dx +
                                    (point[1] - a[1]) * dy) / scale))
        return math.hypot(point[0] - a[0] - ratio * dx,
                          point[1] - a[1] - ratio * dy)

    return min(point_edge(point, shape[index], shape[(index + 1) % 4])
               for points, shape in ((first, second), (second, first))
               for point in points for index in range(4))


def lap_crossing_times(states, points):
    crossings, previous = [], None
    for state in states:
        index = min(range(len(points)), key=lambda i: (points[i][0] - state[1]) ** 2 +
                    (points[i][1] - state[2]) ** 2)
        if previous is not None and previous > 0.75 * len(points) and index < 0.25 * len(points):
            crossings.append(state[0])
        previous = index
    return crossings


def lap_times_from_crossings(crossings, start):
    return ([crossings[0] - start] if start is not None and crossings else []) + [
        b - a for a, b in zip(crossings, crossings[1:])
    ]


def recompute(raw):
    sim = Path(get_package_share_directory("f1tenth_racing_sim"))
    centerline = sim / "maps/iccas2025/iccas2025_centerline.csv"
    track = ClosedTrackGeometry(load_centerline_csv(centerline))
    with centerline.open(newline="") as stream:
        points = [(float(row["x_m"]), float(row["y_m"])) for row in csv.DictReader(stream)]
    blue, red = raw["odom0"], raw["odom1"]
    aligned = []
    for item in blue:
        target = min(red, key=lambda other: abs(other[0] - item[0]))
        if abs(target[0] - item[0]) <= 0.04:
            aligned.append((item, target))
    center = min(math.hypot(a[1] - b[1], a[2] - b[2]) for a, b in aligned)
    clearance = min(polygon_distance(corners(a), corners(b)) for a, b in aligned)
    passed, was_behind = 0, False
    for a, b in aligned:
        delta = track.signed_delta_s(track.project(b[1], b[2]).s, track.project(a[1], a[2]).s)
        if delta < -1.0:
            was_behind = True
        elif was_behind and delta > 1.0:
            passed += 1
            was_behind = False
    crossings = lap_crossing_times(blue, points)
    start = raw.get("race_start_time_s")
    lap_times = lap_times_from_crossings(crossings, start)
    return {
        "aligned_pairs": len(aligned),
        "minimum_center_separation_m": center,
        "minimum_footprint_clearance_m": clearance,
        "physical_pass_count": passed,
        "crossing_times_s": crossings,
        "lap_times_s": lap_times,
        "flying_lap_times_s": lap_times[1:],
        "sampled_car_collision": clearance <= 0.0,
    }


def self_test():
    square = ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
    assert polygon_distance(square, tuple((x + 3.0, y + 4.0) for x, y in square)) == math.sqrt(5.0)
    assert polygon_distance(square, square) == 0.0
    points = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (-1.0, 1.0)]
    states = [(1.0, 0.0, 1.0, 0.0), (2.0, -1.0, 1.0, 0.0),
              (3.0, 0.0, 0.0, 0.0)]
    assert lap_crossing_times(states, points) == [3.0]
    assert lap_times_from_crossings([3.0, 7.0], 1.0) == [2.0, 4.0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("raw", nargs="?", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.raw is None:
        parser.error("raw_runtime.json is required")
    result = recompute(json.loads(args.raw.read_text()))
    output = args.raw.with_name("independent_metrics.json")
    output.write_text(json.dumps(result, sort_keys=True, indent=2) + "\n")
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
