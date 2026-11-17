import bisect
import csv
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TrackSample:
    x: float
    y: float
    heading: float
    s: float
    segment_index: int


@dataclass(frozen=True)
class Projection:
    x: float
    y: float
    s: float
    d: float
    heading: float
    distance: float
    segment_index: int
    valid: bool = True


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def wrap_angle(angle):
    angle = _finite(angle, "angle")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def load_centerline_csv(path, x_field="x_m", y_field="y_m"):
    """Load centerline map coordinates without owning or copying the asset."""
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    try:
        return [(float(row[x_field]), float(row[y_field])) for row in rows]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid centerline CSV fields {x_field!r}, {y_field!r}") from error


class ClosedTrackGeometry:
    """Immutable piecewise-linear closed-track geometry in map-frame metres."""

    def __init__(self, points):
        canonical = tuple((_finite(x, "x"), _finite(y, "y")) for x, y in points)
        if len(canonical) >= 2 and canonical[0] == canonical[-1]:
            canonical = canonical[:-1]
        if len(canonical) < 3:
            raise ValueError("closed track requires at least three usable points")

        segments = []
        cumulative = [0.0]
        for index, start in enumerate(canonical):
            end = canonical[(index + 1) % len(canonical)]
            dx, dy = end[0] - start[0], end[1] - start[1]
            length = math.hypot(dx, dy)
            if length == 0.0:
                raise ValueError(f"zero-length segment at index {index}")
            tx, ty = dx / length, dy / length
            segments.append((start[0], start[1], dx, dy, length, tx, ty, math.atan2(dy, dx)))
            cumulative.append(cumulative[-1] + length)

        self._points = canonical
        self._segments = tuple(segments)
        self._cumulative = tuple(cumulative)
        self._length = cumulative[-1]

    @property
    def points(self):
        return self._points

    @property
    def segment_count(self):
        return len(self._segments)

    @property
    def length(self):
        return self._length

    def wrap_s(self, s):
        return _finite(s, "s") % self._length

    def forward_delta_s(self, start_s, end_s):
        return (self.wrap_s(end_s) - self.wrap_s(start_s)) % self._length

    def signed_delta_s(self, start_s, end_s):
        return (self.forward_delta_s(start_s, end_s) + 0.5 * self._length) % self._length - 0.5 * self._length

    def _segment_at(self, s):
        wrapped = self.wrap_s(s)
        index = bisect.bisect_right(self._cumulative, wrapped) - 1
        index = min(index, self.segment_count - 1)
        return wrapped, index, wrapped - self._cumulative[index]

    def interpolate(self, s):
        wrapped, index, along = self._segment_at(s)
        x, y, _, _, _, tx, ty, heading = self._segments[index]
        return TrackSample(x + along * tx, y + along * ty, heading, wrapped, index)

    def from_frenet(self, s, d):
        d = _finite(d, "d")
        sample = self.interpolate(s)
        return TrackSample(
            sample.x - d * math.sin(sample.heading),
            sample.y + d * math.cos(sample.heading),
            sample.heading,
            sample.s,
            sample.segment_index,
        )

    def project(self, x, y):
        x, y = _finite(x, "x"), _finite(y, "y")
        best = None
        for index, (x0, y0, dx, dy, length, tx, ty, heading) in enumerate(self._segments):
            fraction = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / (length * length)))
            px, py = x0 + fraction * dx, y0 + fraction * dy
            ex, ey = x - px, y - py
            distance_sq = ex * ex + ey * ey
            candidate = (distance_sq, index, px, py, ex, ey, length, fraction, tx, ty, heading)
            if best is None or candidate[:2] < best[:2]:
                best = candidate
        distance_sq, index, px, py, ex, ey, length, fraction, tx, ty, heading = best
        s = self.wrap_s(self._cumulative[index] + fraction * length)
        d = tx * ey - ty * ex
        return Projection(px, py, s, d, heading, math.sqrt(distance_sq), index)


def _periodic_interval(sample_s, query_s, track_length):
    if track_length <= 0.0 or not math.isfinite(track_length):
        raise ValueError("track_length must be finite and positive")
    positions = tuple(_finite(s, "sample_s") for s in sample_s)
    if len(positions) < 2 or any(not 0.0 <= s < track_length for s in positions):
        raise ValueError("sample_s must contain at least two values in [0, track_length)")
    if any(a >= b for a, b in zip(positions, positions[1:])):
        raise ValueError("sample_s must be strictly increasing")
    query = _finite(query_s, "query_s") % track_length
    upper = bisect.bisect_right(positions, query)
    if upper == 0 or upper == len(positions):
        low, high = len(positions) - 1, 0
        span = positions[0] + track_length - positions[-1]
        offset = (query - positions[-1]) % track_length
    else:
        low, high = upper - 1, upper
        span = positions[high] - positions[low]
        offset = query - positions[low]
    return low, high, offset / span


def interpolate_periodic_scalar(sample_s, values, query_s, track_length):
    sample_s = tuple(sample_s)
    values = tuple(_finite(value, "value") for value in values)
    if len(values) != len(sample_s):
        raise ValueError("sample_s and values lengths differ")
    low, high, ratio = _periodic_interval(sample_s, query_s, track_length)
    return values[low] + ratio * (values[high] - values[low])


def interpolate_periodic_vector(sample_s, values, query_s, track_length):
    sample_s = tuple(sample_s)
    values = tuple(tuple(_finite(value, "value") for value in vector) for vector in values)
    if not values or len(values) != len(sample_s) or any(len(vector) != len(values[0]) for vector in values):
        raise ValueError("sample_s and equal-width vector values are required")
    low, high, ratio = _periodic_interval(sample_s, query_s, track_length)
    return tuple(a + ratio * (b - a) for a, b in zip(values[low], values[high]))


def interpolate_periodic_angle(sample_s, angles, query_s, track_length):
    sample_s = tuple(sample_s)
    angles = tuple(_finite(angle, "angle") for angle in angles)
    if len(angles) != len(sample_s):
        raise ValueError("sample_s and angles lengths differ")
    low, high, ratio = _periodic_interval(sample_s, query_s, track_length)
    return wrap_angle(angles[low] + ratio * wrap_angle(angles[high] - angles[low]))
