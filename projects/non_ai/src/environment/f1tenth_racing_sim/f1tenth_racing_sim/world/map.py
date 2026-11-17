import math
from pathlib import Path

import numpy as np


class OccupancyMap:
    def __init__(self, pgm_path, config):
        self.resolution = config["resolution_m_per_pixel"]
        self.origin_x = config["origin"]["x_m"]
        self.origin_y = config["origin"]["y_m"]
        self.origin_yaw = float(config["origin"].get("yaw_rad", 0.0))
        if not math.isfinite(self.origin_yaw):
            raise ValueError("non-finite map origin yaw")
        self.origin_cos = math.cos(self.origin_yaw)
        self.origin_sin = math.sin(self.origin_yaw)
        self.threshold = config["runtime_pixel_occupied_below"]
        self.flip_y = config["runtime_flip_y"]
        self.width, self.height, self.pixels = self._read_pgm(Path(pgm_path))
        self.pixel_array = np.frombuffer(self.pixels, dtype=np.uint8).reshape(
            self.height, self.width
        )
        # Immutable PGM-backed occupancy. A single prefix sum gives the
        # exact same occupied-cell predicate in O(1) for any rectangle.
        # The extra zero border makes inclusive queries unambiguous.
        occupied = (self.pixel_array < self.threshold).astype(np.uint64)
        self._occupied_integral = np.zeros(
            (self.height + 1, self.width + 1), dtype=np.uint64
        )
        self._occupied_integral[1:, 1:] = occupied.cumsum(
            axis=0, dtype=np.uint64
        ).cumsum(axis=1, dtype=np.uint64)
        self._integral_threshold = self.threshold

    def rectangle_is_free(self, row0, row1, col0, col1):
        """True only if an in-map inclusive pixel rectangle has ZERO obstacles.

        The original threshold and immutable source PGM bytes are authoritative.
        A changed threshold is not silently trusted; use the raw predicate.
        """
        if (row0 < 0 or col0 < 0 or row1 >= self.height or
                col1 >= self.width or row0 > row1 or col0 > col1):
            return False
        if self.threshold != self._integral_threshold:
            return bool((self.pixel_array[row0:row1+1, col0:col1+1]
                         >= self.threshold).all())
        ii = self._occupied_integral
        count = (int(ii[row1+1, col1+1]) - int(ii[row0, col1+1])
                 - int(ii[row1+1, col0]) + int(ii[row0, col0]))
        return count == 0

    @staticmethod
    def _read_pgm(path):
        with path.open("rb") as stream:
            assert stream.readline().strip() == b"P5"
            line = stream.readline()
            while line.startswith(b"#"):
                line = stream.readline()
            width, height = map(int, line.split())
            assert int(stream.readline()) == 255
            pixels = stream.read()
        assert len(pixels) == width * height
        return width, height, pixels

    def occupied(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        col = math.floor((c * dx + s * dy) / self.resolution)
        row_from_bottom = math.floor((-s * dx + c * dy) / self.resolution)
        row = self.height - 1 - row_from_bottom if self.flip_y else row_from_bottom
        if not (0 <= col < self.width and 0 <= row < self.height):
            return True
        return self.pixels[row * self.width + col] < self.threshold
