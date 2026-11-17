import math

import numpy as np


class StaticExpectedScan:
    """Exact fixed-grid scan with conservative free-space skipping."""

    def __init__(self, occupancy_map, angle_min, angle_increment, beam_count, minimum, maximum):
        self.map = occupancy_map
        self.relative_angles = angle_min + np.arange(beam_count) * angle_increment
        self.distances = np.arange(minimum, maximum, occupancy_map.resolution * 0.5)
        self.maximum = maximum
        occupied = occupancy_map.pixel_array < occupancy_map.threshold
        height, width = occupied.shape
        rows, columns = np.indices((height, width))
        distance = np.minimum.reduce((rows + 1, height - rows, columns + 1, width - columns)).astype(np.int32)
        distance[occupied] = 0
        for row in range(height):
            for column in range(width):
                if row:
                    distance[row, column] = min(distance[row, column], distance[row - 1, column] + 1)
                if column:
                    distance[row, column] = min(distance[row, column], distance[row, column - 1] + 1)
        for row in range(height - 1, -1, -1):
            for column in range(width - 1, -1, -1):
                if row + 1 < height:
                    distance[row, column] = min(distance[row, column], distance[row + 1, column] + 1)
                if column + 1 < width:
                    distance[row, column] = min(distance[row, column], distance[row, column + 1] + 1)
        self.occupied = occupied
        self.skip_samples = np.maximum(
            1, np.floor(2.0 * (distance / math.sqrt(2.0) - 2.0)).astype(np.int32),
        )

    def scan(self, x, y, yaw):
        angles = yaw + self.relative_angles
        cosines, sines = np.cos(angles), np.sin(angles)
        count = len(angles)
        indices = np.zeros(count, dtype=np.int32)
        result = np.full(count, self.maximum)
        rays = np.arange(count)
        while rays.size:
            distance = self.distances[indices[rays]]
            columns = np.floor(
                (x + cosines[rays] * distance - self.map.origin_x) / self.map.resolution
            ).astype(np.int32)
            rows_from_bottom = np.floor(
                (y + sines[rays] * distance - self.map.origin_y) / self.map.resolution
            ).astype(np.int32)
            rows = self.map.height - 1 - rows_from_bottom if self.map.flip_y else rows_from_bottom
            inside = ((columns >= 0) & (columns < self.map.width) &
                      (rows >= 0) & (rows < self.map.height))
            hit = ~inside
            hit[inside] = self.occupied[rows[inside], columns[inside]]
            hit_rays = rays[hit]
            result[hit_rays] = distance[hit]
            free_rays = rays[~hit]
            if not free_rays.size:
                break
            # L1/sqrt(2) bounds Euclidean center distance. Two cells cover both
            # endpoint-to-center diagonals, so skipped fixed samples are provably free.
            indices[free_rays] += self.skip_samples[rows[~hit], columns[~hit]]
            rays = free_rays[indices[free_rays] < len(self.distances)]
        return result.tolist()
