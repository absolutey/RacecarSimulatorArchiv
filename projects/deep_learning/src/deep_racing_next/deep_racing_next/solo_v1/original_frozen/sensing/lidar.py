import numpy as np


def scan_occupancy(occupancy_map, x, y, angles, minimum, maximum):
    distances = np.arange(minimum, maximum, occupancy_map.resolution * 0.5)
    world_x = x + np.cos(angles)[:, None] * distances
    world_y = y + np.sin(angles)[:, None] * distances
    columns = np.floor(
        (world_x - occupancy_map.origin_x) / occupancy_map.resolution
    ).astype(np.int32)
    rows_from_bottom = np.floor(
        (world_y - occupancy_map.origin_y) / occupancy_map.resolution
    ).astype(np.int32)
    rows = (
        occupancy_map.height - 1 - rows_from_bottom
        if occupancy_map.flip_y
        else rows_from_bottom
    )
    inside = (
        (columns >= 0)
        & (columns < occupancy_map.width)
        & (rows >= 0)
        & (rows < occupancy_map.height)
    )
    occupied = ~inside
    valid_rows = np.clip(rows, 0, occupancy_map.height - 1)
    valid_columns = np.clip(columns, 0, occupancy_map.width - 1)
    occupied |= (
        occupancy_map.pixel_array[valid_rows, valid_columns]
        < occupancy_map.threshold
    )
    first = occupied.argmax(axis=1)
    hit = occupied.any(axis=1)
    return np.where(hit, distances[first], maximum).astype(float).tolist()
