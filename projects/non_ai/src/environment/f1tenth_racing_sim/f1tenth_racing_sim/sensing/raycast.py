import math

import numpy as np


def ray_box_ranges(origin_x, origin_y, angles, box_state, footprint, maximum):
    dx = np.cos(angles)
    dy = np.sin(angles)
    cos_yaw = math.cos(box_state.yaw)
    sin_yaw = math.sin(box_state.yaw)
    relative_x = origin_x - box_state.x
    relative_y = origin_y - box_state.y
    local_origin_x = relative_x * cos_yaw + relative_y * sin_yaw
    local_origin_y = -relative_x * sin_yaw + relative_y * cos_yaw
    local_dx = dx * cos_yaw + dy * sin_yaw
    local_dy = -dx * sin_yaw + dy * cos_yaw
    with np.errstate(divide="ignore", invalid="ignore"):
        tx1 = (footprint["min_x_m"] - local_origin_x) / local_dx
        tx2 = (footprint["max_x_m"] - local_origin_x) / local_dx
        ty1 = (footprint["min_y_m"] - local_origin_y) / local_dy
        ty2 = (footprint["max_y_m"] - local_origin_y) / local_dy
    near = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
    far = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
    hit = (
        (far >= np.maximum(near, 0.0))
        & (near >= 0.0)
        & (near <= maximum)
    )
    return np.where(hit, near, maximum)
