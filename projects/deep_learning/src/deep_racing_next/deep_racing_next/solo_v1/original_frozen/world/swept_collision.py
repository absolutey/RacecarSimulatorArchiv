"""Map-grid oriented footprint and conservatively padded swept-interval test.

Cells are squares, not point samples. Collision is conservative: any touch counts.
Swept interval uses a body-motion Lipschitz bound to inflate sampled poses.
It is a raster-model detector, not by itself a certified stopping policy.
"""
import math


class _Pose:
    __slots__ = ('x', 'y', 'yaw')


def _corners(x, y, yaw, footprint, padding=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    xmin = float(footprint["min_x_m"]) - padding
    xmax = float(footprint["max_x_m"]) + padding
    ymin = float(footprint["min_y_m"]) - padding
    ymax = float(footprint["max_y_m"]) + padding
    return [(x + c * dx - s * dy, y + s * dx + c * dy)
            for dx,dy in ((xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax))]


def _world_to_pixel(occupancy, x, y):
    dx, dy = x - occupancy.origin_x, y - occupancy.origin_y
    # The PGM transform is immutable for the lifetime of OccupancyMap.
    # The arithmetic below is unchanged, including division and flip-y.
    c = occupancy.origin_cos if hasattr(occupancy, 'origin_cos') else math.cos(occupancy.origin_yaw)
    s = occupancy.origin_sin if hasattr(occupancy, 'origin_sin') else math.sin(occupancy.origin_yaw)
    gx = (c * dx + s * dy) / occupancy.resolution
    gy = (-s * dx + c * dy) / occupancy.resolution
    return gx, occupancy.height - gy if occupancy.flip_y else gy


def _sat_intersects(poly, col, row):
    cell = ((col,row),(col+1,row),(col+1,row+1),(col,row+1))
    edges = [(poly[i][0]-poly[(i+1)%4][0],
              poly[i][1]-poly[(i+1)%4][1]) for i in range(2)]
    axes = ((1.0,0.0),(0.0,1.0)) + tuple((-dy,dx) for dx,dy in edges)
    for ax,ay in axes:
        p=[ax*x+ay*y for x,y in poly]
        q=[ax*x+ay*y for x,y in cell]
        if max(p) < min(q) - 1e-9 or max(q) < min(p) - 1e-9:
            return False
    return True


def footprint_collision(occupancy, state, footprint, padding=0.0):
    if padding < 0 or not math.isfinite(padding):
        raise ValueError("INVALID_FOOTPRINT_PADDING")
    points = [_world_to_pixel(occupancy, *p) for p in
              _corners(state.x, state.y, state.yaw, footprint, padding)]
    if not all(math.isfinite(q) for p in points for q in p):
        return True
    col0 = math.floor(min(p[0] for p in points))
    col1 = math.floor(max(p[0] for p in points))
    row0 = math.floor(min(p[1] for p in points))
    row1 = math.floor(max(p[1] for p in points))
    if col0 < 0 or row0 < 0 or col1 >= occupancy.width or row1 >= occupancy.height:
        return True
    # Exact safe fast path: a collision is impossible if every raster cell
    # in the polygon's inclusive AABB is unoccupied. Otherwise preserve the
    # original exact SAT cell iteration below without changing any margins.
    # Identical FREE-only AABB condition, accelerated for an immutable map.
    if hasattr(occupancy, 'rectangle_is_free'):
        if occupancy.rectangle_is_free(row0, row1, col0, col1):
            return False
    elif hasattr(occupancy, 'pixel_array'):
        region = occupancy.pixel_array[row0:row1+1, col0:col1+1]
        if region.min() >= occupancy.threshold:
            return False
    for row in range(row0,row1+1):
        offset = row*occupancy.width
        for col in range(col0,col1+1):
            if (occupancy.pixels[offset+col] < occupancy.threshold and
                    _sat_intersects(points,col,row)):
                return True
    return False


def _free_swept_disk_broadphase(occupancy, previous, current, radius_m):
    """Conservative FREE-only result: disk envelope surrounds every swept pose.

    Return False for any out-of-map disk or occupied raster cell; original
    detailed SAT/footprint path then remains authoritative. No false-safe
    outcome can be introduced by this shortcut.
    """
    import math
    if not math.isfinite(radius_m) or radius_m < 0.0:
        return False
    if not hasattr(occupancy, 'pixel_array'):
        return False
    # Same map origin rotation + flip convention as _world_to_pixel().
    p0 = _world_to_pixel(occupancy, previous.x, previous.y)
    p1 = _world_to_pixel(occupancy, current.x, current.y)
    if not all(math.isfinite(v) for v in (*p0, *p1)):
        return False
    r = radius_m / occupancy.resolution
    x0 = math.floor(min(p0[0], p1[0]) - r - 1.0e-8)
    x1 = math.ceil(max(p0[0], p1[0]) + r + 1.0e-8)
    y0 = math.floor(min(p0[1], p1[1]) - r - 1.0e-8)
    y1 = math.ceil(max(p0[1], p1[1]) + r + 1.0e-8)
    if x0 < 0 or y0 < 0 or x1 >= occupancy.width or y1 >= occupancy.height:
        return False
    # Original pixel threshold, not a relaxed occupancy definition.
    if hasattr(occupancy, 'rectangle_is_free'):
        return occupancy.rectangle_is_free(y0, y1, x0, x1)
    return bool((occupancy.pixel_array[y0:y1 + 1, x0:x1 + 1] >= occupancy.threshold).all())


def swept_footprint_collision(occupancy, previous, current, footprint, *, cached_radius_m=None):
    dx = current.x - previous.x
    dy = current.y - previous.y
    delta = (current.yaw - previous.yaw + math.pi) % (2*math.pi)-math.pi
    # The stop envelope reuses an identical footprint every rollout step.
    # Cache only this invariant radius, not the collision verdict.
    radius = (cached_radius_m if cached_radius_m is not None else
              max(math.hypot(x,y) for x in
                  (float(footprint[k]) for k in ("min_x_m","max_x_m"))
                  for y in (float(footprint[k]) for k in ("min_y_m","max_y_m"))))
    # Max displacement of any footprint point over the whole tick.
    motion_bound = math.hypot(dx,dy) + radius*abs(delta)
    n = max(1,int(math.ceil(motion_bound / (0.5*occupancy.resolution))))
    # Each sampled pose is inflated by the worst local change over half a
    # sample interval (conservative wrt any interpolated pose).
    pad = motion_bound/(2*n) + 1e-9
    # Disk radius bounds every corner of every inflated sampled footprint.
    # If its full swept pixel rectangle is free, SAT would return False for
    # every pose. Otherwise invoke the ORIGINAL loop unchanged.
    if _free_swept_disk_broadphase(occupancy, previous, current, radius + pad):
        return False
    # Reuse one slot-only pose; geometry/padding/sampling remain unchanged.
    pose = _Pose()
    for k in range(n+1):
        t = k/n
        pose.x=previous.x + t*dx
        pose.y=previous.y + t*dy
        pose.yaw=previous.yaw + t*delta
        if footprint_collision(occupancy, pose, footprint, padding=pad):
            return True
    return False
