import math


def footprints_overlap(first, second, footprint):
    half_x = (footprint["max_x_m"] - footprint["min_x_m"]) * 0.5
    half_y = (footprint["max_y_m"] - footprint["min_y_m"]) * 0.5
    center_x = (footprint["max_x_m"] + footprint["min_x_m"]) * 0.5
    center_y = (footprint["max_y_m"] + footprint["min_y_m"]) * 0.5

    def corners(state):
        c, s = math.cos(state.yaw), math.sin(state.yaw)
        cx = state.x + center_x * c - center_y * s
        cy = state.y + center_x * s + center_y * c
        return [
            (
                cx + sx * half_x * c - sy * half_y * s,
                cy + sx * half_x * s + sy * half_y * c,
            )
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]

    a, b = corners(first), corners(second)
    axes = [
        (math.cos(first.yaw), math.sin(first.yaw)),
        (-math.sin(first.yaw), math.cos(first.yaw)),
        (math.cos(second.yaw), math.sin(second.yaw)),
        (-math.sin(second.yaw), math.cos(second.yaw)),
    ]
    for axis_x, axis_y in axes:
        pa = [x * axis_x + y * axis_y for x, y in a]
        pb = [x * axis_x + y * axis_y for x, y in b]
        if max(pa) < min(pb) or max(pb) < min(pa):
            return False
    return True


def swept_vehicle_disk_overlap(ego_previous, ego_current,
                               red_previous, red_current, footprint,
                               additional_margin_m=0.0):
    """Conservative *sufficient* swept collision detector (disks enclose cars).

    A positive result MAY be a false positive; a negative result certifies
    nonintersection of the two linearly moving disk enclosures. This is
    diagnostic collision detection, not a vehicle avoidance certificate.
    Both paths span the same simulation interval; async Red state age is
    handled by the caller conservatively.
    """
    if not all(math.isfinite(x) for x in (
            ego_previous.x,ego_previous.y,ego_current.x,ego_current.y,
            red_previous.x,red_previous.y,red_current.x,red_current.y,
            additional_margin_m)) or additional_margin_m<0:
        return True
    radius=max(math.hypot(x,y) for x in (
        footprint['min_x_m'],footprint['max_x_m']) for y in (
        footprint['min_y_m'],footprint['max_y_m']))
    x0=ego_previous.x-red_previous.x
    y0=ego_previous.y-red_previous.y
    dx=(ego_current.x-red_current.x)-x0
    dy=(ego_current.y-red_current.y)-y0
    vv=dx*dx+dy*dy
    u=max(0.,min(1.,-(x0*dx+y0*dy)/vv)) if vv>0 else 0.
    return math.hypot(x0+u*dx,y0+u*dy)<=2*radius+additional_margin_m
