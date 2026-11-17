from pathlib import Path
import math
from types import SimpleNamespace
import numpy as np
import yaml

from f1tenth_racing_sim.physics.vehicle import Dynamics, State
from f1tenth_racing_sim.sensing.lidar import scan_occupancy
from f1tenth_racing_sim.sensing.raycast import ray_box_ranges
from f1tenth_racing_sim.world.collision import footprints_overlap
from f1tenth_racing_sim.world.map import OccupancyMap
from f1tenth_racing_sim.world.track import load_centerline, pose_at_s
from f1tenth_racing_sim.simulator import Simulator


ROOT = Path(__file__).parents[1]
CONFIG = yaml.safe_load((ROOT / "config/competition_environment.yaml").read_text())


def test_initialized_straight_motion_and_limits():
    state = State()
    dynamics = Dynamics(CONFIG["dynamics"])
    for _ in range(40):
        dynamics.step(state, 0.0, 2.0, 0.025)
    assert state.x > 0.5
    assert state.y == 0.0
    assert 1.9 < state.speed < 2.1


def test_stationary_vehicle_does_not_integrate_residual_yaw_rate():
    state = State(yaw=0.4, speed=0.0, yaw_rate=1.0)
    Dynamics(CONFIG["dynamics"]).step(state, 0.0, 0.0, 0.025)
    assert math.isclose(state.yaw, 0.4, abs_tol=1.0e-12)
    assert state.yaw_rate == 0.0


def test_map_identity_and_outside_is_occupied():
    occupancy = OccupancyMap(ROOT / CONFIG["map"]["image_file"], CONFIG["map"])
    assert (occupancy.width, occupancy.height) == (378, 464)
    assert occupancy.occupied(-100.0, -100.0)
    scan = scan_occupancy(occupancy, -2.5768695672353106, 9.001046180725098, np.array([0.0, math.pi]), 0.1, 10.0)
    assert len(scan) == 2
    assert all(0.1 <= value <= 10.0 for value in scan)


def test_track_pose_dynamic_ray_and_collision_geometry():
    centerline = load_centerline(ROOT / CONFIG["map"]["centerline_file"])
    blue = State(*pose_at_s(centerline, 0.0))
    red = State(*pose_at_s(centerline, 8.0))
    assert math.hypot(red.x - blue.x, red.y - blue.y) > 0.4
    assert not footprints_overlap(blue, red, CONFIG["collision"]["footprint_relative_to_base_link"])
    overlapping = State(x=blue.x + 0.1, y=blue.y, yaw=blue.yaw)
    assert footprints_overlap(blue, overlapping, CONFIG["collision"]["footprint_relative_to_base_link"])
    visible = State(x=blue.x + 1.0, y=blue.y, yaw=blue.yaw)
    angle = np.array([math.atan2(visible.y - blue.y, visible.x - blue.x)])
    hit = ray_box_ranges(blue.x, blue.y, angle, visible, CONFIG["collision"]["footprint_relative_to_base_link"], 10.0)
    assert hit[0] < 10.0


def test_static_map_publication_stops_after_the_transient_local_sample(monkeypatch):
    class Message(SimpleNamespace):
        def __init__(self):
            super().__init__(
                header=SimpleNamespace(),
                info=SimpleNamespace(origin=SimpleNamespace(
                    position=SimpleNamespace(), orientation=SimpleNamespace(),
                )),
                pose=SimpleNamespace(
                    position=SimpleNamespace(), orientation=SimpleNamespace(),
                ),
                poses=[],
            )

    import f1tenth_racing_sim.simulator as simulator
    monkeypatch.setattr(simulator, "OccupancyGrid", Message)
    monkeypatch.setattr(simulator, "PathMsg", Message)
    monkeypatch.setattr(simulator, "PoseStamped", Message)
    published, cancelled = [], []
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        config={"map": {"frame_id": "map"}},
        map=SimpleNamespace(resolution=1.0, width=1, height=1, origin_x=0.0,
                            origin_y=0.0, pixels=[255], flip_y=False, threshold=128),
        centerline=[{"x_m": 0.0, "y_m": 0.0}],
        map_pub=SimpleNamespace(publish=lambda message: published.append(message)),
        path_pub=SimpleNamespace(publish=lambda message: published.append(message)),
        map_timer=SimpleNamespace(cancel=lambda: cancelled.append(True)),
    )
    Simulator.publish_map_and_path(node)
    assert len(published) == 2
    assert cancelled == [True]
