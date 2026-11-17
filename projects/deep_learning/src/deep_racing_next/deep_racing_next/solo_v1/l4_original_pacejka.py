"""L4: offline training adapter using byte-identical original F1TENTH Pacejka core.

NO ROS actuation. NO reproduction of guarded ROS simulator lifecycle. This is
an actual numerical integration of the user's original physics primitives,
not f110_gym, not a kinematic mock, and not a physical-racing certificate.
"""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import yaml

from .l3_driver import action_from_normalized
from .original_frozen.physics.vehicle import Dynamics, State
from .original_frozen.world.map import OccupancyMap
from .original_frozen.world.swept_collision import footprint_collision, swept_footprint_collision
from .original_frozen.sensing.lidar import scan_occupancy


class OriginalPhysicsHold(RuntimeError):
    pass


# Derived by SHA256 of the ten EXACT original files supplied by the user.
EXPECTED_ORIGINAL_CORE_SHA256 = {
    'common/__init__.py': 'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855',
    'common/geometry.py': '3ed30a5f4625704b346ad33c29fb6458ebc6f9c212c3af6cf1658903c52ee20d',
    'physics/__init__.py': 'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855',
    'physics/pacejka.py': 'f1fe3b4019c653c4e1d8c6984af39ee6f8c74ff1027b589c5a3d7c8bbc2a0a36',
    'physics/vehicle.py': '68abf966e1ca8b97209640770bf52daa50c18a23990200b667370655378222f1',
    'world/__init__.py': 'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855',
    'world/map.py': 'ef08d442cb99781104c59afc1a38636777996ae91deaf00a494af48e2b78452e',
    'world/swept_collision.py': 'd377dd42becaeecc76b2f3caf06a2bebe69258a83282018d468a7268364f899c',
    'sensing/__init__.py': 'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855',
    'sensing/lidar.py': '8adf16ecdc11118e6463bc73a6157a37b73e88ef62fa531aa1858d38be9e22e0',
}


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def _within(root, candidate):
    root = Path(root).resolve()
    candidate = Path(candidate)
    if candidate.is_absolute() or '..' in candidate.parts:
        raise OriginalPhysicsHold('PATH_TRAVERSAL_OR_ABSOLUTE_REFUSED')
    path = (root / candidate).resolve()
    if not path.is_relative_to(root) or not path.is_file() or path.is_symlink():
        raise OriginalPhysicsHold(f'REQUIRED_SOURCE_ASSET_MISSING:{candidate}')
    return path


def find_legacy_sim_root(course, workspace=None):
    if workspace is None:
        for ancestor in Path(course.yaml_path).resolve().parents:
            if (ancestor / 'legacy' / 'CURRENT.json').is_file():
                workspace = ancestor
                break
    if workspace is None:
        raise OriginalPhysicsHold('WORKSPACE_LEGACY_NOT_FOUND')
    workspace = Path(workspace).resolve()
    current = workspace / 'legacy' / 'CURRENT.json'
    if not current.is_file():
        raise OriginalPhysicsHold('LEGACY_POINTER_MISSING')
    candidates = list((workspace / 'legacy' / 'snapshots').glob(
        '*/original/src/environment/f1tenth_racing_sim'))
    valid = []
    for root in candidates:
        core = root / 'f1tenth_racing_sim/physics/pacejka.py'
        if core.is_file() and sha256(core) == EXPECTED_ORIGINAL_CORE_SHA256['physics/pacejka.py']:
            valid.append(root)
    if len(valid) != 1:
        raise OriginalPhysicsHold(f'ORIGINAL_SIM_ROOT_NOT_UNIQUE_OR_MISSING:{len(valid)}')
    return valid[0]


def verify_original_core(source_root):
    package = Path(__file__).resolve().parent / 'original_frozen'
    origin = Path(source_root).resolve() / 'f1tenth_racing_sim'
    fingerprints = {}
    for relative, required in EXPECTED_ORIGINAL_CORE_SHA256.items():
        frozen = _within(package, Path(relative))
        original = _within(origin, Path(relative))
        fp = sha256(frozen)
        if fp != required or sha256(original) != required:
            raise OriginalPhysicsHold('ORIGINAL_CODE_IDENTITY_DRIFT:' + relative)
        fingerprints[relative] = fp
    return fingerprints


def load_original_configuration(source_root, course):
    root = Path(source_root).resolve()
    configs = {'iccas2025': 'competition_environment.yaml',
               'Spielberg': 'spielberg_environment.yaml'}
    if course.name not in configs:
        raise OriginalPhysicsHold('NO_ORIGINAL_PHYSICS_MAP_AUTHORITY_FOR:' + course.name)
    cfg_path = _within(root, Path('config') / configs[course.name])
    config = yaml.safe_load(cfg_path.read_text(encoding='utf-8-sig'))
    if not isinstance(config, dict):
        raise OriginalPhysicsHold('INVALID_ORIGINAL_ENVIRONMENT_YAML')
    mc, dyn, sim = config['map'], config['dynamics'], config['simulation']
    dt = float(sim['physics_timestep_s'])
    if not math.isclose(dt, 1.0 / float(sim['physics_rate_hz']), abs_tol=1e-12):
        raise OriginalPhysicsHold('ORIGINAL_PHYSICS_RATE_MISMATCH')
    speed_cap = float(dyn['limits']['speed_abs_max_m_s'])
    if speed_cap != 20.0:
        raise OriginalPhysicsHold('EXPECTED_ORIGINAL_20MPS_CAP_MISMATCH')
    if dyn['model'] != 'pacejka_single_track' or sim.get('physics_rate_hz') != 40:
        raise OriginalPhysicsHold('ORIGINAL_DYNAMICS_CONTRACT_CHANGED')
    if abs(float(mc['origin']['yaw_rad'])) > 1e-10:
        raise OriginalPhysicsHold('ORIGINAL_RAYCAST_ROTATED_MAP_NOT_QUALIFIED')
    if not mc.get('runtime_flip_y', False):
        raise OriginalPhysicsHold('UNEXPECTED_ORIGINAL_GRID_ORIENTATION')
    image = _within(root, Path(mc['image_file']))
    meta = _within(root, Path(mc['metadata_file']))
    center = _within(root, Path(mc['centerline_file']))
    for kind, path in [('image', image), ('metadata', meta), ('centerline', center)]:
        expected = mc[kind + '_sha256']
        if sha256(path) != expected:
            raise OriginalPhysicsHold('AUTHORITATIVE_MAP_SHA_DRIFT:' + kind)
    if course.fingerprint['centerline_sha256'] != mc['centerline_sha256']:
        raise OriginalPhysicsHold('COURSE_ORIGINAL_CENTERLINE_SHA_MISMATCH')
    if image.suffix.lower() != '.pgm':
        raise OriginalPhysicsHold('ORIGINAL_OCCUPANCY_REQUIRES_PGM')
    return config, {'environment_yaml': sha256(cfg_path),
                    'map_image': sha256(image), 'map_metadata': sha256(meta),
                    'centerline': sha256(center)}, image


class OriginalPacejkaBackend:
    """Offline original dynamics / original raster collision / reduced-beam original raycast.

    The *dynamics* are byte-identical to the original ROS simulator's core,
    while the new longitudinal target-speed controller is explicitly an
    experimental wrapper. Safety Guard/watchdog ROS authority is NOT replicated.
    """
    qualified_physics = 'ORIGINAL_PACEJKA_OFFLINE_CORE_NOT_GUARDED_ROS_CERTIFIED'

    def __init__(self, course, dt=.025, workspace=None, original_source_root=None,
                 scan_beams=108, speed_kp=4.0):
        self.course = course
        self.original_root = (Path(original_source_root).resolve()
            if original_source_root is not None else find_legacy_sim_root(course, workspace))
        self.core_fingerprints = verify_original_core(self.original_root)
        config, self.asset_fingerprints, image = load_original_configuration(
            self.original_root, course)
        expected_dt = float(config['simulation']['physics_timestep_s'])
        if not math.isfinite(dt) or not math.isclose(float(dt), expected_dt, rel_tol=0, abs_tol=1e-12):
            raise OriginalPhysicsHold(f'PHYSICS_DT_MUST_MATCH_SOURCE:{expected_dt}')
        if type(scan_beams) is not int or not 16 <= scan_beams <= int(config['lidar']['beam_count']):
            raise OriginalPhysicsHold('INVALID_DOWNSAMPLED_BEAM_COUNT')
        if not math.isfinite(speed_kp) or speed_kp <= 0:
            raise OriginalPhysicsHold('INVALID_LONGITUDINAL_CONTROLLER')
        self.dt = expected_dt
        self.c = config
        self.dynamics = Dynamics(config['dynamics'])
        self.map = OccupancyMap(image, config['map'])
        self.footprint = config['collision']['footprint_relative_to_base_link']
        self.scan_beams = scan_beams
        self.speed_kp = float(speed_kp)
        self.state = None
        self.terminated = True
        self.ticks = 0
        self.fingerprints = {
            'core': self.core_fingerprints, 'assets': self.asset_fingerprints,
            'original_source_root': str(self.original_root),
            'trainer_scan_beams': scan_beams,
            'original_simulator_lidar_beams': int(config['lidar']['beam_count']),
            'dt_s': self.dt, 'speed_cap_mps': 20.0,
            'longitudinal_controller': 'EXPERIMENTAL_TARGET_SPEED_PROPORTIONAL_KP_4',
            'guarded_ros_ego_authority': False,
        }

    def reset(self, pose):
        pose = np.asarray(pose, dtype=float)
        if pose.shape != (3,) or not np.isfinite(pose).all():
            raise OriginalPhysicsHold('INVALID_START_POSE')
        ref = np.asarray(self.course.frame(0), dtype=float)
        yaw_delta = math.remainder(float(pose[2] - ref[2]), 2 * math.pi)
        if np.linalg.norm(pose[:2] - ref[:2]) > 1e-8 or abs(yaw_delta) > 1e-8:
            raise OriginalPhysicsHold('START_MUST_BE_COURSE_START_LINE')
        state = State(x=float(pose[0]), y=float(pose[1]), yaw=float(pose[2]))
        if footprint_collision(self.map, state, self.footprint):
            raise OriginalPhysicsHold('START_FOOTPRINT_COLLIDES_WITH_MAP')
        self.state = state
        self.terminated = False
        self.ticks = 0
        obs = self._observe(False)
        if obs['v'] != 0.0:
            raise OriginalPhysicsHold('START_NOT_AT_REST')
        return obs

    def step(self, action):
        if self.terminated or self.state is None:
            raise OriginalPhysicsHold('STEP_AFTER_TERMINATION_OR_BEFORE_RESET')
        steering, target_speed = action_from_normalized(action)
        if not math.isfinite(steering) or not math.isfinite(target_speed):
            raise OriginalPhysicsHold('NONFINITE_POLICY_ACTION')
        max_accel = float(self.c['dynamics']['limits']['acceleration_max_m_s2'])
        max_decel = float(self.c['dynamics']['limits']['deceleration_max_m_s2'])
        requested_accel = float(np.clip(self.speed_kp * (target_speed - self.state.speed),
                                        -max_decel, max_accel))
        previous = State(x=self.state.x, y=self.state.y, yaw=self.state.yaw)
        self.dynamics.step(self.state, steering, requested_accel, self.dt)
        vals = vars(self.state)
        if not all(math.isfinite(float(value)) for value in vals.values()):
            self.terminated = True
            raise OriginalPhysicsHold('NONFINITE_PHYSICS_STATE')
        self.ticks += 1
        breach = abs(self.state.speed) > 20.0000001
        collision = bool(swept_footprint_collision(self.map, previous, self.state,
                                                   self.footprint))
        reverse = self.state.speed < -1e-6
        self.terminated = bool(collision or reverse or breach)
        obs = self._observe(collision)
        if breach:
            raise OriginalPhysicsHold('PHYSICAL_SPEED_CAP_BREACH')
        return obs, self.terminated, {'collision': collision,
                 'reverse_detected': reverse, 'source_core_sha256':
                 self.core_fingerprints['physics/vehicle.py'],
                 'physics_timestep_s': self.dt}

    def _observe(self, collision):
        if self.state is None:
            raise OriginalPhysicsHold('UNINITIALIZED_PHYSICS_STATE')
        state = self.state
        c = self.c['lidar']
        mount = self.c['vehicle']['description']
        origin_x = state.x + mount['lidar_x_m'] * math.cos(state.yaw) - mount['lidar_y_m'] * math.sin(state.yaw)
        origin_y = state.y + mount['lidar_x_m'] * math.sin(state.yaw) + mount['lidar_y_m'] * math.cos(state.yaw)
        angles = state.yaw + np.linspace(c['angle_min_rad'], c['angle_max_rad'],
                                         self.scan_beams)
        scan = scan_occupancy(self.map, origin_x, origin_y, angles,
                              c['range_min_m'], c['range_max_m'])
        if not np.isfinite(scan).all():
            raise OriginalPhysicsHold('NONFINITE_ORIGINAL_LIDAR_SCAN')
        return {'x': state.x, 'y': state.y, 'yaw': state.yaw,
                'v': state.speed, 'yaw_rate': state.yaw_rate,
                'scan': np.asarray(scan, dtype=float),
                'collision': bool(collision), 'sim_label': self.qualified_physics}

    def close(self):
        self.terminated = True
        self.state = None
