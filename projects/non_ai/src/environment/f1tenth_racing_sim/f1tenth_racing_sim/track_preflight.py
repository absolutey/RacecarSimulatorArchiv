"""Offline track asset and swept rectangle preflight (not runtime qualification).

No mock map or automatic qualification is generated. All official inputs must
exist, hashes must match configuration, and every wheelbase footprint sweep
must avoid occupied map cells. Validation is conservative and may reject.
"""
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def asset_bundle(environment_path):
    from f1tenth_racing_sim.world.map import OccupancyMap
    envpath = Path(environment_path).resolve(strict=True)
    env = yaml.safe_load(envpath.read_text())
    base = envpath.parent
    def path_for(name):
        p = Path(env['map'][name])
        if not p.is_absolute():
            options = (base / p, base.parent / p)
            p = next((v for v in options if v.is_file()), options[0])
        return p.resolve(strict=True)
    source = {k: path_for(k) for k in ('image_file','metadata_file','centerline_file')}
    for name, key in [('image_file','image_sha256'),('metadata_file','metadata_sha256'),('centerline_file','centerline_sha256')]:
        expected = env['map'].get(key)
        if not expected or expected != sha(source[name]):
            raise ValueError(f'{name} SHA mismatch or missing expected hash')
    yaml_map = yaml.safe_load(source['metadata_file'].read_text())
    if Path(str(yaml_map.get('image', ''))).name != source['image_file'].name:
        raise ValueError('PGM/YAML image name mismatch')
    origin = env['map']['origin']
    yaml_origin = yaml_map['origin']
    for actual, expected in zip(yaml_origin, (origin['x_m'], origin['y_m'], origin.get('yaw_rad',0.0))):
        if not math.isclose(float(actual), float(expected), abs_tol=1e-7):
            raise ValueError('map YAML vs environment origin mismatch')
    if not math.isclose(float(yaml_map['resolution']), float(env['map']['resolution_m_per_pixel']), abs_tol=1e-9):
        raise ValueError('map YAML vs environment resolution mismatch')
    if not env['environment_identity'].get('active_track'):
        raise ValueError('missing track identity')
    with source['centerline_file'].open(newline='') as f:
        points = list(csv.DictReader(f))
    if len(points) < 3:
        raise ValueError('centerline too short')
    return env, source, points, OccupancyMap(source['image_file'], env['map'])


def swept_footprint_validate(samples, footprint, occupancy, *, sweep_step_m=0.02, clearance_m=0.05):
    """Conservative grid-sampled rotated body: edge & interior, pose-sweep.

    Uses half-cell sampling of the occupied map lattice. This is a necessary
    offline clearance check; it does not certify control-error margins.
    """
    points = list(samples)
    if len(points) < 3:
        raise ValueError('closed trajectory requires >=3 samples')
    if sweep_step_m <= 0 or clearance_m < 0:
        raise ValueError('invalid swept sampling limits')
    step = min(sweep_step_m, float(occupancy.resolution) * 0.5)
    xs = np.arange(footprint['min_x_m']-clearance_m, footprint['max_x_m']+clearance_m+step/2, step)
    ys = np.arange(footprint['min_y_m']-clearance_m, footprint['max_y_m']+clearance_m+step/2, step)
    collision = 0
    visited = 0
    for i, a in enumerate(points):
        b = points[(i+1) % len(points)]
        length = math.hypot(b.x_m-a.x_m, b.y_m-a.y_m)
        angle = math.atan2(math.sin(b.heading_rad-a.heading_rad), math.cos(b.heading_rad-a.heading_rad))
        steps = max(1, math.ceil(length/step), math.ceil(abs(angle)/0.03))
        for k in range(steps):
            frac = k/steps
            x = a.x_m + frac * (b.x_m-a.x_m)
            y = a.y_m + frac * (b.y_m-a.y_m)
            yaw = a.heading_rad + angle*frac
            c, s = math.cos(yaw), math.sin(yaw)
            for dx in xs:
                for dy in ys:
                    visited += 1
                    if occupancy.occupied(x + c*dx - s*dy, y + s*dx + c*dy):
                        collision += 1
                        raise ValueError(f'swept footprint occupied: segment={i} fraction={frac:.5f}')
    return {'swept_poses_and_points_checked':visited,'hits':collision,'sweep_step_m':step,'clearance_m':clearance_m}


def preflight_json(environment_path, trajectory_csv, trajectory_metadata, *, output=None):
    from f1tenth_racing_raceline.trajectory import load_racing_trajectory
    env, sources, centerline, occupancy = asset_bundle(environment_path)
    traj = load_racing_trajectory(trajectory_csv, trajectory_metadata)
    if traj.metadata.get('track_id') != env['environment_identity']['active_track']:
        raise ValueError('trajectory track mismatch')
    if traj.metadata.get('source', {}).get('map_image_sha256') != env['map']['image_sha256']:
        raise ValueError('trajectory map SHA mismatch')
    if traj.metadata.get('runtime_tracking_qualified') is not False:
        raise ValueError('offline tool must not claim runtime qualification')
    result = swept_footprint_validate(traj.samples, env['collision']['footprint_relative_to_base_link'], occupancy)
    result.update({'track_id':traj.metadata['track_id'], 'trajectory_id':traj.trajectory_id,
                   'csv_sha256':sha(trajectory_csv),'metadata_sha256':sha(trajectory_metadata),
                   'environment_sha256':sha(environment_path),'map_image_sha256':sha(sources['image_file']),
                   'runtime_tracking_qualified':False,'status':'OFFLINE_SWEPT_PASS_NO_RUNTIME_APPROVAL'})
    if output:
        Path(output).write_text(json.dumps(result, indent=2, sort_keys=True)+'\n')
    return result
