"""Verified H2H map/centerline binding shared by both R-E ROS executables.

This checks source identity, not the trajectory's safety. S0-R and Guard
must still independently validate every proposed trajectory/command.
"""
import csv
from hashlib import sha256
import math
from pathlib import Path
import yaml


def load_verified_runtime(environment_file, centerline_file, sim_share,
                          *, expected_track_id='', expected_map_sha256=''):
    env_path=Path(environment_file).resolve(strict=True)
    environment=yaml.safe_load(env_path.read_text())
    if not isinstance(environment,dict):
        raise ValueError('H2H_ENVIRONMENT_INVALID')
    track=environment['environment_identity']['active_track']
    area=environment['map']
    if (expected_track_id and track!=expected_track_id):
        raise ValueError('H2H_TRACK_MISMATCH')
    if (expected_map_sha256 and area['image_sha256']!=expected_map_sha256):
        raise ValueError('H2H_MAP_IDENTITY_MISMATCH')
    map_file=(Path(sim_share)/area['image_file']).resolve(strict=True)
    if not map_file.is_file() or sha256(map_file.read_bytes()).hexdigest()!=area['image_sha256']:
        raise ValueError('H2H_MAP_FILE_SHA_MISMATCH')
    centerline=Path(centerline_file or str(Path(sim_share)/area['centerline_file'])).resolve(strict=True)
    if not centerline.is_file() or sha256(centerline.read_bytes()).hexdigest()!=area['centerline_sha256']:
        raise ValueError('H2H_CENTERLINE_SHA_MISMATCH')
    with centerline.open(newline='') as stream:
        rows=tuple(csv.DictReader(stream))
    if len(rows)<3:
        raise ValueError('H2H_CENTERLINE_TOO_SHORT')
    for row in rows:
        vals=(float(row['x_m']),float(row['y_m']),
              float(row['w_tr_left_m']),float(row['w_tr_right_m']))
        if not all(math.isfinite(v) for v in vals) or min(vals[2:])<=0:
            raise ValueError('H2H_CENTERLINE_INVALID_GEOMETRY')
    return environment,rows
