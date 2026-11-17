"""R-A/S0-R offline approval boundary. No runtime/high-speed authority granted.

A newly generated experimental SCP candidate can be promoted to an *offline*
nominal candidate only after independent collision, jerk, dynamics, identity
and integrity checks. Does not edit the original input or existing registry.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from f1tenth_racing_raceline.trajectory import (
    RacingTrajectory, load_racing_trajectory, write_racing_trajectory)
from f1tenth_racing_sim.track_preflight import preflight_json


def source_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def numerical_qualification(trajectory, environment):
    v = np.array([sample.target_speed_mps for sample in trajectory.samples])
    acc = np.array([sample.target_accel_mps2 for sample in trajectory.samples])
    curvature = np.array([sample.curvature_per_m for sample in trajectory.samples])
    lengths = np.array([sample.ds_to_next_m for sample in trajectory.samples])
    limits = environment['dynamics']['limits']
    speed_cap = float(limits['speed_abs_max_m_s'])
    accel_cap = float(limits['acceleration_max_m_s2'])
    braking = float(limits['deceleration_max_m_s2'])
    jerk_cap = float(limits['jerk_abs_max_m_s3'])
    lat_cap = float(environment.get('offline_qualified_lateral_acceleration_m_s2',
                                    environment['dynamics'].get('lateral_acceleration_ceiling_m_s2', 0)))
    # Do not invent a certified lateral acceleration from maximum simulator speed.
    if lat_cap <= 0:
        raise ValueError('offline-qualified lateral acceleration unavailable: explicit measured bound required')
    dt = 2 * lengths / (v + np.roll(v,-1))
    jerk = (np.roll(acc,-1)-acc) / (0.5*(dt+np.roll(dt,-1)))
    tol=1e-8
    if not (np.all(np.isfinite(v)) and np.all(v > 0) and
            np.max(v) <= speed_cap + tol and
            np.max(acc) <= accel_cap + tol and
            np.min(acc) >= -braking - tol and
            np.max(v*v*np.abs(curvature)) <= lat_cap + tol and
            np.max(np.abs(jerk)) <= jerk_cap + tol):
        raise ValueError('nonlinear vehicle/jerk/lateral dynamics check rejected')
    return {'max_speed_mps':float(np.max(v)),
            'max_abs_jerk_mps3':float(np.max(np.abs(jerk))),
            'max_abs_lateral_acceleration_mps2':float(np.max(v*v*np.abs(curvature))),
            'bound_lateral_acceleration_mps2':lat_cap}


def qualify_experimental(environment_file, csv_file, metadata_file, output_dir):
    environment=yaml.safe_load(Path(environment_file).read_text())
    incoming=load_racing_trajectory(csv_file,metadata_file)
    if incoming.metadata.get('qualification_state') != 'OFFLINE_EXPERIMENTAL_UNQUALIFIED':
        raise ValueError('candidate is not an unqualified experimental trajectory')
    metrics=numerical_qualification(incoming,environment)
    swept=preflight_json(environment_file,csv_file,metadata_file)
    output_dir=Path(output_dir)
    if output_dir.exists():
        raise ValueError('qualification destination already exists')
    output_dir.mkdir(parents=True)
    receipt={'contract_version':1,'source_trajectory_id':incoming.trajectory_id,
             'experimental_csv_sha256':source_hash(csv_file),
             'experimental_metadata_sha256':source_hash(metadata_file),
             'map_image_sha256':swept['map_image_sha256'],
             'environment_sha256':swept['environment_sha256'],
             'track_id':swept['track_id'],'swept':swept,'dynamics':metrics,
             'status':'OFFLINE_ONLY_RUNTIME_NOT_QUALIFIED'}
    receipt_path=output_dir/'offline_qualification_receipt.json'
    receipt_path.write_text(json.dumps(receipt,sort_keys=True,indent=2)+'\n')
    metadata=dict(incoming.metadata)
    metadata.pop('trajectory_id',None);metadata.pop('csv_sha256',None)
    metadata.update(qualification_state='OFFLINE_PRIMARY_CANDIDATE_QUALIFIED',
                    geometry_qualification_status='VERIFIED',
                    speed_envelope_qualification_status='VERIFIED',
                    trajectory_tier='PRIMARY_CANDIDATE',runtime_tracking_qualified=False,
                    generation_method='R_A3_SPARSE_SCP_JOINT_OFFLINE_QUALIFIED')
    metadata['source']=dict(metadata['source'],
                            offline_receipt_sha256=source_hash(receipt_path),
                            experimental_source_trajectory_id=incoming.trajectory_id)
    candidate=RacingTrajectory(incoming.samples,metadata)
    output_csv,output_metadata,written=write_racing_trajectory(
        output_dir,candidate,stem=swept['track_id']+'_r_a3_scp_offline_qualified')
    policy={'frame_id':'map','track_id':swept['track_id'],
            'map_image_sha256':swept['map_image_sha256'],
            'default_candidate':'scp',
            'candidates':{'scp':{'csv':output_csv.name,'metadata':output_metadata.name,
                                 'source_trajectory_id':written.trajectory_id}}}
    (output_dir/'nominal_reference.json').write_text(json.dumps(policy,indent=2,sort_keys=True)+'\n')
    return receipt_path, written.trajectory_id
