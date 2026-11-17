"""Offline artifact admission only; NOT a runtime tracking or Guard certificate."""
import json
from pathlib import Path
import pytest
from ament_index_python.packages import get_package_share_directory
from f1tenth_racing_raceline import load_racing_trajectory
from f1tenth_racing_reference.contract_guard import load_nominal_policy
from f1tenth_racing_reference.core import select_nominal


def _registered():
    reference=Path(get_package_share_directory('f1tenth_racing_reference'))
    raceline=Path(get_package_share_directory('f1tenth_racing_raceline'))
    policy_file=reference/'config/spielberg_nominal_reference.json'
    raw=json.loads(policy_file.read_text())
    policy=load_nominal_policy(policy_file,track_id='spielberg',
        map_sha256=raw['map_image_sha256'],data_dir=raceline/'data')
    entry=policy['candidates']['baseline']
    trajectory=load_racing_trajectory(raceline/'data'/entry['csv'],
                                      raceline/'data'/entry['metadata'])
    return policy, entry, trajectory


def test_offline_optimized_geometry_and_spatial_speed_are_registered():
    policy,entry,product=_registered()
    assert 'spielberg_r_a2_racing_trajectory' in entry['csv']
    assert product.metadata['generation_method']=='R_A0_MIN_CURVATURE_PLUS_R_A1_FOUNDATION_SPEED_ENVELOPE'
    assert product.metadata['sample_count']==864
    assert product.metadata['runtime_tracking_qualified'] is False
    speeds=[sample.target_speed_mps for sample in product.samples]
    assert max(speeds)>8.0 and max(speeds)-min(speeds)>3.0
    assert max(speeds)<=20.0+1e-9


def test_spielberg_nominal_loads_but_never_claims_tracking_qualification():
    policy,entry,product=_registered()
    active=select_nominal(product,expected_track_id='spielberg',
        expected_map_image_sha256=policy['map_image_sha256'],
        expected_source_trajectory_id=entry['source_trajectory_id'])
    assert active.active_reference_valid
    assert active.source_runtime_tracking_qualified is False


def test_wrong_map_hash_is_rejected():
    policy,entry,product=_registered()
    active=select_nominal(product,expected_track_id='spielberg',
        expected_map_image_sha256='0'*64)
    assert active.active_reference_valid is False


def test_centerline_preliminary_is_retained_but_not_selected():
    policy,entry,product=_registered()
    assert policy['default_candidate']=='baseline'
    assert 'centerline_preliminary' in policy['candidates']
    assert policy['candidates']['centerline_preliminary']['csv'] != entry['csv']
