"""Spielberg R-A must bind exact map/centerline assets under config override."""
import copy
from pathlib import Path
import yaml
import pytest
from ament_index_python.packages import get_package_share_directory
from f1tenth_racing_raceline.foundation_adapter import load_problem


def test_qualified_source_inputs_resolve_registered_spielberg():
    sim=Path(get_package_share_directory('f1tenth_racing_sim'))
    p=load_problem(environment_path_override=sim/'config/spielberg_environment.yaml')
    assert p['environment']['environment_identity']['active_track']=='spielberg'
    assert len(p['corridor'].centerline)==864
    assert p['hashes']['map']==p['environment']['map']['image_sha256']
    assert p['hashes']['centerline']==p['environment']['map']['centerline_sha256']
    assert p['hashes']['map_metadata']==p['environment']['map']['metadata_sha256']


def test_corrupted_spielberg_map_identity_denied(tmp_path):
    sim=Path(get_package_share_directory('f1tenth_racing_sim'))
    env=yaml.safe_load((sim/'config/spielberg_environment.yaml').read_text())
    env['map']['image_sha256']='0'*64
    staged=tmp_path/'env.yaml'
    staged.write_text(yaml.safe_dump(env))
    with pytest.raises(ValueError,match='OFFLINE_RACELINE_MAP_SHA_MISMATCH'):
        load_problem(environment_path_override=staged)
