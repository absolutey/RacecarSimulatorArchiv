"""Isolated stage regressions: allow nested source, reject foreign overlays.

These are artifact *admission* tests, not race performance or Guard safety proofs.
"""
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

# Keep these contract tests runnable without a ROS/ament import environment.
# Load the actual production module from its source file, not a test copy.
_contract_file = (Path(__file__).resolve().parents[1] /
                  'f1tenth_racing_reference/contract_guard.py')
_spec = importlib.util.spec_from_file_location('stage_provenance_under_test', _contract_file)
guard = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(guard)


STAGE_NAME = 'F1TENTH_RAB_UNIFIED_CONSOLIDATED_IMPLEMENTATION_20261018'
SOURCE_MODULE = ('src/platform/f1tenth_racing_reference/'
                 'f1tenth_racing_reference/contract_guard.py')
SOURCE_DATA = 'src/driving/algorithms/f1tenth_racing_raceline/data'
INSTALLED_DATA = ('install/f1tenth_racing_raceline/share/'
                  'f1tenth_racing_raceline/data')


def build_fixture(tmp_path, monkeypatch, parents=('RAB_STRUCTURAL_VERIFY_ABC',)):
    home = tmp_path / 'user'
    stage = home / 'Downloads' / Path(*parents) / STAGE_NAME if parents else home / 'Downloads' / STAGE_NAME
    module = stage / SOURCE_MODULE
    module.parent.mkdir(parents=True)
    module.write_text('# module identity fixture\n')
    source_data = stage / SOURCE_DATA
    source_data.mkdir(parents=True)
    installed_data = stage / INSTALLED_DATA
    installed_data.mkdir(parents=True)
    csv = source_data / 'candidate.csv'
    csv.write_text('s,x,y,v\n0,0,0,6\n')
    metadata = source_data / 'candidate.json'
    metadata.write_text(json.dumps({
        'trajectory_id': 'candidate-1',
        'csv_sha256': hashlib.sha256(csv.read_bytes()).hexdigest(),
    }))
    (installed_data / csv.name).symlink_to(csv)
    (installed_data / metadata.name).symlink_to(metadata)
    policy_path = stage / 'policy.json'
    policy_path.write_text(json.dumps({
        'track_id': 'spielberg',
        'map_image_sha256': 'a' * 64,
        'frame_id': 'map',
        'candidates': {'baseline': {
            'csv': csv.name, 'metadata': metadata.name,
            'source_trajectory_id': 'candidate-1',
        }},
    }))
    monkeypatch.setattr(guard.Path, 'home', classmethod(lambda cls: home))
    monkeypatch.setattr(guard, '__file__', str(module))
    return home, stage, module, source_data, installed_data, policy_path


@pytest.mark.parametrize('parents', [(), ('RAB_STRUCTURAL_VERIFY_ABC',),
                                     ('isolated', 'another', 'test')])
def test_installed_registry_from_same_executing_nested_stage(tmp_path, monkeypatch, parents):
    _, stage, module, source, installed, policy_path = build_fixture(tmp_path, monkeypatch, parents)
    assert guard._trusted_stage_provenance(stage, module)
    policy = guard.load_nominal_policy(policy_path, track_id='spielberg',
                                      map_sha256='a'*64, data_dir=installed)
    assert policy['candidates']['baseline']['source_trajectory_id'] == 'candidate-1'


def test_permitted_canonical_next_workspace(tmp_path, monkeypatch):
    home = tmp_path / 'user'
    stage = home / 'F1TENTH_NEXT_WORKSPACE'
    module = stage / SOURCE_MODULE
    module.parent.mkdir(parents=True)
    module.write_text('module\n')
    monkeypatch.setattr(guard.Path, 'home', classmethod(lambda cls: home))
    assert guard._trusted_stage_provenance(stage, module)


def test_cross_stage_module_does_not_approve_other_registry(tmp_path, monkeypatch):
    home, stage, module, source, installed, policy = build_fixture(tmp_path, monkeypatch)
    other = home / 'Downloads' / 'isolate' / STAGE_NAME / SOURCE_MODULE
    other.parent.mkdir(parents=True)
    other.write_text('different module\n')
    monkeypatch.setattr(guard, '__file__', str(other))
    with pytest.raises(ValueError, match='executing verified stage'):
        guard.load_nominal_policy(policy, track_id='spielberg', map_sha256='a'*64, data_dir=installed)


def test_external_lookalike_workspace_rejected(tmp_path, monkeypatch):
    _, stage, module, _, installed, policy = build_fixture(tmp_path, monkeypatch)
    outside = tmp_path / STAGE_NAME
    external_module = outside / SOURCE_MODULE
    external_module.parent.mkdir(parents=True)
    external_module.write_text('external\n')
    assert not guard._trusted_stage_provenance(outside, external_module)


def test_stage_directory_symlink_rejected(tmp_path, monkeypatch):
    home, stage, module, _, _, _ = build_fixture(tmp_path, monkeypatch)
    stage_link = home / 'Downloads' / 'other' / STAGE_NAME
    stage_link.parent.mkdir(parents=True)
    stage_link.symlink_to(stage, target_is_directory=True)
    assert not guard._trusted_stage_provenance(stage_link, module)


def test_parent_directory_symlink_rejected(tmp_path, monkeypatch):
    home, stage, module, _, _, _ = build_fixture(tmp_path, monkeypatch)
    link_dir = home / 'Downloads' / 'symlink_parent'
    link_dir.symlink_to(stage.parent, target_is_directory=True)
    symlink_stage = link_dir / STAGE_NAME
    assert not guard._trusted_stage_provenance(symlink_stage, module)


def test_foreign_symlinked_candidate_file_denied(tmp_path, monkeypatch):
    _, stage, module, source, installed, policy = build_fixture(tmp_path, monkeypatch)
    foreign = tmp_path / 'foreign.csv'
    foreign.write_text('s,x,y,v\n0,0,0,100\n')
    (installed / 'candidate.csv').unlink()
    (installed / 'candidate.csv').symlink_to(foreign)
    with pytest.raises(ValueError, match='escaped registry'):
        guard.load_nominal_policy(policy, track_id='spielberg', map_sha256='a'*64, data_dir=installed)


def test_installed_registry_symlinked_directory_to_same_source_accepted(tmp_path, monkeypatch):
    _, stage, _, source, installed, policy = build_fixture(tmp_path, monkeypatch)
    for child in installed.iterdir():
        child.unlink()
    installed.rmdir()
    installed.symlink_to(source, target_is_directory=True)
    result = guard.load_nominal_policy(policy, track_id='spielberg',
                                       map_sha256='a'*64, data_dir=installed)
    assert result['track_id'] == 'spielberg'


def test_installed_registry_symlinked_directory_to_foreign_denied(tmp_path, monkeypatch):
    _, _, _, source, installed, policy = build_fixture(tmp_path, monkeypatch)
    foreign = tmp_path / 'foreign_dir'
    foreign.mkdir()
    (foreign / 'candidate.csv').write_bytes((source / 'candidate.csv').read_bytes())
    (foreign / 'candidate.json').write_bytes((source / 'candidate.json').read_bytes())
    for child in installed.iterdir():
        child.unlink()
    installed.rmdir()
    installed.symlink_to(foreign, target_is_directory=True)
    with pytest.raises(ValueError, match='executing verified stage'):
        guard.load_nominal_policy(policy, track_id='spielberg', map_sha256='a'*64, data_dir=installed)


def test_wrong_artifact_content_hash_denied(tmp_path, monkeypatch):
    _, stage, _, source, installed, policy = build_fixture(tmp_path, monkeypatch)
    (source / 'candidate.csv').write_text('tampered data\n')
    with pytest.raises(ValueError, match='fingerprint mismatch'):
        guard.load_nominal_policy(policy, track_id='spielberg', map_sha256='a'*64, data_dir=installed)


def test_wrong_map_hash_denied(tmp_path, monkeypatch):
    _, stage, _, source, installed, policy = build_fixture(tmp_path, monkeypatch)
    with pytest.raises(ValueError, match='map identity mismatch'):
        guard.load_nominal_policy(policy, track_id='spielberg', map_sha256='b'*64, data_dir=installed)


def test_candidate_name_traversal_denied(tmp_path, monkeypatch):
    _, stage, _, source, installed, policy = build_fixture(tmp_path, monkeypatch)
    raw = json.loads(policy.read_text())
    raw['candidates']['baseline']['csv'] = '../candidate.csv'
    policy.write_text(json.dumps(raw))
    with pytest.raises(ValueError, match='artifact basename'):
        guard.load_nominal_policy(policy, track_id='spielberg', map_sha256='a'*64, data_dir=installed)


def test_foreign_named_stage_rejected(tmp_path, monkeypatch):
    home, stage, module, _, _, _ = build_fixture(tmp_path, monkeypatch)
    renamed = home / 'Downloads' / 'RAB_STRUCTURAL_VERIFY_ABC' / 'UNKNOWN_STAGE'
    assert not guard._permitted_stage_location(renamed, home)


def test_real_spielberg_policy_and_both_registered_artifacts(tmp_path, monkeypatch):
    """All trusted Spielberg profiles retain their provenance and baseline."""
    _, stage, _, source, installed, policy = build_fixture(tmp_path, monkeypatch)
    repo = Path(__file__).resolve().parents[4]
    real_data = (repo / 'src/driving/algorithms/f1tenth_racing_raceline/data')
    real_policy = (repo / 'src/platform/f1tenth_racing_reference/config/'
                   'spielberg_nominal_reference.json')
    raw = json.loads(real_policy.read_text())
    policy.write_bytes(real_policy.read_bytes())
    for artifact in raw['candidates'].values():
        for kind in ('csv', 'metadata'):
            name = artifact[kind]
            (source / name).write_bytes((real_data / name).read_bytes())
            (installed / name).symlink_to(source / name)
    accepted = guard.load_nominal_policy(policy, track_id='spielberg',
                                         map_sha256=raw['map_image_sha256'],
                                         data_dir=installed)
    assert accepted['default_candidate'] == 'baseline'
    # PASS1 added two R-A3 candidates. Requiring exactly two candidates
    # rejects valid registrations; requiring merely >=2 would miss removals.
    assert set(accepted['candidates']) == {
        'baseline', 'centerline_preliminary', 'scp',
        'scp_true_joint_experimental',
    }
    assert set(accepted['candidates']) == set(raw['candidates'])
    assert accepted['candidates']['baseline']['csv'] == 'spielberg_r_a2_racing_trajectory.csv'
    for name, registered in accepted['candidates'].items():
        expected = raw['candidates'][name]
        assert registered['csv'] == expected['csv']
        assert registered['metadata'] == expected['metadata']
        assert registered['source_trajectory_id'] == expected['source_trajectory_id']
        assert registered['source_trajectory_id'].startswith('sha256:')
        assert json.loads((source / registered['metadata']).read_text())['sample_count'] == 864


def test_real_spielberg_tampered_artifact_is_rejected(tmp_path, monkeypatch):
    _, _, _, source, installed, policy = build_fixture(tmp_path, monkeypatch)
    repo = Path(__file__).resolve().parents[4]
    real_data = repo / 'src/driving/algorithms/f1tenth_racing_raceline/data'
    real_policy = repo / 'src/platform/f1tenth_racing_reference/config/spielberg_nominal_reference.json'
    raw = json.loads(real_policy.read_text())
    policy.write_bytes(real_policy.read_bytes())
    for artifact in raw['candidates'].values():
        for kind in ('csv', 'metadata'):
            name = artifact[kind]
            (source / name).write_bytes((real_data / name).read_bytes())
            (installed / name).symlink_to(source / name)
    with (source / raw['candidates']['baseline']['csv']).open('ab') as f:
        f.write(b'\nCORRUPTED\n')
    with pytest.raises(ValueError, match='fingerprint mismatch'):
        guard.load_nominal_policy(policy, track_id='spielberg',
                                  map_sha256=raw['map_image_sha256'],
                                  data_dir=installed)
