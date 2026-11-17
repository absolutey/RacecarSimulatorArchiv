"""Hermetic tests: do not need ROS packages."""
import importlib.util
from pathlib import Path
import pytest

path=(Path(__file__).resolve().parents[1]/'src/platform/f1tenth_racing_reference/'
      'f1tenth_racing_reference/contract_guard.py')
spec=importlib.util.spec_from_file_location('tested_contract_guard',path)
cg=importlib.util.module_from_spec(spec)
spec.loader.exec_module(cg)


def _stage(tmp_path, name):
    root=tmp_path/'Downloads';root.mkdir(exist_ok=True)
    stage=root/name
    module=stage/'src/platform/f1tenth_racing_reference/f1tenth_racing_reference/contract_guard.py'
    module.parent.mkdir(parents=True)
    module.write_text('module origin')
    return stage,module


def test_source_stage_identity_is_exact(tmp_path,monkeypatch):
    monkeypatch.setenv('HOME',str(tmp_path))
    stage,module=_stage(tmp_path,'F1TENTH_RAB_TEST_STAGE')
    assert cg._trusted_stage_provenance(stage,module)
    foreign,foreign_module=_stage(tmp_path,'F1TENTH_RAB_FOREIGN_STAGE')
    assert not cg._trusted_stage_provenance(foreign,module)
    assert not cg._trusted_stage_provenance(stage,foreign_module)
    bad,bad_module=_stage(tmp_path,'UNRELATED_STAGE')
    assert not cg._trusted_stage_provenance(bad,bad_module)
    link=tmp_path/'Downloads/F1TENTH_RAB_LINK_STAGE'
    link.symlink_to(stage,target_is_directory=True)
    assert not cg._trusted_stage_provenance(link,module)


def test_installed_artifact_only_same_stage(tmp_path,monkeypatch):
    monkeypatch.setenv('HOME',str(tmp_path))
    stage,module=_stage(tmp_path,'F1TENTH_RAB_VERIFIED')
    source=stage/'src/driving/algorithms/f1tenth_racing_raceline/data'
    source.mkdir(parents=True)
    (source/'baseline.csv').write_text('s,v\n0,1\n')
    installed=stage/'install/f1tenth_racing_raceline/share/f1tenth_racing_raceline/data'
    installed.mkdir(parents=True)
    (installed/'baseline.csv').symlink_to(source/'baseline.csv')
    monkeypatch.setattr(cg,'__file__',str(module))
    assert cg._registered_artifact(installed,{'csv':'baseline.csv'},'csv') == installed/'baseline.csv'
    foreign,_=_stage(tmp_path,'F1TENTH_RAB_OTHER')
    other=foreign/'src/driving/algorithms/f1tenth_racing_raceline/data'
    other.mkdir(parents=True)
    (other/'baseline.csv').write_text('other')
    (installed/'baseline.csv').unlink()
    (installed/'baseline.csv').symlink_to(other/'baseline.csv')
    with pytest.raises(ValueError,match='escaped registry'):
        cg._registered_artifact(installed,{'csv':'baseline.csv'},'csv')
    other_installed=foreign/'install/f1tenth_racing_raceline/share/f1tenth_racing_raceline/data'
    other_installed.mkdir(parents=True)
    (other_installed/'baseline.csv').symlink_to(other/'baseline.csv')
    with pytest.raises(ValueError,match='executing verified stage'):
        cg._registered_artifact(other_installed,{'csv':'baseline.csv'},'csv')


def test_traversal_basename_rejected(tmp_path):
    for basename in ('../secret.csv','other/path.csv','',r'..\secret.csv'):
        with pytest.raises(ValueError):
            cg._registered_artifact(tmp_path,{'csv':basename},'csv')
