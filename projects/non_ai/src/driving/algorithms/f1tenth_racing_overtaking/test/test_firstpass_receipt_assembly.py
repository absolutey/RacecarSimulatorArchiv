"""G0-G9 assembler cannot invent missing metrics or mix runs/maps."""
import importlib.util
from pathlib import Path
import json
import pytest
ROOT=Path(__file__).resolve().parents[5]
spec=importlib.util.spec_from_file_location('firstpass_gates',ROOT/'BUILD_FIRSTPASS_G0_G9_RECEIPTS.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
SHA='a'*64


def test_missing_evidence_remains_missing_and_does_not_promote(tmp_path):
    assert m.assemble(tmp_path,run_id='r1',source_sha256=SHA)==[]
    assessment=m.inspect_receipts([],tmp_path,run_id='r1',source_tree_sha256=SHA)
    assert not assessment['evidence_complete'] and not assessment['product_promoted']
    assert assessment['decisions']['G8/spielberg']=='MISSING'


def test_wrong_run_cannot_be_rebound(tmp_path):
    (tmp_path/'G6_iccas2025.json').write_text(json.dumps(dict(
        schema='RAB_GATE_EVIDENCE_V2',gate='G6',scope='iccas2025',
        source_tree_sha256=SHA,run_id='other',metrics={})))
    with pytest.raises(ValueError,match='IDENTITY_INVALID'):
        m.assemble(tmp_path,run_id='r1',source_sha256=SHA)


def test_no_ambiguous_relative_symlink(tmp_path):
    (tmp_path/'original.json').write_text('{}')
    (tmp_path/'G0_GLOBAL.json').symlink_to(tmp_path/'original.json')
    with pytest.raises(ValueError,match='SYMLINK_FORBIDDEN'):
        m.assemble(tmp_path,run_id='r1',source_sha256=SHA)
