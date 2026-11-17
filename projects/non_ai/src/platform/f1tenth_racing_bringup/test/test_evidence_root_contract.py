"""Prevent runtime evidence from polluting the signed ROS source release."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
FINAL = ROOT / 'RUN_FINAL_INTEGRATED_ROS_VERIFY.sh'
STRUCTURAL = ROOT / 'RUN_STRUCTURAL_IMPLEMENTATION_ROS_VERIFY.sh'
PROVENANCE = ROOT / 'RUN_STRUCTURAL_PROVENANCE_REPAIR_ROS_VERIFY.sh'


def test_release_verifier_is_strict_not_relaxed():
    src = (ROOT / 'VERIFY_RELEASE.py').read_text()
    assert 'UNEXPECTED_OR_MISSING_FILES' in src
    assert 'RELEASE_HASH_MISMATCH' in src


def test_final_evidence_defaults_outside_the_source_root():
    src = FINAL.read_text()
    assert '$ROOT/../EVIDENCE_FINAL_INTEGRATED' in src
    assert 'EVIDENCE_ROOT_INSIDE_RELEASE_TREE' in src
    assert 'realpath -m' in src
    assert '$PWD/EVIDENCE_FINAL_INTEGRATED' not in src


def test_requested_in_tree_evidence_path_is_denied_before_writing():
    # Must finish before ROS setup or build; code 12 is the explicit guard.
    env = __import__('os').environ.copy()
    path = ROOT / 'EVIDENCE_TEST_DO_NOT_CREATE'
    assert not path.exists()
    env['EVIDENCE_ROOT'] = str(path)
    result = subprocess.run(['bash', str(FINAL)], cwd=ROOT, env=env,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 12, result.stderr
    assert 'BLOCKED=EVIDENCE_ROOT_INSIDE_RELEASE_TREE' in result.stderr
    assert not path.exists()


def test_other_ros_verifiers_keep_evidence_outside_the_tree():
    s = STRUCTURAL.read_text()
    p = PROVENANCE.read_text()
    assert '$ROOT/../EVIDENCE_STRUCTURAL_IMPLEMENTATION' in s
    assert 'EVIDENCE_ROOT_INSIDE_RELEASE_TREE' in s
    assert '$PWD/../EVIDENCE_STRUCTURAL_PROVENANCE_REPAIR' in p
    assert '$PWD/EVIDENCE_STRUCTURAL_PROVENANCE_REPAIR}' not in p


def test_release_integrity_still_passes_when_evidence_is_external():
    # Testing the exact first stage of the Ubuntu verifier without starting ROS.
    with tempfile.TemporaryDirectory(prefix='rab_release_evidence_') as d:
        p = Path(d) / 'RELEASE_HASHES.log'
        with p.open('w') as out:
            r = subprocess.run(['python3', '-B', str(ROOT / 'VERIFY_RELEASE.py')],
                               cwd=ROOT, stdout=out, stderr=subprocess.STDOUT,
                               timeout=15)
        assert r.returncode == 0, p.read_text()[-700:]
        assert 'INTEGRATION_RELEASE_HASH_PASS' in p.read_text()
