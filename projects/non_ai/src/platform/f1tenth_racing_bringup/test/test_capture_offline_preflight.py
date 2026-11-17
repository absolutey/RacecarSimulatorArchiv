"""ROS-registered regression for every Solo prelaunch offline gate.

Running these with the ordinary colcon test suite catches stale test stubs,
source hashes and Guard marker assumptions before a ROS process is launched.
These tests do not claim runtime actuation authority or vehicle safety.
"""
import hashlib
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[4]
PRESTART_SCRIPTS = (
    'TEST_UNIFIED_EXECUTION.py',
    'TEST_RUNTIME_CONTRACT_V2.py',
    'TEST_SCHED_INSTRUMENTATION.py',
    'TEST_P0_EXECUTION_PATH.py',
    'TEST_GUARD_TICK_BINDING.py',
    'TEST_OLD_VS_NEW_GUARD_CERT.py',
    'TEST_STOP_PROOF_EQUIVALENCE.py',
    'BENCHMARK_GUARD_REPLAY.py',
    'TEST_P0_ARCHITECTURE.py',
)


@pytest.mark.parametrize('script', PRESTART_SCRIPTS)
def test_prestart_script_success_before_launch(script):
    path = ROOT / script
    assert path.is_file(), script
    run = subprocess.run([sys.executable, '-B', str(path)], cwd=ROOT,
                         capture_output=True, text=True, timeout=60)
    assert run.returncode == 0, (script, run.stdout[-2000:], run.stderr[-2000:])


def test_stop_envelope_provenance_pin_equals_actual_model_bytes():
    sim = ROOT / 'src/environment/f1tenth_racing_sim/f1tenth_racing_sim'
    ctl = ROOT / 'src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control'
    stop_sha=hashlib.sha256((sim/'safety/stopping_envelope.py').read_bytes()).hexdigest()
    sweep_sha=hashlib.sha256((sim/'world/swept_collision.py').read_bytes()).hexdigest()
    adapter=(ctl/'fast_stop_proof.py').read_text()
    assert "PINNED_STOP_SOURCE_SHA256 = '" + stop_sha + "'" in adapter
    assert "PINNED_SWEEP_SOURCE_SHA256 = '" + sweep_sha + "'" in adapter
    assert 'class CertifiedFreeStoppingEnvelope(StoppingEnvelope)' in adapter
    assert '_verified_source_hash(_stop_source, PINNED_STOP_SOURCE_SHA256' in adapter
