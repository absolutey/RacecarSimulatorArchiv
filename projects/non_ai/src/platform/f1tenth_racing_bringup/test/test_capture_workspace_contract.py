"""Verify the isolated runtime-capture launcher cannot execute an unrelated stage."""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[4]
CAPTURE = ROOT / 'RUN_INTEGRATED_SOLO_CAPTURE.sh'
NATIVE = ROOT / 'RUN_NATIVE_SOLO_INTEGRATED.sh'


def test_capture_is_valid_bash():
    assert subprocess.run(['bash', '-n', str(CAPTURE)], check=False).returncode == 0


def test_stage_resolves_to_invoking_capture_script():
    source = CAPTURE.read_text()
    assert 'WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"' in source
    assert 'WS="$HOME/Downloads/F1TENTH_' not in source
    assert '"$WS/install/setup.bash"' in source


def test_package_overlay_identity_is_enforced():
    source = CAPTURE.read_text()
    assert '"$WS/install/"*' in source
    assert 'WRONG_OVERLAY:' in source
    assert 'LIVE_PRESTART_GATE_FAILED; START_NOT_SENT' in source
    assert 'PREFLIGHT_LIVE.py' in source


def test_preflight_failure_does_not_call_launch_cleanup():
    source = CAPTURE.read_text()
    preflight = source.split('python3 -B "$WS/PREFLIGHT_ARTIFACTS.py"', 1)[1].split('# Guardian remains enabled.', 1)[0]
    assert 'stop_launch' not in preflight.replace('do not call stop_launch', '')
    assert 'EVIDENCE_ZIP=' in preflight
    assert source.index('stop_launch() {') > source.index('PREFLIGHT_ARTIFACTS.py')


def test_native_launcher_dispatches_same_stage():
    source = NATIVE.read_text()
    assert 'cd "$(dirname "$0")"' in source
    assert 'exec bash RUN_INTEGRATED_SOLO_CAPTURE.sh' in source
    assert 'F1TENTH_EXPERIMENT_NATIVE_SOLO=1' in source
