import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import pytest

import resource_preflight as gate


def sample(runnable=2, psi=2.0, idle=.5):
    return gate.ResourceSample(8, idle, runnable, 2.0, psi, 8_000_000, 4_000_000)


def test_healthy_fixture_passes():
    assert gate.evaluate_resources([sample(), sample()])["status"] == "PASS"


def test_d230_pressure_fixture_blocks():
    result = gate.evaluate_resources([sample(runnable=28, psi=17.5)])
    assert result["status"] == "BLOCKED_INSUFFICIENT_HEADROOM"
    assert len(result["reasons"]) == 2


def test_unrelated_process_is_never_sent_to_cleanup(monkeypatch):
    calls = []
    assert gate.cleanup_stale_runtime([], killpg=lambda *args: calls.append(args)) == []
    assert calls == []


def test_ambiguous_manifest_blocks_without_cleanup(tmp_path, monkeypatch):
    path = tmp_path / "1.json"
    path.write_text(json.dumps({"pid": 1, "controller_pid": 2}))
    monkeypatch.setattr(gate, "process_identity", lambda pid: gate.ProcessIdentity(pid, 0, pid, 9, "unrelated"))
    classification, _, _ = gate.classify_owner_record(path, tmp_path)
    assert classification == "AMBIGUOUS"


def test_bounded_preflight_returns_without_retry(monkeypatch, tmp_path):
    monkeypatch.setattr(gate, "inspect_stale_runtime", lambda repository: ([], []))
    called = []
    with pytest.raises(RuntimeError, match="BLOCKED_INSUFFICIENT_HEADROOM"):
        gate.require_qualification_preflight(
            tmp_path, sample_function=lambda: called.append(True) or [sample(runnable=30, psi=40)],
        )
    assert called == [True]


def test_guarded_user_runtime_does_not_launch_when_resources_block(tmp_path):
    launched = []
    with pytest.raises(RuntimeError, match="BLOCKED_INSUFFICIENT_HEADROOM"):
        gate.run_guarded(
            ["never-launch"], tmp_path, 999, "fixture",
            preflight=lambda repository: (_ for _ in ()).throw(
                RuntimeError("BLOCKED_INSUFFICIENT_HEADROOM")
            ),
            popen=lambda *args, **kwargs: launched.append(args),
        )
    assert launched == []


def test_guarded_runtime_cleans_owned_group_after_parent_exit(monkeypatch, tmp_path):
    process = type("Process", (), {"pid": 42, "wait": lambda self: 0})()
    stopped = []
    monkeypatch.setattr(gate, "register_runtime_owner", lambda *args: tmp_path / "owner")
    monkeypatch.setattr(gate, "stop_owned_group", lambda pgid: stopped.append(pgid))
    monkeypatch.setattr(gate, "process_group_members", lambda pgid: [])
    monkeypatch.setattr(gate, "unregister_runtime_owner", lambda path: stopped.append(path))
    assert gate.run_guarded(
        ["runtime"], tmp_path, 1, "run", preflight=lambda repository: None,
        popen=lambda *args, **kwargs: process,
    ) == 0
    assert stopped == [42, tmp_path / "owner"]


def test_proven_owned_dummy_is_cleaned_without_collateral(tmp_path):
    process = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(60)",
         "ros2", "launch", "f1tenth_racing_bringup", "race_foundation.launch.py"],
        start_new_session=True,
    )
    path = gate.register_runtime_owner(process.pid, tmp_path, 999, "fixture", tmp_path / "owners")
    record = json.loads(path.read_text())
    record["controller_pid"] = 999999
    path.write_text(json.dumps(record))
    try:
        classification, identity, _ = gate.classify_owner_record(path, tmp_path)
        assert classification == "STALE_F1TENTH_MISSION_PROCESS"
        assert gate.cleanup_stale_runtime([(path, identity, "fixture")], timeout_s=2.0) == [process.pid]
        process.wait(timeout=2.0)
        assert gate.process_group_members(process.pid) == []
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=2.0)


def test_live_qualification_controller_is_blocked_not_cleaned(tmp_path, monkeypatch):
    launch = gate.ProcessIdentity(10, 9, 10, 100, "f1tenth_racing_bringup race_foundation.launch.py")
    controller = gate.ProcessIdentity(9, 1, 9, 90, "qualify_h2h.py")
    path = tmp_path / "10.json"
    path.write_text(json.dumps({
        **gate.asdict(launch), "controller_pid": 9, "controller_start_ticks": 90,
        "repository": str(tmp_path), "mission_id": gate.MISSION_ID,
    }))
    monkeypatch.setattr(gate, "process_identity", lambda pid: {9: controller, 10: launch}.get(pid))
    classification, identity, _ = gate.classify_owner_record(path, tmp_path)
    assert classification == "CURRENT_MISSION_PROCESS"
    assert identity == launch


def test_installed_user_path_has_the_same_owned_runtime_contract():
    assert gate.is_owned_launch_command("ros2 run f1tenth_racing_tools f1sim use_rviz:=false")


def test_qualifier_interlocks_before_runtime_launch():
    source = Path(__file__).with_name("qualify_h2h.py").read_text()
    assert source.index("require_qualification_preflight(") < source.index("subprocess.Popen(command")
    assert '"--preflight-only"' in source


def test_heavy_attribution_is_not_in_normal_qualifier_path():
    source = Path(__file__).with_name("qualify_h2h.py").read_text()
    assert "capture_host_attribution" not in source
