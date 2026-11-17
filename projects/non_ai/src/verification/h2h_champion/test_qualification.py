import copy
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest


path = Path(__file__).with_name("qualify_h2h.py")
spec = importlib.util.spec_from_file_location("qualify_h2h", path)
qualify_h2h = importlib.util.module_from_spec(spec)
spec.loader.exec_module(qualify_h2h)


def valid_result():
    return {
        "launch_exit_code": 0, "residual_processes": 0, "laps": 5,
        "race_valid": True, "walls0": 0, "walls1": 0,
        "vehicle_collision": False, "override_observed": True,
        "late_join_override_observed": True, "truth_pass_count": 1,
        "nominal_return_observed": True,
        "plan_states": ["COMMITTED_PASS", "PASSING", "MERGE", "COMPLETE"],
        "peak_blue_speed_mps": 3.0,
    }


def test_resource_preflight_blocks_normal_path_before_launch(monkeypatch):
    monkeypatch.setattr(qualify_h2h, "require_current_product_install", lambda repository: {})
    monkeypatch.setattr(
        qualify_h2h, "require_qualification_preflight",
        lambda repository: (_ for _ in ()).throw(
            RuntimeError("RESOURCE_PREFLIGHT=BLOCKED_INSUFFICIENT_HEADROOM")
        ),
    )
    launched = []
    monkeypatch.setattr(qualify_h2h.subprocess, "Popen", lambda *args, **kwargs: launched.append(args))
    monkeypatch.setattr(sys, "argv", ["qualify_h2h.py", "--domain", "999"])
    with pytest.raises(SystemExit) as stopped:
        qualify_h2h.main()
    assert stopped.value.code == 2
    assert launched == []


def test_complete_protocol_is_required():
    assert qualify_h2h.qualification_failures(valid_result(), 5, 3.0) == []
    for key in ("race_valid", "nominal_return_observed"):
        result = copy.deepcopy(valid_result())
        result[key] = False
        assert qualify_h2h.qualification_failures(result, 5, 3.0)
    result = valid_result(); result["laps"] = 4
    assert "laps" in qualify_h2h.qualification_failures(result, 5, 3.0)
    result = valid_result(); result["plan_states"].remove("MERGE")
    assert "lifecycle" in qualify_h2h.qualification_failures(result, 5, 3.0)
    split = valid_result() | {"interaction_cap_mps": 4.0, "peak_blue_speed_mps": 4.5}
    assert qualify_h2h.qualification_failures(split, 5, 5.0) == []
    split["peak_blue_speed_mps"] = 4.0
    assert "speed_low" in qualify_h2h.qualification_failures(split, 5, 5.0)


def test_footprint_clearance_is_euclidean():
    square = ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
    shifted = tuple((x + 3.0, y + 4.0) for x, y in square)
    assert qualify_h2h.footprint_clearance(square, shifted) == 5.0 ** 0.5
    assert qualify_h2h.footprint_clearance(square, square) == 0.0


def test_footprint_uses_foundation_length_and_width_independently():
    assert qualify_h2h.footprint(0.0, 0.0, 0.0, 0.2, 0.1) == (
        (-0.2, -0.1), (0.2, -0.1), (0.2, 0.1), (-0.2, 0.1),
    )


def test_runtime_waits_for_merge_complete_and_nominal_release():
    plan = lambda revision, state: ("PLAN", revision, state)
    source = lambda kind: ("REFERENCE", kind)
    assert not qualify_h2h.runtime_complete(5, 5, [
        plan(1, "COMMITTED_PASS"), plan(1, "PASSING"),
        plan(1, "COMPLETE"), source("NOMINAL"),
    ])
    assert not qualify_h2h.runtime_complete(5, 5, [
        plan(1, "COMMITTED_PASS"), plan(1, "PASSING"),
        plan(2, "MERGE"), plan(1, "COMPLETE"), source("NOMINAL"),
    ])
    assert not qualify_h2h.runtime_complete(5, 5, [
        plan(1, "COMMITTED_PASS"), plan(1, "PASSING"),
        plan(1, "MERGE"), source("NOMINAL"), plan(1, "COMPLETE"),
    ])
    assert qualify_h2h.runtime_complete(5, 5, [
        plan(1, "COMMITTED_PASS"), plan(1, "PASSING"),
        plan(1, "MERGE"), plan(1, "COMPLETE"), source("NOMINAL"),
    ])
    assert qualify_h2h.runtime_complete(5, 5, [
        plan(1, "COMMITTED_PASS"), plan(1, "PASSING"),
        plan(1, "ABORT"), plan(1, "COMPLETE"), source("NOMINAL"),
    ])


def test_stale_runtime_owner_install_is_rejected(tmp_path):
    source = tmp_path / "source.py"
    source.write_text("current")
    installed = tmp_path / "installed_core.py"
    installed.write_text("stale")

    try:
        qualify_h2h.require_matching_install(
            tmp_path, "Foundation", "source.py", SimpleNamespace(__file__=str(installed)),
        )
    except RuntimeError as error:
        assert "stale installed Foundation" in str(error)
    else:
        raise AssertionError("stale installed runtime owner was accepted")


def test_stale_installed_profile_input_is_rejected(tmp_path):
    source = tmp_path / "source.yaml"
    installed = tmp_path / "installed.yaml"
    source.write_text("current")
    installed.write_text("stale")

    try:
        qualify_h2h.require_matching_installed_file(
            tmp_path, "R-E", "source.yaml", installed,
        )
    except RuntimeError as error:
        assert "stale installed R-E" in str(error)
    else:
        raise AssertionError("stale installed profile input was accepted")


def test_runtime_inventory_includes_transitive_product_inputs():
    repository = path.parents[3]
    inventory = {str(source.relative_to(repository))
                 for _, source, _, _, _ in qualify_h2h.runtime_source_inventory(repository)}
    required = {
        "src/driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/mpc.py",
        "src/driving/algorithms/f1tenth_racing_control/config/r_b3_mpc.yaml",
        "src/platform/f1tenth_racing_geometry/f1tenth_racing_geometry/closed_track.py",
        "src/platform/f1tenth_racing_reference/f1tenth_racing_reference/transport.py",
        "src/driving/f1tenth_racing_opponent/f1tenth_racing_opponent/controller.py",
        "src/platform/f1tenth_racing_evaluator/f1tenth_racing_evaluator/evaluator.py",
    }
    assert required <= inventory


def test_generated_runtime_contracts_match_current_source():
    repository = path.parents[3]
    scripts = qualify_h2h.source_console_scripts(
        repository / "src/driving/algorithms/f1tenth_racing_control/setup.py"
    )
    assert scripts == {"r_b0_controller = f1tenth_racing_control.node:main"}
    assert qualify_h2h.generated_control_message_files(repository)


def test_stale_generated_message_binding_is_rejected(monkeypatch):
    monkeypatch.setattr(
        qualify_h2h.control_messages, "CarState",
        SimpleNamespace(get_fields_and_field_types=lambda: {"px": "double"}),
    )
    try:
        qualify_h2h.generated_control_message_files(path.parents[3])
    except RuntimeError as error:
        assert "stale generated control_msgs binding" in str(error)
    else:
        raise AssertionError("stale generated message binding was accepted")


def test_generated_entrypoint_wrapper_must_dispatch_declared_target(tmp_path):
    wrapper = tmp_path / "node"
    wrapper.write_text(
        "# expected-package console_scripts node\n"
        "load_entry_point('other-package', 'console_scripts', 'node')\n"
    )
    assert not qualify_h2h.entrypoint_wrapper_coherent(wrapper, "expected-package", "node")
    wrapper.write_text("load_entry_point('expected-package', 'console_scripts', 'node')")
    assert qualify_h2h.entrypoint_wrapper_coherent(wrapper, "expected-package", "node")
    wrapper.write_text("load_entry_point('expected-package==0.1.0', 'console_scripts', 'node')")
    assert qualify_h2h.entrypoint_wrapper_coherent(wrapper, "expected-package", "node")


def test_f1solo_rejects_profile_overrides_and_allows_rviz(tmp_path):
    script = path.parents[2] / "platform/f1tenth_racing_tools/scripts/f1solo"
    rejected = subprocess.run(
        [script, "speed_cap_mps:=1.0"], capture_output=True, text=True, check=False,
    )
    assert rejected.returncode == 2
    assert "unsupported Champion profile override" in rejected.stderr

    ros2 = tmp_path / "ros2"
    ros2.write_text("#!/usr/bin/env bash\nexit 0\n")
    ros2.chmod(0o755)
    environment = dict(os.environ, PATH=f"{tmp_path}:{os.environ['PATH']}")
    allowed = subprocess.run(
        [script, "use_rviz:=false"], env=environment, capture_output=True, text=True,
        check=False,
    )
    assert allowed.returncode == 0


def test_normal_user_path_preserves_r_b_ceiling_without_legacy_overtaking():
    repository = path.parents[3]
    script = repository / "src/platform/f1tenth_racing_tools/scripts/f1sim"
    text = script.read_text()
    assert "speed_cap_mps:=5.0" in text
    assert "interaction_speed_cap_mps" not in text
    assert "use_opponent_perception:=true" in text
    assert "use_opponent_prediction:=true" in text
    assert "use_overtaking:=false" in text
    assert "use_overtaking:=true" not in text
    assert "use_flight_recorder:=true" in text
    assert "record_rosbag:=false" in text
    assert "use_spectator_hud:=true" in text
    assert "resource_preflight.py" in text
    assert "speed_cap_mps:=3.0" not in text


def test_f1fast_alias_and_safe_profile_preserve_legacy_quarantine():
    repository = path.parents[3]
    fast = (repository / "src/platform/f1tenth_racing_tools/scripts/f1fast").read_text()
    safe = (repository / "src/platform/f1tenth_racing_tools/scripts/f1safe").read_text()
    assert '$(dirname "$0")/f1sim' in fast
    assert "speed_cap_mps:=3.0" in safe
    assert "interaction_speed_cap_mps" not in safe
    assert "use_overtaking:=false" in safe
    assert "use_overtaking:=true" not in safe
    assert "resource_preflight.py" in safe


def test_f1sim_resource_block_cannot_reach_ros_launch(tmp_path):
    repository = path.parents[3]
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    simulator = scripts / "f1sim"
    shutil.copy2(repository / "src/platform/f1tenth_racing_tools/scripts/f1sim", simulator)
    (scripts / "resource_preflight.py").write_text("raise SystemExit(2)\n")
    launched = tmp_path / "launched"
    ros2 = tmp_path / "ros2"
    ros2.write_text(f"#!/usr/bin/env bash\ntouch {launched}\n")
    ros2.chmod(0o755)
    environment = dict(os.environ, PATH=f"{tmp_path}:{os.environ['PATH']}")
    result = subprocess.run([simulator, "use_rviz:=false"], env=environment, check=False)
    assert result.returncode == 2
    assert not launched.exists()


def test_hud_and_recorder_are_read_only_product_consumers():
    repository = path.parents[3]
    hud = (repository / "src/platform/f1tenth_racing_telemetry/f1tenth_racing_telemetry/hud.py").read_text()
    recorder = (repository / "src/platform/f1tenth_racing_telemetry/f1tenth_racing_telemetry/recorder.py").read_text()
    assert 'create_publisher(MarkerArray, "spectator_hud"' in hud
    assert "create_publisher" not in recorder
    for forbidden in ("ackermann_cmd0", "active_target_trajectory"):
        assert f'create_publisher({forbidden}' not in hud


def test_startup_wait_fails_closed_on_launch_exit_or_deadline():
    check = qualify_h2h.startup_wait_failure
    assert check(None, 9.9, 10.0) is None
    assert check(1, 9.9, 10.0) == "launch exited before start: 1"
    assert check(None, 10.0, 10.0) == "race start service timed out"


def test_physical_pass_state_requires_behind_to_ahead_crossing():
    class Geometry:
        @staticmethod
        def project(x, _y):
            return SimpleNamespace(s=x)

        @staticmethod
        def signed_delta_s(red_s, blue_s):
            return blue_s - red_s

    geometry = Geometry()
    pose = lambda s: (0.0, s, 0.0, 0.0)
    was_behind, passed = qualify_h2h.update_pass_state(
        geometry, False, pose(0.0), pose(2.0),
    )
    assert was_behind and not passed
    was_behind, passed = qualify_h2h.update_pass_state(
        geometry, was_behind, pose(2.0), pose(0.0),
    )
    assert not was_behind and passed
