from pathlib import Path

import yaml


def displays(profile="h2h_spectator.rviz"):
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "rviz" / profile).read_text()
    )
    return {item["Name"]: item for item in config["Visualization Manager"]["Displays"]}


def test_static_visuals_use_transient_local_reliable_qos():
    configured = displays()
    for name, topic in (("Competition Map", "/map"), ("Centerline", "/centerline")):
        assert configured[name]["Topic"]["Value"] == topic
        assert configured[name]["Topic"]["Durability Policy"] == "Transient Local"
        assert configured[name]["Topic"]["Reliability Policy"] == "Reliable"


def test_spectator_profile_contains_vehicles_hud_and_active_path_display():
    configured = displays()
    display = "Race HUD + Active Path"
    assert {"Competition Map", "Centerline", "Blue", "Red", display} <= configured.keys()
    assert configured[display]["Topic"]["Value"] == "/spectator_hud"
    assert not any(item["Class"] == "rviz_default_plugins/LaserScan"
                   for item in configured.values())


def test_foundation_profile_exposes_latched_world_without_raw_lidar():
    configured = displays("foundation.rviz")
    for name, topic in (("Competition Map", "/map"), ("Centerline", "/centerline")):
        assert configured[name]["Topic"]["Value"] == topic
        assert configured[name]["Topic"]["Durability Policy"] == "Transient Local"
        assert configured[name]["Topic"]["Reliability Policy"] == "Reliable"
    assert not any(item["Class"] == "rviz_default_plugins/LaserScan"
                   for item in configured.values())
    assert {"Blue car0", "Red car1"} <= configured.keys()


def test_launch_wires_phase_into_hud_and_keeps_legacy_overtaking_off_by_default():
    launch = (Path(__file__).parents[1] / "launch/race_foundation.launch.py").read_text()
    assert 'DeclareLaunchArgument("review_phase", default_value="FOUNDATION")' in launch
    assert '"solo": LaunchConfiguration("solo")' in launch
    assert '"execution_speed_cap_mps": speed_cap' in launch
    assert 'DeclareLaunchArgument("sim_online_adaptive", default_value="true")' in launch
    assert 'DeclareLaunchArgument("use_overtaking", default_value="false")' in launch
    assert 'OrSubstitution(\n            LaunchConfiguration("use_flight_recorder"),\n            LaunchConfiguration("use_spectator_hud"),' in launch
    assert '"review_phase": LaunchConfiguration("review_phase")' in launch



def test_launch_defaults_h2h_overtaking_to_successor():
    from pathlib import Path
    launch = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "race_foundation.launch.py"
    ).read_text()
    assert (
        'DeclareLaunchArgument("overtaking_executable", '
        'default_value="overtaking_successor")'
    ) in launch
    assert 'default_value="overtaking_planner"' not in launch
