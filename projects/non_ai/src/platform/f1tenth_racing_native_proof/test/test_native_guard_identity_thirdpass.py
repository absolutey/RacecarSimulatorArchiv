"""Check the selected SOLO C++ Guard's actual ROS publisher identity contract."""
from pathlib import Path
import re

SRC = Path(__file__).resolve().parents[1]
WS = SRC.parents[1]
CPP = (SRC/'src/native_guard_ros.cpp').read_text()
LAUNCH = (WS/'platform/f1tenth_racing_bringup/launch/race_foundation.launch.py').read_text()
SIM = (WS/'environment/f1tenth_racing_sim/f1tenth_racing_sim/simulator.py').read_text()
VETO = (WS/'driving/algorithms/f1tenth_racing_control/f1tenth_racing_control/independent_watchdog_node.py').read_text()
COORD = (WS/'platform/f1tenth_racing_bringup/scripts/race_coordinator.py').read_text()


def test_native_guard_ros_name_matches_all_four_independent_authorities():
    # The two selectable executables share ONE identity, never two publishers.
    assert 'explicit NativeGuard():Node("actuator_guard0")' in CPP
    assert re.search(r'Node\(package="f1tenth_racing_native_proof", executable="native_actuator_guard",\s*name="actuator_guard0"', LAUNCH)
    assert "node_name!='actuator_guard0'" in SIM
    assert "node_name!='actuator_guard0'" in VETO
    assert 'node_name!="actuator_guard0"' in COORD


def test_launch_guards_are_mutually_exclusive_and_cpp_h2h_is_rejected():
    assert 'condition=UnlessCondition(LaunchConfiguration("use_native_guard_ros"))' in LAUNCH
    assert 'condition=IfCondition(LaunchConfiguration("use_native_guard_ros"))' in LAUNCH
    assert 'NATIVE_CPP_GUARD_H2H_NOT_QUALIFIED' in LAUNCH
    assert 'name="actuator_guard0", output="screen"' in LAUNCH


def test_cpp_lifetime_and_safety_budget_are_unchanged():
    for required in ('f1_evaluate_native_stop(', 'deadline=begun+15000000',
                     'latched_fault_', 'stop_certificate_verified=true',
                     'MINIMUM_RISK_BRAKE', 'on_collision_', 'on_vehicle_'):
        assert required in CPP, required
    assert 'dynamic_red_guard' not in CPP  # SOLO-only intentionally
