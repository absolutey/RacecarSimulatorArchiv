"""ROS Humble Guard API contract; never substitutes for C++ compilation."""
from pathlib import Path
import re
HERE=Path(__file__).resolve().parents[1]
CODE=(HERE/'src/native_guard_ros.cpp').read_text()

def test_humble_clock_getter_does_not_invoke_mutating_now_on_const_clock():
    assert 'int64_t clock_ns(){return get_clock()->now().nanoseconds();}' in CODE
    assert 'int64_t clock_ns()const' not in CODE
    assert 'bool recent_state(){' in CODE

def test_humble_builtin_stamp_conversion_not_python_time_api():
    assert CODE.count('static_cast<ROSStamp>(get_clock()->now())')==3
    assert 'get_clock()->now().to_msg()' not in CODE

def test_independent_guard_authority_and_deadline_unchanged():
    for key in ('const auto begun=steady_ns(),deadline=begun+15000000;',
                'f1_evaluate_native_stop(', 'stop_certificate_verified=true',
                'NATIVE_PRE_PUBLISH_LEASE_EXPIRED',
                'if(!solo||epoch_.empty()',
                'speed_cap_=max_speed;',
                'state0",reliable,',
                'COLLISION_LATCHED', 'MINIMUM_RISK_BRAKE'):
        assert key in CODE,key

def test_launch_single_authority_wiring_remains_static():
    launch=(HERE.parent/'f1tenth_racing_bringup/launch/race_foundation.launch.py')
    assert launch.is_file()
    s=launch.read_text()
    assert 'native_actuator_guard' in s and 'independent_guard_watchdog' in s
