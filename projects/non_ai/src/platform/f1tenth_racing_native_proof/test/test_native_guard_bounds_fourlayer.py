"""Exercise the same ROS-independent command guard used by native_guard_ros.cpp."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def test_native_guard_physics_and_lease_fail_closed():
    source = r"""
#include "native_guard_intent_bounds.hpp"
#include <cassert>
#include <limits>
using native_guard_intent_bounds::admissible;
int main() {
  const auto ok = [](double s, double a, long long created, long long expiry,
                     long long now, double ms=.4, double ma=4, double mb=8) {
      return admissible(s,a,created,expiry,now,ms,ma,mb);
  };
  assert(ok(.4,4,1000000000,1100000000,1075000000));
  assert(ok(-.4,-8,1000000000,1100000000,1075000000));
  assert(!ok(.401,0,1000000000,1100000000,1075000000));
  assert(!ok(0,-8.001,1000000000,1100000000,1075000000));
  assert(!ok(0,4.001,1000000000,1100000000,1075000000));
  assert(!ok(0,0,1000000000,2000000000,1075000000));
  assert(!ok(0,0,1000000000,1100000000,1075000001));
  assert(!ok(0,0,1000000000,1090000000,1090000001));
  assert(!ok(0,0,-1,1100000000,1075000000));
  assert(!ok(0,0,0,1100000000,1075000000));
  assert(!ok(0,0,1000000000,1100000000,999999999));
  assert(!ok(std::numeric_limits<double>::quiet_NaN(),0,1000000000,1100000000,1075000000));
  assert(!ok(0,std::numeric_limits<double>::infinity(),1000000000,1100000000,1075000000));
  assert(!ok(0,0,1000000000,1100000000,1075000000,0));
}
"""
    with tempfile.TemporaryDirectory() as tmp:
        code=Path(tmp)/'bounds.cpp'; exe=Path(tmp)/'bounds'
        code.write_text(source)
        subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror',
                        '-I'+str(ROOT/'src'),str(code),'-o',str(exe)],
                       check=True,capture_output=True,text=True)
        subprocess.run([str(exe)],check=True,capture_output=True,text=True)


def test_guard_uses_trusted_limits_and_progress_watchdog():
    cpp=(ROOT/'src/native_guard_ros.cpp').read_text()
    assert '#include "native_guard_intent_bounds.hpp"' in cpp
    assert 'native_guard_intent_bounds::admissible(' in cpp
    for expr in ('steering_limit_=parameters[10]',
                 'acceleration_limit_=parameters[13]',
                 'braking_limit_=parameters[14]',
                 'st-last_clock_progress_steady_ns_>75000000',
                 'now<last_ros_ns_', 'now>last_ros_ns_'):
        assert expr in cpp
    assert 'if(now<=last_ros_ns_&&last_ros_ns_>0)' not in cpp
    # Native mode remains SOLO-only; H2H is NOT silently approved.
    launch=(ROOT.parent/'f1tenth_racing_bringup/launch/race_foundation.launch.py').read_text()
    assert 'NATIVE_CPP_GUARD_H2H_NOT_QUALIFIED' in launch
