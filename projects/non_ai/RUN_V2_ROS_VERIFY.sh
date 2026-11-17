#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"
EVIDENCE="${EVIDENCE_ROOT:-$PWD/../RAB_V2_ROS_EVIDENCE}"
mkdir -p "$EVIDENCE"
echo '== RELEASE CHECK =='
python3 -B VERIFY_RELEASE.py | tee "$EVIDENCE/VERIFY_RELEASE.log"
echo '== ROS2 HUMBLE ENVIRONMENT =='
set +u
source /opt/ros/humble/setup.bash
set -u
echo "ROS_DISTRO=${ROS_DISTRO:-MISSING}"
echo '== FULL ROS BUILD =='
colcon build --symlink-install --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/BUILD.log"
set +u
source install/setup.bash
set -u
echo '== ATOMIC COMMAND INTERFACE =='
ros2 interface show control_msgs/msg/GuardedActuatorCommand | tee "$EVIDENCE/ATOMIC_INTERFACE.log"
echo '== FULL ROS TEST =='
colcon test --event-handlers console_direct+ 2>&1 | tee "$EVIDENCE/TEST.log"
echo '== ROS TEST RESULT =='
colcon test-result --verbose | tee "$EVIDENCE/TEST_RESULTS.log"
echo "EVIDENCE_ROOT=$EVIDENCE"
echo 'V2 ROS build/tests completed; H2H/Guard E2E qualification NOT asserted.'
