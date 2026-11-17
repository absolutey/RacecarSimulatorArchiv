"""ROS-built typed atomic Guard command smoke/identity contract."""
from control_msgs.msg import GuardedActuatorCommand


def test_atomic_message_carries_proof_and_lineage_in_one_transport():
    msg=GuardedActuatorCommand()
    msg.stamp.sec=17
    msg.expires_ros_stamp.sec=17
    msg.expires_ros_stamp.nanosec=70_000_000
    msg.run_epoch='verification-only'
    msg.source_id='R_B'
    msg.intent_sequence=19
    msg.source_state_sequence=31
    msg.active_reference_id='nominal-test'
    msg.active_revision=3
    msg.track_id='iccas2025'
    msg.map_sha256='9'*64
    msg.authority_mode='CERTIFIED'
    msg.stop_certificate_verified=True
    msg.steering_rad=0.012
    msg.acceleration_mps2=-0.6
    assert msg.intent_sequence==19
    assert msg.source_state_sequence==31
    assert msg.stop_certificate_verified
    assert msg.authority_mode=='CERTIFIED'
    assert msg.active_reference_id=='nominal-test'
