# Atomic Guard-to-Ego actuation envelope. Only the Guard publishes this.
# ROS graph checks are NOT cryptographic authentication. DDS security is needed
# before deployments in an adversarial process/network environment.
builtin_interfaces/Time stamp
builtin_interfaces/Time expires_ros_stamp
string run_epoch
string source_id
uint64 intent_sequence
uint64 source_state_sequence
string active_reference_id
uint32 active_revision
string track_id
string map_sha256
string authority_mode
bool stop_certificate_verified
float64 steering_rad
float64 acceleration_mps2
