cmake_minimum_required(VERSION 3.8)
project(control_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActiveTargetTrajectory.msg"
  "msg/CarState.msg"
  "msg/ActuatorIntent.msg"
  "msg/GuardedActuatorCommand.msg"
  "msg/ActuatorGuardStatus.msg"
  "msg/ControllerStatus.msg"
  "msg/RacingTrajectorySample.msg"
  "msg/OpponentState.msg"
  "msg/OpponentPredictionSample.msg"
  "msg/OpponentPrediction.msg"
  "msg/OvertakingPlan.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
