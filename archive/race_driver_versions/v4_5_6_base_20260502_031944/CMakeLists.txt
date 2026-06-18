cmake_minimum_required(VERSION 3.8)
project(race_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(race_driver_node
  src/main.cpp
  src/race_driver_node.cpp
  src/state_manager.cpp
  src/track_manager.cpp
  src/obstacle_detector.cpp
  src/occupancy_grid_manager.cpp
  src/map_rollout_planner.cpp
  src/boundary_corridor_planner.cpp
  src/candidate_generator.cpp
  src/cost_evaluator.cpp
  src/raceline_planner.cpp
  src/racing_reference.cpp
  src/speed_planner.cpp
  src/controller.cpp
)

ament_target_dependencies(race_driver_node
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  race_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
