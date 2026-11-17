cmake_minimum_required(VERSION 3.8)
project(f1tenth_racing_tools)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/race_start.py DESTINATION lib/${PROJECT_NAME} RENAME race_start)
install(PROGRAMS scripts/f1sim scripts/f1fast scripts/f1safe scripts/f1go scripts/f1solo scripts/f1h2h
  ../../verification/h2h_champion/resource_preflight.py DESTINATION lib/${PROJECT_NAME})
ament_package()
