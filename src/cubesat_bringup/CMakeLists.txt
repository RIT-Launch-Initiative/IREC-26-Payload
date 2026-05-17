cmake_minimum_required(VERSION 3.16)
project(cubesat_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/start_flight.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
