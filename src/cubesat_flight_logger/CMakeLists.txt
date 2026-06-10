cmake_minimum_required(VERSION 3.16)
project(cubesat_flight_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cubesat_msgs REQUIRED)

# pure library: no ROS dependencies so it is usable outside the node
add_library(flight_logger STATIC
  src/flight_logger.cpp
)
target_include_directories(flight_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(flight_logger_node
  src/flight_logger_node.cpp
)
target_link_libraries(flight_logger_node flight_logger)
ament_target_dependencies(flight_logger_node rclcpp cubesat_msgs)

install(TARGETS flight_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()
