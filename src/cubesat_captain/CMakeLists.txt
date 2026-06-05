cmake_minimum_required(VERSION 3.16)
project(cubesat_captain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cubesat_msgs REQUIRED)

add_executable(captain_node
  src/main.cpp
  src/captain_node.cpp
  src/captain_comms.cpp
  src/pad_state.cpp
  src/flipping_state.cpp
  src/status_accumulator.cpp
  src/packet_writer.cpp
)

add_subdirectory(cubesat_comms)
target_link_libraries(captain_node cubesat_comms)



target_include_directories(captain_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cubesat_comms/include>)

ament_target_dependencies(captain_node rclcpp rclcpp_action cubesat_msgs)

install(TARGETS captain_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()
