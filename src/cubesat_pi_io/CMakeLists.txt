cmake_minimum_required(VERSION 3.16)
project(cubesat_pi_io)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cubesat_msgs REQUIRED)

add_subdirectory(nmea)

add_library(cubesat_pi_io_drivers STATIC
  src/gps_nmea_reader.cpp
  src/ina260_driver.cpp
  src/lis3dh_driver.cpp
)
target_include_directories(cubesat_pi_io_drivers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cubesat_pi_io_drivers PRIVATE nmea)

add_executable(pi_io_node
  src/main.cpp
  src/pi_io_node.cpp
)
target_include_directories(pi_io_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(pi_io_node rclcpp cubesat_msgs)
target_link_libraries(pi_io_node cubesat_pi_io_drivers)

install(TARGETS pi_io_node
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
