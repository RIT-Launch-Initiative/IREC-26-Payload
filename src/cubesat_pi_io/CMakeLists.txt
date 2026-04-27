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

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()
