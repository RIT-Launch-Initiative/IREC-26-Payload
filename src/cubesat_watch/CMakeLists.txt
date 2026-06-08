cmake_minimum_required(VERSION 3.10)
project(cubesat_watch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED) 
find_package(cubesat_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# SSDV library (C sources)
add_library(ssdv STATIC
  third_party/ssdv/ssdv.c
  third_party/ssdv/rs8.c
)
target_include_directories(ssdv PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)
# Suppress pedantic warnings for third-party C code
set_target_properties(ssdv PROPERTIES
  C_STANDARD 99
  COMPILE_OPTIONS "-w"
)

add_executable(cubesat_watch_node
  src/main.cpp
  src/ImageHandlerNode.cpp
  src/ImageTaker.cpp
)


target_link_directories(cubesat_watch_node PRIVATE ${rclcpp_DIR}/../../../lib/)


ament_target_dependencies(cubesat_watch_node
  cubesat_msgs
  rclcpp
  std_msgs
  builtin_interfaces
  cv_bridge
)

target_link_libraries(cubesat_watch_node
  ${OpenCV_LIBRARIES}
  ssdv
  camera camera-base
)

target_include_directories(cubesat_watch_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
  ${rclcpp_DIR}/../../../include/libcamera/
)
install(TARGETS cubesat_watch_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
