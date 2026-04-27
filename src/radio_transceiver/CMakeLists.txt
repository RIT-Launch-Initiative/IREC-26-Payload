cmake_minimum_required(VERSION 3.10)
project(radio_transceiver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(arm_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

add_subdirectory(sx126x_driver)

add_executable(radio_transceiver_node 
  src/main.cpp
  src/RadioTransceiverNode.cpp
  src/sx126xHal.cpp
)

ament_target_dependencies(radio_transceiver_node
  arm_msgs
  rclcpp
  std_msgs
  builtin_interfaces
  sensor_msgs
)

target_include_directories(radio_transceiver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GPIOD_INCLUDE_DIRS}
)

install(TARGETS radio_transceiver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(radio_transceiver_node
  sx126x_driver
  ${GPIOD_LIBRARIES}
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
