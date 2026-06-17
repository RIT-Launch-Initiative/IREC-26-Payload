cmake_minimum_required(VERSION 3.16)
project(cubesat_stm_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cubesat_msgs REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

add_executable(stm_bridge_node
  src/main.cpp
  src/stm_bridge_node.cpp
  src/crashout_stm.cpp

)
target_link_libraries(stm_bridge_node ${GPIOD_LIBRARIES})


target_include_directories(stm_bridge_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${GPIOD_INCLUDE_DIRS})
ament_target_dependencies(stm_bridge_node rclcpp rclcpp_action cubesat_msgs)

install(TARGETS stm_bridge_node
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
