cmake_minimum_required(VERSION 3.16)
project(cubesat_radio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(cubesat_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

add_subdirectory(semtech)

add_library(cubesat_radio_hal STATIC
  linux_hal/sx126x_linux_hal.cpp
)
target_include_directories(cubesat_radio_hal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/linux_hal>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/semtech/src>
  $<INSTALL_INTERFACE:include>
  ${GPIOD_INCLUDE_DIRS}
)
target_link_libraries(cubesat_radio_hal PUBLIC ${GPIOD_LIBRARIES})

add_library(cubesat_radio_core STATIC
  src/sx1262_radio.cpp
)
target_include_directories(cubesat_radio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/linux_hal>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/semtech/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cubesat_radio_core PUBLIC cubesat_radio_hal sx126x_driver)

add_executable(radio_node
  src/main.cpp
  src/radio_node.cpp
)
target_include_directories(radio_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(radio_node rclcpp cubesat_msgs)
target_link_libraries(radio_node cubesat_radio_core)

install(TARGETS
  cubesat_radio_hal
  cubesat_radio_core
  radio_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY semtech/
  DESTINATION share/${PROJECT_NAME}/semtech
)

install(
  DIRECTORY linux_hal/
  DESTINATION share/${PROJECT_NAME}/linux_hal
)

ament_package()
