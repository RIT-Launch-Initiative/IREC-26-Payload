cmake_minimum_required(VERSION 3.8)
project(cubesat_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/FlightState.msg"
  "msg/PowerSample.msg"
  "msg/GpsSample.msg"
  "msg/ImuSample.msg"
  "msg/BoostEvent.msg"
  "msg/ArmStatus.msg"
  "msg/RadioPacket.msg"
  # Services
  "srv/StartNewFlight.srv"
  "srv/SetFlightFolder.srv"
  "srv/SetServoPosition.srv"
  "srv/MoveServoRamp.srv"
  "srv/SendRadioPacket.srv"
  # Actions
  "action/FlipVehicle.action"
  "action/ExtendArm.action"
  "action/CapturePanorama.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
