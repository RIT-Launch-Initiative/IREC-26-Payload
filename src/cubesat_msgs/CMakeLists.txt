cmake_minimum_required(VERSION 3.16)
project(cubesat_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/FlightState.msg"
  "msg/PowerSample.msg"
  "msg/GpsSample.msg"
  "msg/AccelSample.msg"
  "msg/BoostEvent.msg"
  "msg/ArmStatus.msg"
  "msg/RadioPacket.msg"
  "msg/ArmJoint.msg"
  "msg/ArmState.msg"
  "msg/FlipServo.msg"
  "msg/PayloadOrientation.msg"
  "msg/TelemetryType.msg"
  "msg/LoraParameters.msg"
  "msg/RadioState.msg"
  "msg/ImageRequest.msg"
  "msg/ImageMetadata.msg"
  "msg/SystemHealth.msg"
  # Services
  "srv/StartNewFlight.srv"
  "srv/SetFlightFolder.srv"
  "srv/SetServoPosition.srv"
  "srv/MoveServoRamp.srv"
  "srv/SendRadioPacket.srv"
  "srv/TransmitImageBlocks.srv"
  "srv/HoldShut.srv"
  "srv/RequestStateChange.srv"
  "srv/TelemetryRequest.srv"
  "srv/SetBuzzer.srv"
  "srv/JogMotor.srv"
  "srv/ZeroArm.srv"
  # Actions
  "action/FlipVehicle.action"
  "action/FlipServoAction.action"
  "action/ExtendArm.action"
  "action/CapturePanorama.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
