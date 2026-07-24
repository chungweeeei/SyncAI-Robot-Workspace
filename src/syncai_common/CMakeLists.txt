cmake_minimum_required(VERSION 3.8)
project(syncai_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArtifactState.msg"
  "msg/IMUState.msg"
  "msg/MotorState.msg"
  "msg/MotorStates.msg"
  "msg/RobotBatteryStatus.msg"
  "msg/RobotLocalizationStatus.msg"
  "msg/RobotMode.msg"
  "msg/RobotNetworkStatus.msg"
  "msg/RobotPose.msg"
  "msg/RobotState.msg"
  "msg/RobotStatus.msg"
  "msg/WifiNetwork.msg"
  "msg/WifiStatus.msg"
  "srv/ConnectWifiNetwork.srv"
  "srv/ScanWifiNetworks.srv"
  "srv/SetMotionKey.srv"
  "srv/SetPolicyMode.srv"
  "srv/SetSpeedScale.srv"
  "action/ExecuteTask.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
