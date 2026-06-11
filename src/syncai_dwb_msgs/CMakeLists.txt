cmake_minimum_required(VERSION 3.8)
project(syncai_dwb_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Twist2D.msg"
  "msg/Twist2D32.msg"
  "msg/Twist2DStamped.msg"
  "msg/Pose2D32.msg"
  "msg/Pose2DStamped.msg"
  "msg/Path2D.msg"
  "msg/CriticScore.msg"
  "msg/Trajectory2D.msg"
  "msg/TrajectoryScore.msg"
  "msg/LocalPlanEvaluation.msg"
  "srv/DebugLocalPlan.srv"
  "srv/GenerateTrajectory.srv"
  "srv/GenerateTwists.srv"
  "srv/GetCriticScore.srv"
  "srv/ScoreTrajectory.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
