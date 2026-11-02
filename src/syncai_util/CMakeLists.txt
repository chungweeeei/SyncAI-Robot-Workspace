cmake_minimum_required(VERSION 3.8)
project(syncai_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# For StairMode (stair_mode.hpp): SetPolicyMode and the `mode` / `stair_mode`
# messages. syncai_common is vcs-imported from SyncAI-Robot-Interface, so this
# package, and every nav package above it, now needs that checkout to build.
find_package(std_msgs REQUIRED)
find_package(syncai_common REQUIRED)
# plan_crossings.cpp: the planner's sidecar is JSON (plan_crossings.hpp has
# why). Header-only and used in the .cpp only, but linked with the plain
# signature ament_target_dependencies uses, so it is in the exported target's
# interface and exported below for the packages that link this one.
find_package(nlohmann_json REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  syncai_common
)

add_library(${PROJECT_NAME} SHARED
  src/node_thread.cpp
  src/node_utils.cpp
  src/odometry_utils.cpp
  src/robot_utils.cpp
  src/stair_mode.cpp
  src/plan_crossings.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # ROS-free: the stair state machine driven through configure() with fake
  # policy / service / publish callbacks on a synthetic rising path.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stair_mode test/test_stair_mode.cpp)
  target_link_libraries(test_stair_mode ${PROJECT_NAME})
  ament_target_dependencies(test_stair_mode ${dependencies})

  # Header-only: the robot's place on a path that passes back over itself
  # (path_progress.hpp), on synthetic patrol routes.
  ament_add_gtest(test_plan_crossings test/test_plan_crossings.cpp)
  target_link_libraries(test_plan_crossings ${PROJECT_NAME})
  ament_add_gtest(test_path_progress test/test_path_progress.cpp)
  target_link_libraries(test_path_progress ${PROJECT_NAME})
  ament_target_dependencies(test_path_progress ${dependencies})

  # Header-only: where a path turns back on itself and the legs it is cut
  # into there (path_cusps.hpp), on synthetic patrol routes and a stair.
  ament_add_gtest(test_path_cusps test/test_path_cusps.cpp)
  target_link_libraries(test_path_cusps ${PROJECT_NAME})
  ament_target_dependencies(test_path_cusps ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies} nlohmann_json)

ament_package()
