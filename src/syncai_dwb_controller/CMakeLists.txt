cmake_minimum_required(VERSION 3.8)
project(syncai_dwb_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(syncai_costmap_2d REQUIRED)
find_package(syncai_dwb_msgs REQUIRED)
find_package(syncai_nav_core REQUIRED)
find_package(syncai_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  angles
  builtin_interfaces
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs
  syncai_costmap_2d
  syncai_dwb_msgs
  syncai_nav_core
  syncai_util
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/alignment_util.cpp
  src/base_obstacle.cpp
  src/conversions.cpp
  src/costmap_queue.cpp
  src/dwb_local_planner.cpp
  src/goal_align.cpp
  src/goal_dist.cpp
  src/illegal_trajectory_tracker.cpp
  src/kinematic_parameters.cpp
  src/limited_accel_generator.cpp
  src/limited_costmap_queue.cpp
  src/map_grid.cpp
  src/obstacle_footprint.cpp
  src/oscillation.cpp
  src/path_align.cpp
  src/path_dist.cpp
  src/path_ops.cpp
  src/prefer_forward.cpp
  src/publisher.cpp
  src/rotate_to_goal.cpp
  src/standard_traj_generator.cpp
  src/tf_help.cpp
  src/trajectory_utils.cpp
  src/twirling.cpp
  src/xy_theta_iterator.cpp
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name} ${dependencies})

pluginlib_export_plugin_description_file(syncai_nav_core controller_plugin.xml)
pluginlib_export_plugin_description_file(syncai_dwb_controller dwb_plugins.xml)

install(
  TARGETS ${library_name}
  EXPORT export_${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES controller_plugin.xml dwb_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_targets(export_${library_name})
ament_export_dependencies(${dependencies})

ament_package()
