cmake_minimum_required(VERSION 3.8)
project(syncai_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# A dependency since 2026-10, for the CLIENT types of the docking nodes only
# (syncai_common/action/DockToPose, syncai_common/srv/SetMotionKey). The rule
# that kept it out until then still holds: the numeric error codes belong to
# the action being served, so no BT node reads NavigateToGoal's constants --
# the node -> code table stays in syncai_task_runner. Consequence: a change
# to syncai_common now rebuilds this package too.
find_package(syncai_common REQUIRED)
find_package(syncai_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  behaviortree_cpp_v3
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  std_srvs
  syncai_common
  syncai_util
  tf2
  tf2_ros
  tf2_geometry_msgs
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name} ${dependencies})

# Behavior Tree action node plugins
add_library(syncai_compute_path_to_pose_action_bt_node SHARED
  plugins/action/compute_path_to_pose_action.cpp)
list(APPEND plugin_libs syncai_compute_path_to_pose_action_bt_node)

add_library(syncai_follow_path_action_bt_node SHARED
  plugins/action/follow_path_action.cpp)
list(APPEND plugin_libs syncai_follow_path_action_bt_node)

add_library(syncai_clear_costmap_service_bt_node SHARED
  plugins/action/clear_costmap_service.cpp)
list(APPEND plugin_libs syncai_clear_costmap_service_bt_node)

# Docking (2026-10): the nodes dock.xml / undock.xml in syncai_task_runner use
add_library(syncai_dock_to_pose_action_bt_node SHARED
  plugins/action/dock_to_pose_action.cpp)
list(APPEND plugin_libs syncai_dock_to_pose_action_bt_node)

add_library(syncai_set_motion_key_service_bt_node SHARED
  plugins/action/set_motion_key_service.cpp)
list(APPEND plugin_libs syncai_set_motion_key_service_bt_node)

add_library(syncai_compute_staging_pose_action_bt_node SHARED
  plugins/action/compute_staging_pose.cpp)
list(APPEND plugin_libs syncai_compute_staging_pose_action_bt_node)

add_library(syncai_compute_back_out_pose_action_bt_node SHARED
  plugins/action/compute_back_out_pose.cpp)
list(APPEND plugin_libs syncai_compute_back_out_pose_action_bt_node)

add_library(syncai_wait_action_bt_node SHARED
  plugins/action/wait_action.cpp)
list(APPEND plugin_libs syncai_wait_action_bt_node)

add_library(syncai_wait_for_motion_state_action_bt_node SHARED
  plugins/action/wait_for_motion_state.cpp)
list(APPEND plugin_libs syncai_wait_for_motion_state_action_bt_node)

# Behavior Tree control node plugins
add_library(syncai_pipeline_sequence_bt_node SHARED
  plugins/control/pipeline_sequence.cpp)
list(APPEND plugin_libs syncai_pipeline_sequence_bt_node)

add_library(syncai_recovery_node_bt_node SHARED
  plugins/control/recovery_node.cpp)
list(APPEND plugin_libs syncai_recovery_node_bt_node)

# Behavior Tree condition node plugins
add_library(syncai_initial_pose_received_condition_bt_node SHARED
  plugins/condition/initial_pose_received_condition.cpp)
list(APPEND plugin_libs syncai_initial_pose_received_condition_bt_node)

add_library(syncai_is_path_valid_condition_bt_node SHARED
  plugins/condition/is_path_valid_condition.cpp)
list(APPEND plugin_libs syncai_is_path_valid_condition_bt_node)

add_library(syncai_globally_updated_goal_condition_bt_node SHARED
  plugins/condition/globally_updated_goal_condition.cpp)
list(APPEND plugin_libs syncai_globally_updated_goal_condition_bt_node)

# Behavior Tree decorator node plugins
add_library(syncai_rate_controller_bt_node SHARED
  plugins/decorator/rate_controller.cpp)
list(APPEND plugin_libs syncai_rate_controller_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_include_directories(${bt_plugin} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(
  TARGETS ${library_name}
  EXPORT export_${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Example executables
add_executable(demo examples/demo.cpp)
target_link_libraries(demo ${library_name})
ament_target_dependencies(demo ${dependencies})

install(
  TARGETS demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${plugin_libs}
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
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_targets(export_${library_name})
ament_export_dependencies(${dependencies})

ament_package()
