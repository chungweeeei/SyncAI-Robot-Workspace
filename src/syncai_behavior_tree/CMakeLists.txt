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

add_library(syncai_compute_path_through_poses_action_bt_node SHARED
  plugins/action/compute_path_through_poses_action.cpp)
list(APPEND plugin_libs syncai_compute_path_through_poses_action_bt_node)

add_library(syncai_remove_passed_goals_action_bt_node SHARED
  plugins/action/remove_passed_goals_action.cpp)
list(APPEND plugin_libs syncai_remove_passed_goals_action_bt_node)

add_library(syncai_say_something_action_bt_node SHARED
  plugins/action/say_something_action.cpp)
list(APPEND plugin_libs syncai_say_something_action_bt_node)

# Behavior Tree control node plugins
add_library(syncai_pipeline_sequence_bt_node SHARED
  plugins/control/pipeline_sequence.cpp)
list(APPEND plugin_libs syncai_pipeline_sequence_bt_node)

# Behavior Tree condition node plugins
add_library(syncai_initial_pose_received_condition_bt_node SHARED
  plugins/condition/initial_pose_received_condition.cpp)
list(APPEND plugin_libs syncai_initial_pose_received_condition_bt_node)

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
add_executable(sync_action_demo examples/sync_action_demo.cpp)
target_link_libraries(sync_action_demo ${library_name})
ament_target_dependencies(sync_action_demo ${dependencies})

add_executable(stateful_action_demo examples/stateful_action_demo.cpp)
target_link_libraries(stateful_action_demo ${library_name})
ament_target_dependencies(stateful_action_demo ${dependencies})

add_executable(async_action_demo examples/async_action_demo.cpp)
target_link_libraries(async_action_demo ${library_name})
ament_target_dependencies(async_action_demo ${dependencies})

install(
  TARGETS sync_action_demo stateful_action_demo async_action_demo
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
