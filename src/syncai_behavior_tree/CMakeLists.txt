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

# Example showing how to drive the BehaviorTreeEngine
# (ros2 run syncai_behavior_tree engine_demo)
add_executable(engine_demo examples/engine_demo.cpp)
target_link_libraries(engine_demo ${library_name})
ament_target_dependencies(engine_demo ${dependencies})

# Behavior Tree action node plugins
add_library(syncai_compute_path_to_pose_action_bt_node SHARED
  plugins/action/compute_path_to_pose_action.cpp)
list(APPEND plugin_libs syncai_compute_path_to_pose_action_bt_node)

add_library(syncai_follow_path_action_bt_node SHARED
  plugins/action/follow_path_action.cpp)
list(APPEND plugin_libs syncai_follow_path_action_bt_node)

add_library(syncai_navigate_to_pose_action_bt_node SHARED
  plugins/action/navigate_to_pose_action.cpp)
list(APPEND plugin_libs syncai_navigate_to_pose_action_bt_node)

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

install(
  TARGETS engine_demo
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
