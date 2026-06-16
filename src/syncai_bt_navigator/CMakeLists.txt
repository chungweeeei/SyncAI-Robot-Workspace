cmake_minimum_required(VERSION 3.8)
project(syncai_bt_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(syncai_behavior_tree REQUIRED)
find_package(syncai_test_bt_action_msgs REQUIRED)
find_package(syncai_util REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  ament_index_cpp
  behaviortree_cpp_v3
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  syncai_behavior_tree
  syncai_test_bt_action_msgs
  syncai_util
  tf2_ros
)

# NavigateToPose navigator
add_library(syncai_navigate_to_pose_navigator SHARED
  src/navigate_to_pose.cpp
)
target_include_directories(syncai_navigate_to_pose_navigator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(syncai_navigate_to_pose_navigator ${dependencies})

# BtNavigator host node library
add_library(${PROJECT_NAME} SHARED
  src/bt_navigator.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} syncai_navigate_to_pose_navigator)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# BtNavigator executable entry point
add_executable(bt_navigator src/main.cpp)
target_link_libraries(bt_navigator ${PROJECT_NAME})
ament_target_dependencies(bt_navigator ${dependencies})

# Standalone runner that exposes the /askmodel action and runs behavior_trees/
# askmodel.xml — a minimal BtActionServer node WITHOUT the Navigator layer.
add_executable(askmodel_bt_runner src/askmodel_bt_runner.cpp)
ament_target_dependencies(askmodel_bt_runner ${dependencies})

install(
  TARGETS syncai_navigate_to_pose_navigator ${PROJECT_NAME}
  EXPORT export_syncai_bt_navigator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS bt_navigator askmodel_bt_runner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(syncai_navigate_to_pose_navigator ${PROJECT_NAME})
ament_export_targets(export_syncai_bt_navigator)
ament_export_dependencies(${dependencies})

ament_package()
