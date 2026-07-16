cmake_minimum_required(VERSION 3.8)
project(syncai_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(syncai_costmap_2d REQUIRED)
find_package(syncai_nav_core REQUIRED)
find_package(syncai_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  angles
  geometry_msgs
  nav2_msgs
  nav_msgs
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_action
  syncai_costmap_2d
  syncai_nav_core
  syncai_util
  tf2
  tf2_geometry_msgs
  tf2_ros
)

set(library_name ${PROJECT_NAME})
set(executable_name controller_server)

# Controller / progress checker / goal checker plugins
add_library(${library_name}_plugins SHARED
  plugins/simple_progress_checker.cpp
  plugins/pose_progress_checker.cpp
  plugins/simple_goal_checker.cpp
  plugins/stopped_goal_checker.cpp
  plugins/position_goal_checker.cpp
  plugins/regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.cpp
)

target_include_directories(${library_name}_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name}_plugins ${dependencies})

pluginlib_export_plugin_description_file(syncai_nav_core plugins.xml)

# Controller server
add_library(${library_name} SHARED
  src/controller_server.cpp
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name} ${dependencies})

add_executable(${executable_name} src/main.cpp)
target_link_libraries(${executable_name} ${library_name})
ament_target_dependencies(${executable_name} ${dependencies})

install(
  TARGETS ${library_name} ${library_name}_plugins
  EXPORT export_${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name} ${library_name}_plugins)
ament_export_targets(export_${library_name})
ament_export_dependencies(${dependencies})

ament_package()
