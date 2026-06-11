cmake_minimum_required(VERSION 3.8)
project(syncai_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(syncai_util REQUIRED)
find_package(syncai_costmap_2d REQUIRED)
find_package(syncai_nav_core REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  builtin_interfaces
  rcl_interfaces
  tf2_ros
  pluginlib
  syncai_util
  syncai_costmap_2d
  syncai_nav_core
)

set(library_name ${PROJECT_NAME})
set(executable_name planner_server)

add_library(${library_name} SHARED
  plugins/navfn_planner/navfn.cpp
  plugins/navfn_planner/navfn_planner.cpp
  plugins/straight_line_planner/straight_line_planner.cpp
  src/planner_server.cpp
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name} ${dependencies})

pluginlib_export_plugin_description_file(syncai_nav_core global_planner_plugin.xml)

add_executable(${executable_name} src/main.cpp)
target_link_libraries(${executable_name} ${library_name})
ament_target_dependencies(${executable_name} ${dependencies})

install(
  TARGETS ${library_name}
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

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES global_planner_plugin.xml
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
