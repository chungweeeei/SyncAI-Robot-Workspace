cmake_minimum_required(VERSION 3.8)
project(syncai_costmap_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(syncai_util REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  geometry_msgs
  nav_msgs
  nav2_msgs
  map_msgs
  rcl_interfaces
  syncai_util
  sensor_msgs
  laser_geometry
  message_filters
  tf2_sensor_msgs
  std_srvs
)

add_library(${PROJECT_NAME} SHARED
  src/array_parser.cpp
  src/costmap_2d.cpp
  src/costmap_2d_publisher.cpp
  src/costmap_2d_ros.cpp
  src/costmap_layer.cpp
  src/costmap_math.cpp
  src/costmap_subscriber.cpp
  src/footprint.cpp
  src/footprint_collision_checker.cpp
  src/footprint_subscriber.cpp
  src/layer.cpp
  src/layered_costmap.cpp
  src/observation_buffer.cpp
  plugins/static_layer.cpp
  plugins/inflation_layer.cpp
  plugins/obstacle_layer.cpp
  plugins/costmap_filters/costmap_filter.cpp
  plugins/costmap_filters/keepout_filter.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Register the costmap layer plugins (e.g. StaticLayer) with pluginlib
pluginlib_export_plugin_description_file(syncai_costmap_2d costmap_plugins.xml)

# Standalone node to run the costmap (manual configure/activate, non-lifecycle)
add_executable(costmap_2d_node src/costmap_2d_node.cpp)
target_link_libraries(costmap_2d_node ${PROJECT_NAME})
ament_target_dependencies(costmap_2d_node ${dependencies})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS costmap_2d_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/fake_scan.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
