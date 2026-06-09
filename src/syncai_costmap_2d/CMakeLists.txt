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
)

add_library(${PROJECT_NAME} SHARED
  src/array_parser.cpp
  src/costmap_2d.cpp
  src/costmap_2d_publisher.cpp
  src/costmap_2d_ros.cpp
  src/costmap_math.cpp
  src/costmap_subscriber.cpp
  src/footprint.cpp
  src/footprint_collision_checker.cpp
  src/footprint_subscriber.cpp
  src/layer.cpp
  src/layered_costmap.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

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
