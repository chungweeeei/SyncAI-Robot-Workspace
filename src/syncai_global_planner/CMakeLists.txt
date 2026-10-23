cmake_minimum_required(VERSION 3.8)
project(syncai_global_planner)

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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(syncai_util REQUIRED)
find_package(syncai_costmap_2d REQUIRED)
find_package(syncai_nav_core REQUIRED)
# OctomapPlanner3D's dynamic overlay subscribes the lidar cloud through a TF
# message filter and transforms it into the map frame (2026-10-21). All three
# came in transitively through syncai_costmap_2d before, but a target that
# includes their headers has to name them.
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
# smac planner dependencies. ompl is still required after the Hybrid-A*
# planner was dropped: Smoother uses its Dubins state space to resample the
# boundary segments of a path. Its humble CMake config only exports classic
# OMPL_INCLUDE_DIRS / OMPL_LIBRARIES variables (no imported target), so it is
# linked by variable below rather than via the dependencies list.
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
# OctomapPlanner3D: octomap's humble config also exports only the classic
# OCTOMAP_INCLUDE_DIRS / OCTOMAP_LIBRARIES variables (the imported targets
# arrived in 1.10), so it is linked by variable like OMPL. It is the same
# ros-humble-octomap syncai_mapping's clean_map builds against.
find_package(octomap REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  builtin_interfaces
  rcl_interfaces
  std_msgs
  std_srvs
  tf2_ros
  pluginlib
  syncai_util
  syncai_costmap_2d
  syncai_nav_core
  angles
  sensor_msgs
  message_filters
  tf2_sensor_msgs
)

set(library_name ${PROJECT_NAME})
set(executable_name planner_server)

add_library(${library_name} SHARED
  plugins/navfn_planner/navfn.cpp
  plugins/navfn_planner/navfn_planner.cpp
  plugins/straight_line_planner/straight_line_planner.cpp
  plugins/smac_planner/a_star.cpp
  plugins/smac_planner/collision_checker.cpp
  plugins/smac_planner/costmap_downsampler.cpp
  plugins/smac_planner/node_2d.cpp
  plugins/smac_planner/node_basic.cpp
  plugins/smac_planner/smac_planner_2d.cpp
  plugins/smac_planner/smoother.cpp
  plugins/octomap_planner_3d/surface_map.cpp
  plugins/octomap_planner_3d/surface_astar.cpp
  plugins/octomap_planner_3d/path_postprocess.cpp
  plugins/octomap_planner_3d/stair_links.cpp
  plugins/octomap_planner_3d/dynamic_overlay.cpp
  plugins/octomap_planner_3d/octomap_planner_3d.cpp
  src/planner_server.cpp
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# SYSTEM keeps the workspace-wide -Wpedantic from flagging OMPL's headers,
# which were never written against it.
target_include_directories(${library_name} SYSTEM PUBLIC ${OMPL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

ament_target_dependencies(${library_name} ${dependencies})
target_link_libraries(${library_name}
  ${OMPL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  Eigen3::Eigen
)
# The smac A* search is the hot path and upstream always builds it -O3; plan
# latency is user-visible, so force it regardless of the workspace build type.
target_compile_options(${library_name} PRIVATE -O3)

pluginlib_export_plugin_description_file(syncai_nav_core global_planner_plugin.xml)

add_executable(${executable_name} src/main.cpp)
target_link_libraries(${executable_name} ${library_name})
ament_target_dependencies(${executable_name} ${dependencies})

# The offline driver of OctomapPlanner3D's ROS-free core: builds the surface
# map from a map's octomap.bt, prints what it made of it, checks it against
# the map's own poses.txt, dumps it for a viewer, plans. It links the planner
# library for the three core objects (the simplest thing inside the ROS
# build); plugins/octomap_planner_3d/standalone/CMakeLists.txt builds the same
# binary with octomap alone on a workstation without ROS.
add_executable(octomap_planner_3d_cli plugins/octomap_planner_3d/octomap_planner_3d_cli.cpp)
target_link_libraries(octomap_planner_3d_cli ${library_name})
target_compile_options(octomap_planner_3d_cli PRIVATE -O3)

install(
  TARGETS ${library_name}
  EXPORT export_${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${executable_name} octomap_planner_3d_cli
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

  # OctomapPlanner3D's geometry rules on synthetic scenes (ramp / stairs /
  # kerb / wall / holes / two floors); no ROS, runs in seconds.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_surface_map test/test_surface_map.cpp)
  target_link_libraries(test_surface_map ${library_name})
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_targets(export_${library_name})
ament_export_dependencies(${dependencies})

ament_package()
