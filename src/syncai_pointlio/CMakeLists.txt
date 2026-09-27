cmake_minimum_required(VERSION 3.8)
project(syncai_pointlio)

# Point-LIO is the one package in this workspace where an unoptimised build is
# not merely slow but unusable: the ikd-tree and the per-point EKF update cannot
# hold the lidar rate on the Jetson without -O3. The upstream CMakeLists forced
# CMAKE_BUILD_TYPE to Release unconditionally, which also silently defeated any
# `--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` passed for profiling; the
# guard below keeps the default and lets an explicit choice through.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# -pthread / -fexceptions carried over from upstream: ikd_Tree runs its rebuild
# on a raw pthread, and the flags predate the CMake thread-target idiom.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Sophus is built into /usr/local by the Dockerfile's deps-builder stage with
# SOPHUS_USE_BASIC_LOGGING=ON (no fmt dependency); the define has to match on
# the consumer side or its headers pull fmt back in.
add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

# ikd_Tree multi-processing switches (upstream names). MP_PROC_NUM=2 is the
# OpenMP thread count for the point-to-plane residual loop; it was tuned for
# the Jetson's core budget alongside the localizer and the costmaps.
add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
# The `reset` service type. It moved from SyncAI-Fast-LIO2's `interface`
# package to syncai_common (SyncAI-Robot-Interface) with this port, because the
# server is here and the only client (pgo_node) stayed in that repo.
find_package(syncai_common REQUIRED)

# PCL / Eigen come in through pcl_conversions' exported include dirs as well;
# the explicit finds are for ${PCL_LIBRARIES}. Sophus is header-only and has no
# rosdep key that matches the source build -- see package.xml.
find_package(OpenMP QUIET)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

add_executable(pointlio_node
  src/main.cpp
  src/pointlio_node.cpp
  src/utils.cpp
  src/map_builder/commons.cpp
  src/map_builder/point_ekf.cpp
  src/map_builder/imu_initializer.cpp
  src/map_builder/ikd_Tree.cpp
  src/map_builder/lidar_processor.cpp
  src/map_builder/map_builder.cpp
)
target_include_directories(pointlio_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pointlio_node
  rclcpp
  std_msgs
  tf2
  tf2_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  livox_ros_driver2
  syncai_common
)
target_link_libraries(pointlio_node
  ${PCL_LIBRARIES}
)

install(TARGETS pointlio_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # map_builder/ is ported Point-LIO / ikd-Tree code with no per-file licence
  # header (the package LICENSE covers it) and not clang-format clean; the two
  # linters below would fail on every one of those files, so they stay off as
  # they were upstream. The rest of ament_lint_common still runs.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
