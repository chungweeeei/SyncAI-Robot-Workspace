cmake_minimum_required(VERSION 3.8)
project(syncai_mapping)

# GTSAM iSAM2 plus an ICP loop-closure check on every keyframe, and a full-map
# transform+voxel merge on the worker: unoptimised this cannot keep up with a
# 0.5 m keyframe cadence on the Jetson. The upstream CMakeLists forced
# CMAKE_BUILD_TYPE to Release unconditionally, which also silently defeated an
# explicit `--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`; the guard keeps
# the default and lets a deliberate choice through (same as syncai_pointlio).
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# -pthread / -fexceptions carried over from upstream: the map-cloud merge runs
# on a raw std::thread, and the flags predate the CMake thread-target idiom.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
# SaveMaps / ResetMapping (served here) and ResetLIO (the client that drives
# syncai_pointlio's reset). All three are in syncai_common since the port:
# the backend, the only caller of the first two, builds against that package.
find_package(syncai_common REQUIRED)

# GTSAM (both nodes) and Sophus (hba_node's BALM) are source-built into
# /usr/local by the Dockerfile's deps-builder stage (GTSAM 4.2.0 with system
# Eigen and TBB on; Sophus 1.22.10 header-only) and have no rosdep key
# matching those builds -- see package.xml. Eigen / PCL come in through
# pcl_conversions as well; the explicit finds are for ${PCL_LIBRARIES} and
# the include dirs GTSAM's headers need unqualified.
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenMP QUIET)
# pgo_node's loop verification (small_gicp's RegistrationPCL, since 2026-10).
# Same arrangement as syncai_localizer: src/third-party/small_gicp is a
# build_type cmake package, ordered before this one by <depend>small_gicp</depend>.
find_package(small_gicp REQUIRED)

set(SYNCAI_MAPPING_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  pcl_conversions
  message_filters
  syncai_common
)

# ---- pgo_node: the online pose graph (mapping session) ----------------------
add_executable(pgo_node
  src/pgo_main.cpp
  src/pgo_node.cpp
  src/pgos/commons.cpp
  src/pgos/simple_pgo.cpp
)
target_include_directories(pgo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})
ament_target_dependencies(pgo_node ${SYNCAI_MAPPING_DEPS})
# small_gicp::small_gicp carries OpenMP::OpenMP_CXX as a PUBLIC usage
# requirement; without -fopenmp its omp pragmas are silently ignored and the
# kd-tree / covariance estimation run single-threaded.
target_link_libraries(pgo_node
  ${PCL_LIBRARIES}
  gtsam
  small_gicp::small_gicp
)

# ---- hba_node: offline hierarchical bundle adjustment (by hand) -------------
# Its own target rather than a second node in pgo_node's process: it is never
# in a session, and hba/commons.h and pgos/commons.h both define the global
# PointType / CloudType aliases (differently), so the two trees must never
# share a translation unit.
add_executable(hba_node
  src/hba_main.cpp
  src/hba_node.cpp
  src/hba/commons.cpp
  src/hba/blam.cpp
  src/hba/hba.cpp
)
target_include_directories(hba_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})
ament_target_dependencies(hba_node ${SYNCAI_MAPPING_DEPS})
# SOPHUS_USE_BASIC_LOGGING must match the source build (no fmt) or Sophus's
# headers pull fmt back in; MP_EN / MP_PROC_NUM are BALM's OpenMP switches,
# upstream names, 4 threads as upstream shipped them for an offline job that
# runs with nothing else on the box. Per target: pgo_node needs neither.
target_compile_definitions(hba_node PRIVATE SOPHUS_USE_BASIC_LOGGING MP_EN MP_PROC_NUM=4)
target_link_libraries(hba_node
  ${PCL_LIBRARIES}
  gtsam
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(hba_node OpenMP::OpenMP_CXX)
endif()

install(TARGETS pgo_node hba_node
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
  # pgos/ and hba/ are ported upstream code with no per-file licence header
  # (the package LICENSE covers them), so the copyright linter would fail on
  # every one of those files; cpplint is off for the same files as it was
  # upstream. The rest of ament_lint_common still runs.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
