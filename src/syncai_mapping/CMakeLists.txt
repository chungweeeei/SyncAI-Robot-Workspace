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
# pgo_node finds the clean_map executable it spawns after a save through the
# package index, so the install tree it runs from decides which one.
find_package(ament_index_cpp REQUIRED)
# SaveMaps / ResetMapping (served here) and ResetLIO (the client that drives
# syncai_pointlio's reset). All three are in syncai_common since the port:
# the backend, the only caller of the first two, builds against that package.
find_package(syncai_common REQUIRED)

# GTSAM is source-built into /usr/local by the Dockerfile's deps-builder stage
# (4.2.0 with system Eigen and TBB on) and has no rosdep key matching that
# build -- see package.xml. Sophus left with hba_node (2026-10): pgo_node never
# included it. Eigen / PCL come in through
# pcl_conversions as well; the explicit finds are for ${PCL_LIBRARIES} and
# the include dirs GTSAM's headers need unqualified.
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# pgo_node's loop verification (small_gicp's RegistrationPCL, since 2026-10).
# Same arrangement as syncai_localizer: src/third-party/small_gicp is a
# build_type cmake package, ordered before this one by <depend>small_gicp</depend>.
find_package(small_gicp REQUIRED)
# clean_map's ray traversal and voxel keys (2026-10). ros-humble-octomap from
# apt -- the rosdep key `octomap`, in the Dockerfile's dev stage -- rather than
# a src/third-party checkout: upstream's top level would also put octovis and
# dynamicEDT3D into every colcon build. Its config sets OCTOMAP_INCLUDE_DIRS /
# OCTOMAP_LIBRARIES. No tree is ever built (see map_cleaner.hpp).
find_package(octomap REQUIRED)

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

# ---- The post-save map cleaning: sidecar, cleaner, CLI ------------------------
# Split so pgo_node links neither OctoMap nor the ray-cast: it only writes the
# "converting" sidecar and spawns clean_map, which does the rest in its own
# process (see PGONode::startMapClean for why a process).
add_library(map_clean_recipe STATIC src/map_clean_recipe.cpp)
target_include_directories(map_clean_recipe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(map_cleaner STATIC src/map_cleaner.cpp)
target_include_directories(map_cleaner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(map_cleaner PUBLIC ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(clean_map src/clean_map_main.cpp)
target_link_libraries(clean_map map_cleaner map_clean_recipe)

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
ament_target_dependencies(pgo_node ${SYNCAI_MAPPING_DEPS} ament_index_cpp)
# small_gicp::small_gicp carries OpenMP::OpenMP_CXX as a PUBLIC usage
# requirement; without -fopenmp its omp pragmas are silently ignored and the
# kd-tree / covariance estimation run single-threaded.
target_link_libraries(pgo_node
  ${PCL_LIBRARIES}
  gtsam
  small_gicp::small_gicp
  map_clean_recipe
)

install(TARGETS pgo_node clean_map
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
  # pgos/ is ported upstream code with no per-file licence header
  # (the package LICENSE covers them), so the copyright linter would fail on
  # every one of those files; cpplint is off for the same files as it was
  # upstream. The rest of ament_lint_common still runs.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
