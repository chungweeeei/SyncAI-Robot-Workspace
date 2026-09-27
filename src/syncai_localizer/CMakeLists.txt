cmake_minimum_required(VERSION 3.8)
project(syncai_localizer)

# A rough + refine GICP round is 30-50 ms at -O3 and has to fit inside the
# 200 ms period of update_hz 5.0 while sharing the CPU with pointlio; an
# unoptimised build does not. The upstream CMakeLists forced CMAKE_BUILD_TYPE
# to Release unconditionally, which also silently defeated an explicit
# `--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`; the guard keeps the default
# and lets a deliberate choice through (same as syncai_pointlio / syncai_mapping).
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
# Relocalize / IsValid, the two services served here. They were already in
# syncai_common (SyncAI-Robot-Interface) before this port -- they moved there
# from the fork's `interface` package with the hba port, because the backend,
# their only caller, builds against syncai_common.
find_package(syncai_common REQUIRED)

# PCL / Eigen come in through pcl_conversions' exported include dirs as well;
# the explicit finds are for ${PCL_LIBRARIES} and the unqualified Eigen
# includes in localizers/. small_gicp is ICPLocalizer's registration backend:
# src/third-party/small_gicp is a package with build_type cmake, so colcon
# builds it first (ordered by <depend>small_gicp</depend> in package.xml) and
# puts its lib/cmake/small_gicp on CMAKE_PREFIX_PATH.
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(small_gicp REQUIRED)

add_executable(localizer_node
  src/main.cpp
  src/localizer_node.cpp
  src/localizers/icp_localizer.cpp
)
target_include_directories(localizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})
ament_target_dependencies(localizer_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters
  tf2_ros
  pcl_conversions
  syncai_common
)
# RegistrationPCL itself is header-only (pcl_registration.hpp includes its own
# impl at the end); linking small_gicp::small_gicp is for the include path and
# for the Eigen3::Eigen and OpenMP::OpenMP_CXX usage requirements it carries as
# PUBLIC -- without -fopenmp, small_gicp's omp pragmas are silently ignored and
# KD-tree construction and the reductions all fall back to a single thread
# (same result, just slower, and num_threads becomes a no-op).
target_link_libraries(localizer_node
  ${PCL_LIBRARIES}
  small_gicp::small_gicp
)

install(TARGETS localizer_node
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
  # localizers/ is ported upstream-lineage code with no per-file licence header
  # (the package LICENSE covers it), so the copyright linter would fail on every
  # one of those files; cpplint is off for the same files as it was upstream.
  # The rest of ament_lint_common still runs.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
