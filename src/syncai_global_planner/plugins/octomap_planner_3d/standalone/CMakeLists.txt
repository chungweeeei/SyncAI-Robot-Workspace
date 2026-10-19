# Builds octomap_planner_3d_cli WITHOUT ROS, for running the surface map and
# the search offline against a map's octomap.bt on a workstation (homebrew /
# apt octomap is enough). The ROS build (../../../CMakeLists.txt) compiles
# the same sources into the planner library and the same CLI; this file is
# only the no-ROS entry point and is not a colcon package (no package.xml, and
# colcon stops at the syncai_global_planner root anyway).
#
#   cmake -S plugins/octomap_planner_3d/standalone -B /tmp/octo3d -DCMAKE_PREFIX_PATH=/opt/homebrew
#   cmake --build /tmp/octo3d -j
#   /tmp/octo3d/octomap_planner_3d_cli map/<name>/octomap.bt --stats
cmake_minimum_required(VERSION 3.16)
project(octomap_planner_3d_standalone CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(octomap REQUIRED)

set(core_dir ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(include_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../../include)

add_executable(octomap_planner_3d_cli
  ${core_dir}/octomap_planner_3d_cli.cpp
  ${core_dir}/surface_map.cpp
  ${core_dir}/surface_astar.cpp
  ${core_dir}/path_postprocess.cpp
  ${core_dir}/stair_links.cpp
)
target_include_directories(octomap_planner_3d_cli PRIVATE ${include_dir})
target_include_directories(octomap_planner_3d_cli SYSTEM PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(octomap_planner_3d_cli ${OCTOMAP_LIBRARIES})
target_compile_options(octomap_planner_3d_cli PRIVATE -O3 -Wall -Wextra -Wpedantic)

# The package's gtest, when GoogleTest is installed (brew install googletest /
# apt libgtest-dev): the same test the ROS build runs with colcon test, so a
# change to the core can be checked on the workstation before the robot.
#   cmake --build /tmp/octo3d --target test_surface_map && /tmp/octo3d/test_surface_map
find_package(GTest QUIET)
if(GTest_FOUND)
  add_executable(test_surface_map
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../test/test_surface_map.cpp
    ${core_dir}/surface_map.cpp
    ${core_dir}/surface_astar.cpp
    ${core_dir}/path_postprocess.cpp
    ${core_dir}/stair_links.cpp
  )
  target_include_directories(test_surface_map PRIVATE ${include_dir})
  target_include_directories(test_surface_map SYSTEM PRIVATE ${OCTOMAP_INCLUDE_DIRS})
  target_link_libraries(test_surface_map ${OCTOMAP_LIBRARIES} GTest::gtest_main)
endif()
