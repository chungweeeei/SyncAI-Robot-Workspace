# Offline PCD -> OctoMap tools for a pgo save directory. Not an ament package and
# not part of the colcon build: OctoMap is not in the robot image, so these are
# built by hand in a throwaway container. README.md beside this file has the
# whole procedure.
cmake_minimum_required(VERSION 3.10)
project(syncai_octomap_tools CXX)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(octomap REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

add_executable(pcd2octomap pcd2octomap.cpp)
target_include_directories(pcd2octomap PRIVATE ${OCTOMAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd2octomap ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen)

add_executable(slice slice.cpp)
target_include_directories(slice PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(slice ${OCTOMAP_LIBRARIES})

add_executable(export_ply export_ply.cpp)
target_include_directories(export_ply PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(export_ply ${OCTOMAP_LIBRARIES})
