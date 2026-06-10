cmake_minimum_required(VERSION 3.8)
project(syncai_amcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(syncai_util REQUIRED)

include_directories(include)

# Detect drand48 for portable_utils.hpp
include(CheckSymbolExists)
check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)

set(dependencies
  rclcpp
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  nav2_msgs
  syncai_util
)

set(library_name ${PROJECT_NAME}_core)

# Algorithm core (portable C/C++) + ROS node, all in one library.
add_library(${library_name} SHARED
  # particle filter (C)
  src/pf/pf.c
  src/pf/pf_kdtree.c
  src/pf/pf_pdf.c
  src/pf/pf_vector.c
  src/pf/eig3.c
  # map (C + C++)
  src/map/map.c
  src/map/map_range.c
  src/map/map_cspace.cpp
  # motion models (C++)
  src/motion_model/differential_motion_model.cpp
  src/motion_model/omni_motion_model.cpp
  # laser sensor models (C++)
  src/sensors/laser/laser.cpp
  src/sensors/laser/beam_model.cpp
  src/sensors/laser/likelihood_field_model.cpp
  src/sensors/laser/likelihood_field_model_prob.cpp
  # ROS node
  src/amcl_node.cpp
)

if(HAVE_DRAND48)
  target_compile_definitions(${library_name} PRIVATE "HAVE_DRAND48")
endif()

target_link_libraries(${library_name} m)
ament_target_dependencies(${library_name} ${dependencies})

set(executable_name amcl)
add_executable(${executable_name} src/main.cpp)
target_link_libraries(${executable_name} ${library_name})
ament_target_dependencies(${executable_name} ${dependencies})

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch params
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
ament_export_dependencies(${dependencies})

ament_package()
