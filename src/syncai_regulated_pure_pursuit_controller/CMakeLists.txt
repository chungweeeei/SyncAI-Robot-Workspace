cmake_minimum_required(VERSION 3.8)
project(syncai_regulated_pure_pursuit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(syncai_costmap_2d REQUIRED)
find_package(syncai_nav_core REQUIRED)
find_package(syncai_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
  syncai_costmap_2d
  syncai_nav_core
  syncai_util
  tf2
  tf2_geometry_msgs
  tf2_ros
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/regulated_pure_pursuit_controller.cpp
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name} ${dependencies})

pluginlib_export_plugin_description_file(syncai_nav_core controller_plugin.xml)

install(
  TARGETS ${library_name}
  EXPORT export_${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES controller_plugin.xml
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
ament_export_targets(export_${library_name})
ament_export_dependencies(${dependencies})

ament_package()
