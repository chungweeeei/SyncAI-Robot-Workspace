cmake_minimum_required(VERSION 3.8)
project(syncai_map_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(syncai_util REQUIRED)

# GraphicsMagick++ (Magick++) has no ament/CMake config, locate it via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GRAPHICSMAGICKCPP REQUIRED GraphicsMagick++)

set(dependencies
  rclcpp
  rclcpp_components
  nav_msgs
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  syncai_util
)

include_directories(include)

# --- core library: map IO + map mode ---
add_library(map_io_core SHARED
  src/map_io.cpp
  src/map_mode.cpp
)
target_include_directories(map_io_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GRAPHICSMAGICKCPP_INCLUDE_DIRS}
)
ament_target_dependencies(map_io_core ${dependencies} Eigen3)
target_link_libraries(map_io_core
  yaml-cpp
  ${GRAPHICSMAGICKCPP_LIBRARIES}
)

# --- map_server component + executable ---
add_library(map_server_core SHARED src/map_server/map_server.cpp)
ament_target_dependencies(map_server_core ${dependencies})
target_link_libraries(map_server_core map_io_core yaml-cpp)
rclcpp_components_register_nodes(map_server_core "syncai_map_server::MapServer")

add_executable(map_server src/map_server/main.cpp)
ament_target_dependencies(map_server ${dependencies})
target_link_libraries(map_server map_server_core)

# --- map_saver component + executable ---
add_library(map_saver_core SHARED src/map_saver/map_saver.cpp)
ament_target_dependencies(map_saver_core ${dependencies})
target_link_libraries(map_saver_core map_io_core)
rclcpp_components_register_nodes(map_saver_core "syncai_map_server::MapSaver")

add_executable(map_saver src/map_saver/main.cpp)
ament_target_dependencies(map_saver ${dependencies})
target_link_libraries(map_saver map_saver_core)

# --- install ---
install(TARGETS
  map_io_core
  map_server_core
  map_saver_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  map_server
  map_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(map_io_core map_server_core map_saver_core)
ament_export_dependencies(${dependencies})

ament_package()
