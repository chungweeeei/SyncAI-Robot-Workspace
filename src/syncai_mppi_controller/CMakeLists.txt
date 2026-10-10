cmake_minimum_required(VERSION 3.8)
project(syncai_mppi_controller)

# Port of nav2_mppi_controller (humble); the build flags below are upstream's.
# xtensor evaluates the optimizer's batch maths through xsimd: SSE/AVX on x86,
# NEON on the Jetson (arm64), where the x86 flags further down fail
# check_cxx_compiler_flag and drop out on their own.
add_definitions(-DXTENSOR_ENABLE_XSIMD)
add_definitions(-DXTENSOR_USE_XSIMD)

set(XTENSOR_USE_TBB 0)
set(XTENSOR_USE_OPENMP 0)
set(XTENSOR_USE_XSIMD 1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)

set(dependencies
  angles
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
  visualization_msgs
)

foreach(pkg IN LISTS dependencies)
  find_package(${pkg} REQUIRED)
endforeach()

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-mno-avx512f" COMPILER_SUPPORTS_AVX512)
check_cxx_compiler_flag("-msse4.2" COMPILER_SUPPORTS_SSE4)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_AVX512)
  add_compile_options(-mno-avx512f)
endif()

if(COMPILER_SUPPORTS_SSE4)
  add_compile_options(-msse4.2)
endif()

if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()

if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()

# Not -march=native: the image is built on a workstation as well as on the
# robot, and an artefact tuned for one CPU must not land in the shared install/.
add_compile_options(-O3 -finline-limit=10000000 -ffp-contract=fast -ffast-math -mtune=generic)

# Library names carry the syncai_ prefix (upstream: mppi_controller /
# mppi_critics) so they can never be confused with an apt nav2 install's
# libmppi_controller.so on the same LD_LIBRARY_PATH.
add_library(${PROJECT_NAME} SHARED
  src/controller.cpp
  src/optimizer.cpp
  src/critic_manager.cpp
  src/trajectory_visualizer.cpp
  src/path_handler.cpp
  src/parameters_handler.cpp
  src/noise_generator.cpp
)

add_library(syncai_mppi_critics SHARED
  src/critics/obstacles_critic.cpp
  src/critics/cost_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_align_legacy_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/prefer_forward_critic.cpp
  src/critics/twirling_critic.cpp
  src/critics/constraint_critic.cpp
  src/critics/velocity_deadband_critic.cpp
)

set(libraries ${PROJECT_NAME} syncai_mppi_critics)

foreach(lib IN LISTS libraries)
  target_compile_options(${lib} PUBLIC -fconcepts)
  target_include_directories(${lib} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${xsimd_INCLUDE_DIRS}
  )
  target_link_libraries(${lib} xtensor xtensor::optimize xtensor::use_xsimd)
  ament_target_dependencies(${lib} ${dependencies})
endforeach()

install(TARGETS ${libraries}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies} xtensor xsimd)
pluginlib_export_plugin_description_file(syncai_nav_core mppic.xml)
pluginlib_export_plugin_description_file(syncai_mppi_controller critics.xml)

ament_package()
