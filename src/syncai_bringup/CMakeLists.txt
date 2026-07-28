cmake_minimum_required(VERSION 3.8)
project(syncai_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install shared resources
#
# maps/ and rviz/ hold nothing but .gitkeep — they are installed anyway so that
# dropping a map or an rviz config in works without touching this file.
#
# __pycache__ is excluded because install(DIRECTORY) copies launch/ wholesale:
# running a launch file leaves a .pyc behind in the source tree (gitignored, so
# never committed) and it would otherwise be installed into the share dir. Worse,
# the bytecode outlives its source — a stale bringup_3d.launch.cpython-312.pyc
# from the retired 3D launch file was still being installed.
install(DIRECTORY
  launch
  params
  maps
  rviz
  description
  meshes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
