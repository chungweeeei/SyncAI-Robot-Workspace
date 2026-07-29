cmake_minimum_required(VERSION 3.8)
project(syncai_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install shared resources
#
# maps/ and rviz/ used to be installed here as empty .gitkeep placeholders, a
# leftover of the nav2_bringup skeleton this package started from. They were
# removed: maps live in the repo-root map/ directory referenced from
# config/instances/robotNN.ini (resolved against the workspace cwd, never a
# package share dir), and no RViz config is checked in. Don't re-add them
# speculatively — a map that belongs to a robot instance does not belong in a
# package share.
#
# __pycache__ is excluded because install(DIRECTORY) copies launch/ wholesale:
# running a launch file leaves a .pyc behind in the source tree (gitignored, so
# never committed) and it would otherwise be installed into the share dir. Worse,
# the bytecode outlives its source — a stale bringup_3d.launch.cpython-312.pyc
# from the retired 3D launch file was still being installed.
install(DIRECTORY
  launch
  params
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
