cmake_minimum_required(VERSION 3.8)
project(syncai_task_runner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# --- Scaffold ---------------------------------------------------------------
# Add dependencies and targets here as the task runner takes shape. Typical setup:
#
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
#
# set(dependencies rclcpp rclcpp_action)
#
# add_executable(task_runner src/task_runner.cpp)
# target_include_directories(task_runner PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(task_runner ${dependencies})
# install(TARGETS task_runner DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY include/ DESTINATION include)
# ----------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
