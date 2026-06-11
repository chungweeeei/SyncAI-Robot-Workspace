cmake_minimum_required(VERSION 3.8)
project(syncai_test_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(syncai_util REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  syncai_util
)

# Generate the Countdown action interface in this same package. Note that
# rosidl_generate_interfaces claims the ${PROJECT_NAME} target name, so the
# executable below must use a different name.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Countdown.action"
)

set(executable_name test_action_server)

add_executable(${executable_name}
  src/main.cpp
  src/test_action_server.cpp
)

target_include_directories(${executable_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${executable_name} ${dependencies})

# Link against the C++ typesupport of the interfaces generated above
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${executable_name} "${cpp_typesupport_target}")

install(
  TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
