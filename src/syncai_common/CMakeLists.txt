cmake_minimum_required(VERSION 3.8)
project(syncai_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare shared interfaces here. Drop files under msg/ srv/ action/, list them
# below, and uncomment. Add any message dependencies via DEPENDENCIES. Example:
#
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Example.msg"
#   "srv/Example.srv"
#   "action/Example.action"
#   DEPENDENCIES builtin_interfaces geometry_msgs
# )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
