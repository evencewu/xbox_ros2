cmake_minimum_required(VERSION 3.8)
project(xbox_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/XboxControl.msg"

  DEPENDENCIES
    sensor_msgs
    std_msgs
    geometry_msgs
)

ament_auto_package(
  INSTALL_TO_SHARE
)
