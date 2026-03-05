cmake_minimum_required(VERSION 3.8)
project(color_detection_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Pakete finden (In ROS 2 ist das ament und rosidl)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 2. Nachrichten generieren
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegoBrick.msg"
  "srv/DetectBricks.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# 3. Exportieren
ament_export_dependencies(rosidl_default_runtime)
ament_package()