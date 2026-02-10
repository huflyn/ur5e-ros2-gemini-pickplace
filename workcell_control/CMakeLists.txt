cmake_minimum_required(VERSION 3.5)
project(workcell_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()