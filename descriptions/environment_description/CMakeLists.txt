cmake_minimum_required(VERSION 3.5)
project(environment_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
