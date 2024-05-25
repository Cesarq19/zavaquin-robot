cmake_minimum_required(VERSION 3.8)
project(zavaquin_gazebo)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY config launch worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()