cmake_minimum_required(VERSION 3.8)
project(simple_gazebo_track)

find_package(ament_cmake REQUIRED)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/simple_gazebo_track.dsv.in"
)

install(DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
