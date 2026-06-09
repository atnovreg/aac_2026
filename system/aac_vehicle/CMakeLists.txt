cmake_minimum_required(VERSION 3.8)
project(aac_vehicle)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(vehicle_state_publisher SHARED
  src/vehicle_state_publisher.cpp)

ament_auto_add_executable(vehicle_state_publisher_exe
  src/vehicle_state_publisher_node.cpp)

target_link_libraries(vehicle_state_publisher_exe
  vehicle_state_publisher)

ament_auto_add_executable(gazebo_ground_truth_publisher
  src/gazebo_ground_truth_publisher.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
  urdf
)
