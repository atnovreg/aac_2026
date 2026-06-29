cmake_minimum_required(VERSION 3.8)
project(arduino_mega_control_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(arduino_mega_control SHARED
  src/arduino_mega_control.cpp)

ament_auto_add_executable(arduino_mega_control_node_exe
  src/arduino_mega_control_node.cpp)

target_link_libraries(arduino_mega_control_node_exe
  arduino_mega_control)

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
)
