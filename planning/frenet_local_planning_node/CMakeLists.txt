cmake_minimum_required(VERSION 3.8)
project(frenet_local_planning_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)

include_directories(
  include
)

ament_auto_add_library(frenet_local_planning SHARED
  src/frenet_local_planning.cpp
  src/frenet_utils.cc
  src/motion_primitive_generate/cubic_spliner.cc
  src/motion_primitive_generate/cubic_spliner_2D.cc
  src/motion_primitive_generate/frenet_path.cc
  src/motion_primitive_generate/frenet_path_generator.cc
  src/motion_primitive_generate/quartic_polynomial.cc
  src/motion_primitive_generate/quintic_polynomial.cc
)

target_link_libraries(frenet_local_planning
  ${OpenCV_LIBS}
)

ament_auto_add_executable(frenet_local_planning_exe
  src/frenet_local_planning_node.cpp
)

target_link_libraries(frenet_local_planning_exe
  frenet_local_planning
)

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
