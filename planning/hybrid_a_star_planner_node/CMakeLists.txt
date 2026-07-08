cmake_minimum_required(VERSION 3.8)
project(hybrid_a_star_planner_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(ompl REQUIRED)

include_directories(include)

ament_auto_add_library(hybrid_a_star_planner SHARED
  src/hybrid_a_star_planner.cpp
)

target_include_directories(hybrid_a_star_planner SYSTEM PUBLIC
  ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(hybrid_a_star_planner
  ${OMPL_LIBRARIES}
)

ament_auto_add_executable(hybrid_a_star_planner_exe
  src/hybrid_a_star_planner_node.cpp
)

target_link_libraries(hybrid_a_star_planner_exe
  hybrid_a_star_planner
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
