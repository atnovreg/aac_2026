cmake_minimum_required(VERSION 3.8)
project(mppi_planner_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(mppi_planner SHARED
  src/mppi_planner.cpp
)

ament_auto_add_executable(mppi_planner_exe
  src/mppi_planner_node.cpp
)

target_link_libraries(mppi_planner_exe
  mppi_planner
)

install(PROGRAMS
  scripts/mppi_analyze_log.py
  scripts/mppi_compare_trials.py
  scripts/mppi_corner_probe.py
  scripts/mppi_drive_trial.py
  scripts/mppi_log_debug.py
  scripts/mppi_pre_drive_check.py
  scripts/mppi_profile_sweep.py
  scripts/mppi_runtime_check.py
  scripts/mppi_suggest_tune.py
  scripts/mppi_tune.py
  scripts/mppi_visualization_check.py
  scripts/mppi_watch.py
  DESTINATION lib/${PROJECT_NAME}
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
