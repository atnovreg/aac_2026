cmake_minimum_required(VERSION 3.8)
project(parking_lines_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(parking_lines_publisher SHARED
  src/parking_lines_publisher.cpp
)

ament_auto_add_executable(parking_lines_publisher_exe
  src/parking_lines_publisher_node.cpp
)

target_link_libraries(parking_lines_publisher_exe
  parking_lines_publisher
)

# Install generated track_line_export assets so the default CSV works from install space.
set(TRACK_LINE_EXPORT_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../track_line_export")

if(EXISTS "${TRACK_LINE_EXPORT_DIR}")
  install(DIRECTORY "${TRACK_LINE_EXPORT_DIR}/"
    DESTINATION share/${PROJECT_NAME}/data/track_line_export
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE)
else()
  message(WARNING
    "track_line_export directory not found at ${TRACK_LINE_EXPORT_DIR}. "
    "Set the csv_path parameter manually at runtime.")
endif()

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
