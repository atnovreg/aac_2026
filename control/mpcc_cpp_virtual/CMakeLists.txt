cmake_minimum_required(VERSION 3.16)
project(mpcc_cpp_virtual LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(mpcc_track src/track.cpp)
target_include_directories(mpcc_track PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(MPCC_GENERATED_DIR AND ACADOS_ROOT)
  if(NOT MPCC_REFERENCE_DIR)
    message(FATAL_ERROR "Set MPCC_REFERENCE_DIR to the matching native export")
  endif()
  find_package(OpenSSL REQUIRED COMPONENTS Crypto)
  file(READ "${MPCC_REFERENCE_DIR}/manifest.json" MPCC_MANIFEST)
  foreach(KIND TRACK RUNTIME SOLVER)
    if(KIND STREQUAL "TRACK")
      set(KEY track_native_sha256)
      set(ARTIFACT "${MPCC_REFERENCE_DIR}/track_native.txt")
    elseif(KIND STREQUAL "RUNTIME")
      set(KEY runtime_params_sha256)
      set(ARTIFACT "${MPCC_REFERENCE_DIR}/runtime_params.txt")
    else()
      set(KEY solver_library_sha256)
      set(ARTIFACT "${MPCC_GENERATED_DIR}/libacados_ocp_solver_slash_mpcc.so")
    endif()
    string(JSON MPCC_${KIND}_SHA GET "${MPCC_MANIFEST}" ${KEY})
    file(SHA256 "${ARTIFACT}" ACTUAL_SHA)
    if(NOT ACTUAL_SHA STREQUAL MPCC_${KIND}_SHA)
      message(FATAL_ERROR "MPCC ${KIND} hash differs from native manifest")
    endif()
  endforeach()
  string(JSON MPCC_PREVIEW_S GET "${MPCC_MANIFEST}" prediction_time_s)
  if(NOT MPCC_PREVIEW_S STREQUAL "1.2")
    message(FATAL_ERROR "MPCC preview must be exactly 1.2 seconds")
  endif()
  configure_file(identity_constants.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/identity_constants.hpp @ONLY)
  add_library(mpcc_core src/core.cpp
    src/identity.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../acados_mpcc/c_batch_bridge.c
    ${MPCC_REFERENCE_DIR}/plant_step_10ms.c
    ${MPCC_REFERENCE_DIR}/plant_step_20ms.c)
  string(JSON MPCC_STATE_DIM ERROR_VARIABLE DIM_ERROR GET "${MPCC_MANIFEST}" state_dimension)
  if(DIM_ERROR)
    set(MPCC_STATE_DIM 10)
  endif()
  target_compile_definitions(mpcc_core PUBLIC MPCC_STATE_DIM=${MPCC_STATE_DIM})
  if(MPCC_STATE_DIM GREATER 10)
    target_sources(mpcc_core PRIVATE
      ${MPCC_REFERENCE_DIR}/prediction_step_10ms.c
      ${MPCC_REFERENCE_DIR}/prediction_step_20ms.c)
  endif()
  target_include_directories(mpcc_core PRIVATE
    ${MPCC_GENERATED_DIR} ${MPCC_REFERENCE_DIR} ${ACADOS_ROOT}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${ACADOS_ROOT}/interfaces ${ACADOS_ROOT}/external
    ${ACADOS_ROOT}/external/blasfeo/include
    ${ACADOS_ROOT}/external/hpipm/include)
  target_link_libraries(mpcc_core PUBLIC mpcc_track PRIVATE
    ${MPCC_GENERATED_DIR}/libacados_ocp_solver_slash_mpcc.so
    ${ACADOS_ROOT}/lib/libacados.so OpenSSL::Crypto ${CMAKE_DL_LIBS} m)
  set_target_properties(mpcc_core PROPERTIES BUILD_RPATH
    "${MPCC_GENERATED_DIR};${ACADOS_ROOT}/lib")
  add_executable(mpcc_offline_run src/offline_run.cpp)
  target_include_directories(mpcc_offline_run PRIVATE ${MPCC_REFERENCE_DIR})
  target_link_libraries(mpcc_offline_run PRIVATE mpcc_core)

  add_library(mpcc_safety src/safety.cpp)
  target_include_directories(mpcc_safety PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  add_executable(mpcc_virtual_node src/virtual_node.cpp)
  target_include_directories(mpcc_virtual_node PRIVATE ${MPCC_REFERENCE_DIR})
  target_link_libraries(mpcc_virtual_node mpcc_core mpcc_safety)
  ament_target_dependencies(mpcc_virtual_node
    rclcpp sensor_msgs geometry_msgs std_msgs)
endif()

if(BUILD_TESTING)
  if(NOT MPCC_REFERENCE_DIR)
    message(FATAL_ERROR "Set MPCC_REFERENCE_DIR to generated reference artifacts")
  endif()
  enable_testing()
  add_executable(test_actuator_response test/test_actuator_response.cpp)
  target_include_directories(test_actuator_response PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME actuator_response COMMAND test_actuator_response)

  add_executable(test_track test/test_track.cpp)
  target_link_libraries(test_track PRIVATE mpcc_track)
  add_test(NAME track_parity COMMAND test_track
    ${MPCC_REFERENCE_DIR}/track_native.txt
    ${MPCC_REFERENCE_DIR}/track_reference.csv)

  add_executable(test_plant test/test_plant.cpp
    ${MPCC_REFERENCE_DIR}/plant_step_10ms.c)
  target_include_directories(test_plant PRIVATE ${MPCC_REFERENCE_DIR})
  target_link_libraries(test_plant PRIVATE m)
  add_test(NAME plant_parity COMMAND test_plant
    ${MPCC_REFERENCE_DIR}/plant_reference.csv)

  add_executable(test_safety test/test_safety.cpp)
  target_link_libraries(test_safety PRIVATE mpcc_safety)
  add_test(NAME safety_gate COMMAND test_safety)

  add_executable(test_solution_validation test/test_solution_validation.cpp)
  target_include_directories(test_solution_validation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME solution_validation COMMAND test_solution_validation)

  add_executable(test_lap test/test_lap.cpp)
  target_include_directories(test_lap PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME lap_rollover COMMAND test_lap)

  if(TARGET mpcc_core)
    add_executable(test_diagnostics test/test_diagnostics.cpp)
    target_link_libraries(test_diagnostics PRIVATE mpcc_core)
    add_test(NAME diagnostics_snapshot COMMAND test_diagnostics
      ${MPCC_REFERENCE_DIR}/track_native.txt
      ${MPCC_REFERENCE_DIR}/runtime_params.txt)
    set_tests_properties(diagnostics_snapshot PROPERTIES ENVIRONMENT
      "LD_LIBRARY_PATH=${MPCC_GENERATED_DIR}:${ACADOS_ROOT}/lib")
    if(MPCC_STATE_DIM GREATER 10)
      add_executable(test_prediction test/test_prediction.cpp)
      target_include_directories(test_prediction PRIVATE ${MPCC_REFERENCE_DIR})
      target_link_libraries(test_prediction PRIVATE mpcc_core)
      add_test(NAME actuator_prediction_parity COMMAND test_prediction
        ${MPCC_REFERENCE_DIR}/runtime_params.txt)
      set_tests_properties(actuator_prediction_parity PROPERTIES ENVIRONMENT
        "LD_LIBRARY_PATH=${MPCC_GENERATED_DIR}:${ACADOS_ROOT}/lib")
    endif()
    add_executable(test_identity test/test_identity.cpp)
    target_link_libraries(test_identity PRIVATE mpcc_core)
    add_test(NAME artifact_identity COMMAND test_identity
      ${MPCC_REFERENCE_DIR}/track_native.txt
      ${MPCC_REFERENCE_DIR}/runtime_params.txt)
    set_tests_properties(artifact_identity PROPERTIES ENVIRONMENT
      "LD_LIBRARY_PATH=${MPCC_GENERATED_DIR}:${ACADOS_ROOT}/lib")
    add_executable(test_core test/test_core.cpp)
    target_link_libraries(test_core PRIVATE mpcc_core)
    add_test(NAME core_parity COMMAND test_core
      ${MPCC_REFERENCE_DIR}/track_native.txt
      ${MPCC_REFERENCE_DIR}/python_replay.csv
      ${MPCC_REFERENCE_DIR}/runtime_params.txt)
    set_tests_properties(core_parity PROPERTIES ENVIRONMENT
      "LD_LIBRARY_PATH=${MPCC_GENERATED_DIR}:${ACADOS_ROOT}/lib")
    add_executable(test_warmstart test/test_warmstart.cpp)
    target_link_libraries(test_warmstart PRIVATE mpcc_core)
    add_test(NAME warmstart_10ms COMMAND test_warmstart
      ${MPCC_REFERENCE_DIR}/track_native.txt
      ${MPCC_REFERENCE_DIR}/python_replay.csv
      ${MPCC_REFERENCE_DIR}/runtime_params.txt)
    set_tests_properties(warmstart_10ms PROPERTIES ENVIRONMENT
      "LD_LIBRARY_PATH=${MPCC_GENERATED_DIR}:${ACADOS_ROOT}/lib")
  endif()
endif()

install(TARGETS mpcc_track ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
if(TARGET mpcc_core)
  install(TARGETS mpcc_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
  install(TARGETS mpcc_offline_run RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(TARGETS mpcc_safety ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
  install(TARGETS mpcc_virtual_node RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()
install(DIRECTORY include/ DESTINATION include)
ament_package()
