#include "mpcc_cpp_virtual/core.hpp"
#include "mpcc_cpp_virtual/actuator_response.hpp"
#include "mpcc_cpp_virtual/identity.hpp"
#include "mpcc_cpp_virtual/lap.hpp"
#include "plant_step_10ms.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
double elapsed_ms(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
}

bool finite(const std::array<double, 7>& value) {
  for (double item : value) if (!std::isfinite(item)) return false;
  return true;
}
}

int main(int argc, char** argv) {
  if (argc < 4 || argc > 7) {
    std::cerr << "usage: offline_run TRACK_NATIVE RUNTIME_PARAMS OUTPUT_CSV [ACTUATOR_CONFIG] [LAPS] [DIAGNOSTIC_PREFIX]\n";
    return 2;
  }
  try {
    mpcc_cpp_virtual::verify_runtime_artifacts(argv[1], argv[2]);
    const auto track = mpcc_cpp_virtual::Track::load(argv[1]);
    const auto parameters = mpcc_cpp_virtual::RuntimeParameters::load(argv[2]);
    std::size_t consumed = 0;
    const int laps = argc >= 6 ? std::stoi(argv[5], &consumed) : 1;
    if (argc >= 6 && consumed != std::string(argv[5]).size())
      throw std::invalid_argument("invalid lap count");
    if (laps < 1 || laps > 10) throw std::invalid_argument("laps must be 1..10");
    mpcc_cpp_virtual::LapCounter lap_counter(track.length());
    mpcc_cpp_virtual::Core core(track, parameters);
    mpcc_cpp_virtual::ActuatorResponse actuator(
        mpcc_cpp_virtual::ActuatorParameters::load(argc >= 5 ? argv[4] : ""), .01);
    const auto initial = track.reference(0.);
    std::array<double, 7> state{initial.center[0], initial.center[1],
        std::atan2(initial.tangent[1], initial.tangent[0]),
        parameters.initial_speed, 0., 0., 0.};
    std::array<double, 3> previous{};
    std::ofstream output(argv[3]);
    if (!output) throw std::runtime_error("cannot write output trajectory");
    output << std::setprecision(17);
    output << "step,t,x,y,yaw,vx,vy,yaw_rate,progress,current,steering,vtheta,"
              "lateral_offset,corridor_violation,solver_ms,core_ms,plant_ms,"
              "loop_ms,status,applied_drive,applied_steering,applied_vtheta,"
              "drive_endpoint,steering_endpoint,completed_laps,cumulative_progress\n";
    int completed = 0;
    double max_corridor_violation = 0;
    double max_speed = 0;
    int lap_count = 0;
    for (int step = 0; step < 10000 * laps; ++step) {
      const auto tick_start = std::chrono::steady_clock::now();
      const auto result = core.step(state, previous, .01, &actuator);
      if (result.status) {
        std::cerr << "solver failed step=" << step << " status=" << result.status << '\n';
        if (argc == 7 && core.diagnostics_available()) core.write_diagnostics(argv[6]);
        else if (argc == 7) std::cerr << "No QP generated for the rejected step\n";
        return 10;
      }
      for (double value : result.command)
        if (!std::isfinite(value)) throw std::runtime_error("nonfinite command");
      std::array<double, 7> next{};
      double* results[] = {next.data()};
      casadi_real work[slash_plant_step_10ms_SZ_W]{};
      const auto plant_start = std::chrono::steady_clock::now();
      const auto applied = actuator.step(result.command);
      const double* arguments[] = {state.data(), applied.data()};
      if (slash_plant_step_10ms(arguments, results, nullptr, work, 0) || !finite(next))
        throw std::runtime_error("virtual plant failed");
      const double plant_ms = elapsed_ms(plant_start);
      const double projected = track.project({next[0], next[1]}, state[6]);
      if (std::abs(projected - state[6]) > .25)
        throw std::runtime_error("unexpected physical progress jump");
      next[6] = projected;
      const auto reference = track.reference(projected);
      const double offset = (next[0] - reference.center[0]) *
                                (-reference.tangent[1]) +
                            (next[1] - reference.center[1]) *
                                reference.tangent[0];
      const double corridor_violation = std::max(
          {0., offset - reference.width_left, -offset - reference.width_right});
      max_corridor_violation = std::max(max_corridor_violation, corridor_violation);
      max_speed = std::max(max_speed, next[3]);
      if (laps > 1) {
        if (lap_counter.wrap(next)) core.reset_prediction();
        lap_count = lap_counter.count();
      } else if (projected >= track.length()) lap_count = 1;
      const double loop_ms = elapsed_ms(tick_start);
      output << step << ',' << (step + 1) * .01;
      for (double value : next) output << ',' << value;
      for (double value : result.command) output << ',' << value;
      output << ',' << offset << ',' << corridor_violation << ','
             << result.solver_time_s * 1000 << ',' << result.core_time_s * 1000
             << ',' << plant_ms << ',' << loop_ms << ',' << result.status;
      for (double value : applied) output << ',' << value;
      for (double value : actuator.endpoint()) output << ',' << value;
      output << ',' << lap_count << ',' << (laps > 1 ? lap_counter.cumulative_progress(next[6]) : next[6]);
      output << '\n';
      state = next;
      previous = result.command;
      completed = step + 1;
      if (corridor_violation > .002) {
        std::cerr << "corridor violation step=" << step
                  << " violation_m=" << corridor_violation << '\n';
        return 11;
      }
      if (lap_count >= laps) break;
    }
    output.flush();
    std::cout << "steps=" << completed << " simulated_s=" << .01 * completed
              << " progress_m=" << state[6] << " route_m=" << track.length()
              << " max_corridor_violation_m=" << max_corridor_violation
              << " max_vx_m_s=" << max_speed << '\n';
    std::cout << "completed_laps=" << lap_count << " target_laps=" << laps << '\n';
    if (lap_count < laps) return 12;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "offline run failed: " << error.what() << '\n';
    return 13;
  }
}
