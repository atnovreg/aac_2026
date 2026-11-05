#include "mpcc_cpp_virtual/core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  const auto track = mpcc_cpp_virtual::Track::load(argv[1]);
  const auto parameters = mpcc_cpp_virtual::RuntimeParameters::load(argv[3]);
  mpcc_cpp_virtual::Core core(track, parameters);
  mpcc_cpp_virtual::ActuatorResponse actuator(parameters.prediction_actuator, .01);
  std::ifstream fixture(argv[2]);
  if (!fixture) return 3;
  std::string line;
  std::getline(fixture, line);
  int rows = 0;
  double max_command_error = 0;
  std::vector<double> solver_times;
  std::vector<double> core_times;
  while (std::getline(fixture, line)) {
    std::stringstream stream(line);
    std::string field;
    std::vector<double> values;
    while (std::getline(stream, field, ',')) values.push_back(std::stod(field));
    constexpr int nx = mpcc_cpp_virtual::kStateDimension;
    if (values.size() != nx+4) return 4;
    std::array<double, 7> state{};
    std::array<double, 3> previous{};
    for (int i = 0; i < 7; ++i) state[i] = values[i];
    for (int i = 0; i < 3; ++i) previous[i] = values[i + 7];
    const auto result = core.step(state, previous, parameters.prediction_dt, &actuator);
    solver_times.push_back(result.solver_time_s);
    core_times.push_back(result.core_time_s);
    if (result.status != 0 || values[nx+3] != 0) {
      std::cerr << "solver status mismatch at row " << rows << '\n';
      return 5;
    }
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(result.command[i])) return 6;
      max_command_error = std::max(max_command_error,
                                   std::abs(result.command[i] - values[i + nx]));
    }
    if (rows == 0) {
      const auto& stage = core.stage(1);
      const auto expected = track.reference(
          state[6] + parameters.prediction_dt * state[3]);
      const double reference_error = std::max({
          std::abs(stage.center[0] - expected.center[0]),
          std::abs(stage.center[1] - expected.center[1]),
          std::abs(stage.tangent[0] - expected.tangent[0]),
          std::abs(stage.tangent[1] - expected.tangent[1]),
          std::abs(stage.lower_h + expected.width_right),
          std::abs(stage.upper_h - expected.width_left)});
      if (reference_error > 1e-8) return 7;
    }
    if constexpr (nx > 10) {
      for (int tick = 0; tick < std::lround(parameters.prediction_dt/.01); ++tick)
        actuator.step({values[nx], values[nx+1], values[nx+2]});
    }
    ++rows;
  }
  if (rows < (mpcc_cpp_virtual::kStateDimension > 10 ? 100 : 1000) || max_command_error > 1e-6) {
    std::cerr << "core parity rows=" << rows
              << " max_command_error=" << max_command_error << '\n';
    return 8;
  }
  std::cout << "core parity rows=" << rows
            << " max_command_error=" << max_command_error << '\n';
  const auto report = [](const char* label, std::vector<double> values) {
    std::sort(values.begin(), values.end());
    double total = 0;
    int misses_10ms = 0;
    int misses_20ms = 0;
    for (double value : values) {
      total += value;
      misses_10ms += value > .01;
      misses_20ms += value > .02;
    }
    std::cout << label << " mean_ms=" << 1000 * total / values.size()
              << " p95_ms=" << 1000 * values[static_cast<std::size_t>(.95 * (values.size() - 1))]
              << " p99_ms=" << 1000 * values[static_cast<std::size_t>(.99 * (values.size() - 1))]
              << " max_ms=" << 1000 * values.back()
              << " over_10ms=" << misses_10ms
              << " over_20ms=" << misses_20ms << '\n';
  };
  report("solver", solver_times);
  report("core", core_times);
}
