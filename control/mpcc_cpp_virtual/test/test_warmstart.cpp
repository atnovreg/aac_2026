#include "mpcc_cpp_virtual/core.hpp"

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
  std::getline(fixture, line);
  std::stringstream stream(line);
  std::string field;
  std::vector<double> values;
  while (std::getline(stream, field, ',')) values.push_back(std::stod(field));
  if (values.size() != mpcc_cpp_virtual::kStateDimension+4) return 4;
  std::array<double, 7> state{};
  std::array<double, 3> previous{};
  for (int i = 0; i < 7; ++i) state[i] = values[i];
  for (int i = 0; i < 3; ++i) previous[i] = values[7 + i];
  const auto first = core.step(state, previous, parameters.prediction_dt, &actuator);
  if (first.status || core.solve_count() != 1) return 5;
  const auto old1 = core.prediction_state(1);
  const auto old2 = core.prediction_state(2);
  const auto old_terminal = core.prediction_state(core.horizon());
  const auto old_u1 = core.prediction_increment(1);
  const auto old_u2 = core.prediction_increment(2);
  const auto second = core.step(state, first.command, .01, &actuator);
  if (second.status || core.solve_count() != 2) return 6;
  const double alpha = .01 / parameters.prediction_dt;
  if (std::abs(second.command[0] - first.command[0]) >
          parameters.input_scale[0] * parameters.increment_limit[0] * alpha + 1e-8 ||
      std::abs(second.command[1] - first.command[1]) >
          parameters.input_scale[1] * parameters.increment_limit[1] * alpha + 1e-8)
    return 10;
  const auto warm1 = core.last_warm_state(1);
  for (int j = 0; j < mpcc_cpp_virtual::kStateDimension; ++j)
    if (std::abs(warm1[j] - ((1 - alpha) * old1[j] + alpha * old2[j])) >
        1e-10) return 7;
  const auto warm_increment = core.last_warm_increment(1);
  for (int j = 0; j < 3; ++j)
    if (std::abs(warm_increment[j] - ((1 - alpha) * old_u1[j] +
                                      alpha * old_u2[j])) > 1e-10)
      return 8;
  const auto warm_terminal = core.last_warm_state(core.horizon());
  if (std::abs(warm_terminal[6] - (old_terminal[6] + .01 * old_terminal[9])) >
      1e-9) return 9;
  if constexpr (mpcc_cpp_virtual::kStateDimension > 10) {
    actuator.step(first.command);
    const auto memory = actuator.prediction_state<mpcc_cpp_virtual::kStateDimension-10>();
    core.reset_prediction();
    const auto resumed = core.step(state, first.command, .01, &actuator);
    if (resumed.status) return 11;
    const auto reset_warm = core.last_warm_state(1);
    for (std::size_t j = 0; j < memory.size(); ++j)
      if (std::abs(reset_warm[10+j] - memory[j]) > 1e-12) return 12;
  }
  std::cout << "10ms fractional warm start and two fresh solves passed\n";
}
