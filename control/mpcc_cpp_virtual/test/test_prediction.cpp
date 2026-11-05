#include "mpcc_cpp_virtual/core.hpp"
#include "plant_step_10ms.h"
#include "prediction_step_20ms.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  const auto p = mpcc_cpp_virtual::RuntimeParameters::load(argv[1]);
  mpcc_cpp_virtual::ActuatorResponse actuator(p.prediction_actuator, .01);
  for (int i = 0; i < 7; ++i) actuator.step({.1*i, -.02*i, 2.});
  mpcc_cpp_virtual::AugmentedState x{};
  x[3] = 3.; x[4] = .1; x[5] = .2; x[6] = 2.;
  const auto memory = actuator.prediction_state<mpcc_cpp_virtual::kStateDimension-10>();
  std::copy(memory.begin(), memory.end(), x.begin()+10);
  const std::array<double, 3> command{-.3, .15, 3.};
  mpcc_cpp_virtual::AugmentedState predicted{};
  const double* args[] = {x.data(), command.data()};
  double* output[] = {predicted.data()};
  casadi_real work[slash_prediction_step_20ms_SZ_W]{};
  if (slash_prediction_step_20ms(args, output, nullptr, work, 0)) return 3;
  std::array<double, 7> expected{};
  std::copy_n(x.begin(), 7, expected.begin());
  for (int i = 0; i < std::lround(p.prediction_dt/.01); ++i) {
    const auto applied = actuator.step(command);
    std::array<double, 7> next{};
    const double* plant_args[] = {expected.data(), applied.data()};
    double* plant_output[] = {next.data()};
    casadi_real plant_work[slash_plant_step_10ms_SZ_W]{};
    if (slash_plant_step_10ms(plant_args, plant_output, nullptr, plant_work, 0)) return 4;
    expected = next;
  }
  double error = 0;
  for (int i = 0; i < 7; ++i) error = std::max(error, std::abs(predicted[i]-expected[i]));
  for (int i = 0; i < 3; ++i) error = std::max(error, std::abs(predicted[i+7]-command[i]));
  const auto expected_memory = actuator.prediction_state<mpcc_cpp_virtual::kStateDimension-10>();
  for (std::size_t i = 0; i < expected_memory.size(); ++i)
    error = std::max(error, std::abs(predicted[i+10]-expected_memory[i]));
  // A cheaper RK4 prediction may differ from the four-substep virtual plant;
  // its local state error must remain below 2 microunits in this fixture.
  if (!std::isfinite(error) || error > 2e-6) return 5;
  std::cout << "prediction vs independently sequenced C++ delayed plant max_error=" << error << '\n';
}
