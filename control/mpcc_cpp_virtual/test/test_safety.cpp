#include "mpcc_cpp_virtual/safety.hpp"

#include <array>
#include <cmath>
#include <iostream>

int main() {
  mpcc_cpp_virtual::SafetyGate gate(.5);
  const std::array<double, 7> state{0., 0., 0., 1., 0., 0., 0.};
  const std::array<double, 3> command{.2, .01, 1.};
  const auto released = gate.precheck(1., .9, false, state);
  if (released.advance || released.command != std::array<double, 3>{} ||
      released.status != mpcc_cpp_virtual::SafetyStatus::DeadmanReleased) return 1;
  const auto stale = gate.precheck(1., .4, true, state);
  if (stale.advance || stale.command != std::array<double, 3>{} ||
      stale.status != mpcc_cpp_virtual::SafetyStatus::JoyTimeout) return 2;
  auto bad_state = state;
  bad_state[3] = NAN;
  const auto nonfinite = gate.precheck(1., .9, true, bad_state);
  if (nonfinite.advance || nonfinite.command != std::array<double, 3>{} ||
      nonfinite.status != mpcc_cpp_virtual::SafetyStatus::NonfiniteState) return 3;
  if (gate.precheck(1., .9, true, state).status !=
      mpcc_cpp_virtual::SafetyStatus::Active) return 4;
  const auto failed_solver = gate.postcheck(command, 4, true, state);
  if (failed_solver.advance || failed_solver.command != std::array<double, 3>{} ||
      failed_solver.status != mpcc_cpp_virtual::SafetyStatus::SolverError) return 5;
  auto bad_command = command;
  bad_command[1] = NAN;
  const auto invalid_command = gate.postcheck(bad_command, 0, true, state);
  if (invalid_command.advance || invalid_command.command != std::array<double, 3>{} ||
      invalid_command.status != mpcc_cpp_virtual::SafetyStatus::NonfiniteCommand)
    return 6;
  const auto failed_plant = gate.postcheck(command, 0, false, state);
  if (failed_plant.advance || failed_plant.command != std::array<double, 3>{} ||
      failed_plant.status != mpcc_cpp_virtual::SafetyStatus::PlantError) return 7;
  auto bad_next = state;
  bad_next[0] = INFINITY;
  if (gate.postcheck(command, 0, true, bad_next).advance) return 8;
  bad_next = state;
  bad_next[6] = -0.01;
  const auto reverse_start = gate.postcheck(command, 0, true, bad_next);
  if (reverse_start.advance || reverse_start.command != std::array<double, 3>{} ||
      reverse_start.status != mpcc_cpp_virtual::SafetyStatus::PlantError) return 10;
  const auto good = gate.postcheck(command, 0, true, state);
  if (!good.advance || good.command != command ||
      good.status != mpcc_cpp_virtual::SafetyStatus::Active) return 9;
  std::cout << "safety gate zero output / no advancement on all failures\n";
}
