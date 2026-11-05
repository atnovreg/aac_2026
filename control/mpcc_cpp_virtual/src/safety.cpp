#include "mpcc_cpp_virtual/safety.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mpcc_cpp_virtual {
namespace {
template <std::size_t N>
bool finite(const std::array<double, N>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}
}

const char* status_name(SafetyStatus status) {
  switch (status) {
    case SafetyStatus::Active: return "active";
    case SafetyStatus::DeadmanReleased: return "deadman_released";
    case SafetyStatus::JoyTimeout: return "joy_timeout";
    case SafetyStatus::NonfiniteState: return "nonfinite_state";
    case SafetyStatus::SolverError: return "solver_error";
    case SafetyStatus::NonfiniteCommand: return "nonfinite_command";
    case SafetyStatus::PlantError: return "plant_error";
  }
  return "invalid_status";
}

SafetyGate::SafetyGate(double joy_timeout_s) : joy_timeout_s_(joy_timeout_s) {
  if (!std::isfinite(joy_timeout_s_) || joy_timeout_s_ <= 0)
    throw std::invalid_argument("invalid joystick timeout");
}

GateDecision SafetyGate::precheck(double now_s, double last_joy_s, bool pressed,
                                   const std::array<double, 7>& state) const {
  if (!finite(state)) return {SafetyStatus::NonfiniteState, {}, false};
  if (!std::isfinite(now_s) || !std::isfinite(last_joy_s) ||
      now_s < last_joy_s || now_s - last_joy_s > joy_timeout_s_)
    return {SafetyStatus::JoyTimeout, {}, false};
  if (!pressed) return {SafetyStatus::DeadmanReleased, {}, false};
  return {SafetyStatus::Active, {}, false};
}

GateDecision SafetyGate::postcheck(const std::array<double, 3>& command,
                                    int solver_status, bool plant_ok,
                                    const std::array<double, 7>& next_state) const {
  if (solver_status) return {SafetyStatus::SolverError, {}, false};
  if (!finite(command)) return {SafetyStatus::NonfiniteCommand, {}, false};
  if (!plant_ok || !finite(next_state) || next_state[6] < 0)
    return {SafetyStatus::PlantError, {}, false};
  return {SafetyStatus::Active, command, true};
}

}  // namespace mpcc_cpp_virtual
