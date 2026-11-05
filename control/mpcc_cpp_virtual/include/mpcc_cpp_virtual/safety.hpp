#pragma once

#include <array>

namespace mpcc_cpp_virtual {

enum class SafetyStatus {
  Active,
  DeadmanReleased,
  JoyTimeout,
  NonfiniteState,
  SolverError,
  NonfiniteCommand,
  PlantError,
};

const char* status_name(SafetyStatus status);

struct GateDecision {
  SafetyStatus status = SafetyStatus::SolverError;
  std::array<double, 3> command{};
  bool advance = false;
};

class SafetyGate {
 public:
  explicit SafetyGate(double joy_timeout_s);
  GateDecision precheck(double now_s, double last_joy_s, bool pressed,
                        const std::array<double, 7>& state) const;
  GateDecision postcheck(const std::array<double, 3>& command,
                         int solver_status, bool plant_ok,
                         const std::array<double, 7>& next_state) const;

 private:
  double joy_timeout_s_;
};

}  // namespace mpcc_cpp_virtual
