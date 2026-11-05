#pragma once

#include "mpcc_cpp_virtual/track.hpp"
#include "mpcc_cpp_virtual/actuator_response.hpp"

#include <array>
#include <memory>
#include <string>

namespace mpcc_cpp_virtual {

#ifndef MPCC_STATE_DIM
#define MPCC_STATE_DIM 10
#endif
inline constexpr int kStateDimension = MPCC_STATE_DIM;
using AugmentedState = std::array<double, kStateDimension>;

struct StageReference {
  std::array<double, 2> center{};
  std::array<double, 2> tangent{};
  double lower_h = 0;
  double upper_h = 0;
};

struct StepResult {
  std::array<double, 3> command{};
  int status = -1;
  double solver_time_s = 0;
  double core_time_s = 0;
  double nlp_time_s = 0;
  double qp_time_s = 0;
  double linearization_time_s = 0;
};

struct RuntimeParameters {
  int horizon = 0;
  double prediction_dt = 0;
  double preview_s = 0;
  AugmentedState state_scale{};
  std::array<double, 3> input_scale{};
  std::array<double, 3> increment_limit{};
  double vehicle_width = 0;
  double safety_scaling = 0;
  double initial_speed = 0;
  double speed_max = 0;
  ActuatorParameters prediction_actuator{};
  static RuntimeParameters load(const std::string& path);
};

class Core {
 public:
  Core(const Track& track, const RuntimeParameters& parameters);
  ~Core();
  Core(const Core&) = delete;
  Core& operator=(const Core&) = delete;
  int horizon() const;
  int solve_count() const;
  AugmentedState prediction_state(int index) const;
  std::array<double, 3> prediction_increment(int index) const;
  AugmentedState last_warm_state(int index) const;
  std::array<double, 3> last_warm_increment(int index) const;
  StepResult step(const std::array<double, 7>& state,
                  const std::array<double, 3>& previous_input,
                  double elapsed_s, const ActuatorResponse* actuator = nullptr);
  const StageReference& stage(int index) const;
  void reset_prediction();
  // Explicit diagnostic I/O, never called by the realtime loop by default.
  bool diagnostics_available() const;
  void write_diagnostics(const std::string& prefix) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mpcc_cpp_virtual
