#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <string>

namespace mpcc_cpp_virtual {

// Provisional plant response; prediction can optionally use the same parameters.
struct ActuatorParameters {
  double drive_delay_s = 0;
  double steering_delay_s = 0;
  double drive_tau_s = 0;
  double steering_tau_s = 0;

  static ActuatorParameters load(const std::string& path) {
    if (path.empty()) return {};
    std::ifstream input(path);
    auto key = [&input](const char* expected) {
      std::string actual;
      input >> actual;
      if (!input || actual != expected)
        throw std::invalid_argument(std::string("invalid actuator configuration: ") + expected);
    };
    ActuatorParameters p;
    key("MPCC_ACTUATOR_V1");
    key("drive_delay_s"); input >> p.drive_delay_s;
    key("steering_delay_s"); input >> p.steering_delay_s;
    key("drive_tau_s"); input >> p.drive_tau_s;
    key("steering_tau_s"); input >> p.steering_tau_s;
    std::string extra;
    if (!input || input >> extra)
      throw std::invalid_argument("invalid actuator configuration values");
    return p;
  }
};

class ActuatorResponse {
 public:
  ActuatorResponse(const ActuatorParameters& parameters, double dt)
      : parameters_(parameters), dt_(dt) {
    if (!std::isfinite(dt) || dt <= 0)
      throw std::invalid_argument("invalid actuator timestep");
    drive_steps_ = steps(parameters.drive_delay_s);
    steering_steps_ = steps(parameters.steering_delay_s);
    for (double tau : {parameters.drive_tau_s, parameters.steering_tau_s})
      if (!std::isfinite(tau) || tau < 0)
        throw std::invalid_argument("invalid actuator time constant");
  }

  std::array<double, 3> step(const std::array<double, 3>& command) {
    for (double value : command)
      if (!std::isfinite(value))
        throw std::invalid_argument("nonfinite actuator command");
    history_[cursor_] = {command[0], command[1]};
    const double drive = history_[(cursor_ + kCapacity - drive_steps_) % kCapacity][0];
    const double steering = history_[(cursor_ + kCapacity - steering_steps_) % kCapacity][1];
    std::array<double, 3> applied{
        lag(drive, parameters_.drive_tau_s, endpoint_[0]),
        lag(steering, parameters_.steering_tau_s, endpoint_[1]), command[2]};
    cursor_ = (cursor_ + 1) % kCapacity;
    return applied;
  }

  void reset() {
    history_ = {};
    endpoint_ = {};
    cursor_ = 0;
  }

  const std::array<double, 2>& endpoint() const { return endpoint_; }
  const ActuatorParameters& parameters() const { return parameters_; }

  template <std::size_t N>
  std::array<double, N> prediction_state() const {
    std::array<double, N> result{};
    if constexpr (N == 0) return result;  // Legacy instantaneous predictor.
    if (N != 2 + drive_steps_ + steering_steps_)
      throw std::invalid_argument("actuator history dimension differs from predictor");
    result[0] = endpoint_[0]; result[1] = endpoint_[1];
    std::size_t index = 2;
    for (std::size_t channel = 0; channel < 2; ++channel) {
      const auto count = channel == 0 ? drive_steps_ : steering_steps_;
      for (std::size_t i = 0; i < count; ++i)
        result[index++] = history_[(cursor_ + kCapacity - count + i) % kCapacity][channel];
    }
    return result;
  }

 private:
  // Fixed memory: no allocations in the 100Hz control tick.
  static constexpr std::size_t kCapacity = 101;
  std::size_t steps(double delay) const {
    const double count = delay / dt_;
    if (!std::isfinite(delay) || delay < 0 || count > kCapacity - 1 ||
        std::abs(count - std::round(count)) > 1e-9)
      throw std::invalid_argument("actuator dead time must be 0..100 whole feedback ticks");
    return static_cast<std::size_t>(std::round(count));
  }

  double lag(double target, double tau, double& endpoint) const {
    if (tau == 0) { endpoint = target; return target; }
    const double ratio = dt_ / tau;
    const double one_minus_decay = -std::expm1(-ratio);
    // Series avoids cancellation and tau/dt overflow for very slow responses.
    const double mean_fraction = ratio < 1e-4
        ? ratio * (.5 + ratio * (-1. / 6 + ratio / 24))
        : 1 - one_minus_decay / ratio;
    const double average = endpoint + (target - endpoint) *
        mean_fraction;
    endpoint += (target - endpoint) * one_minus_decay;
    return average;
  }

  ActuatorParameters parameters_;
  double dt_;
  std::size_t drive_steps_ = 0;
  std::size_t steering_steps_ = 0;
  std::size_t cursor_ = 0;
  std::array<std::array<double, 2>, kCapacity> history_{};
  std::array<double, 2> endpoint_{};
};

}  // namespace mpcc_cpp_virtual
