#include "mpcc_cpp_virtual/core.hpp"
#include "mpcc_cpp_virtual/solution_validation.hpp"

#include "acados_solver_slash_mpcc.h"
#include "plant_step_10ms.h"
#include "plant_step_20ms.h"
#if MPCC_STATE_DIM > 10
#include "prediction_step_10ms.h"
#include "prediction_step_20ms.h"
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

extern "C" int slash_mpcc_c_batch_step_timed(
    slash_mpcc_solver_capsule*, int, const double*, const double*,
    const double*, const double*, const double*, const double*, const double*,
    double*, double*, double*, int*);
extern "C" void ocp_nlp_dump_last_qp_to_json(
    ocp_nlp_config*, ocp_nlp_dims*, ocp_nlp_solver*, const char*);

namespace mpcc_cpp_virtual {
namespace {
constexpr int kN = SLASH_MPCC_N;
constexpr int kX = SLASH_MPCC_NX;
constexpr int kU = SLASH_MPCC_NU;
static_assert(kX == kStateDimension, "generated solver state dimension differs");
constexpr double kPi = 3.141592653589793238462643383279502884;

double seconds(std::chrono::steady_clock::time_point begin,
               std::chrono::steady_clock::time_point end) {
  return std::chrono::duration<double>(end - begin).count();
}
}

RuntimeParameters RuntimeParameters::load(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open runtime parameters");
  auto require_key = [&input](const char* expected) {
    std::string actual;
    input >> actual;
    if (!input || actual != expected)
      throw std::runtime_error(std::string("invalid runtime parameter: ") + expected);
  };
  RuntimeParameters result;
  require_key(kX > 10 ? "MPCC_RUNTIME_V2" : "MPCC_RUNTIME_V1");
  require_key("horizon"); input >> result.horizon;
  require_key("prediction_dt"); input >> result.prediction_dt;
  require_key("preview_s"); input >> result.preview_s;
  require_key("state_scale");
  for (double& value : result.state_scale) input >> value;
  require_key("input_scale");
  for (double& value : result.input_scale) input >> value;
  require_key("increment_limit");
  for (double& value : result.increment_limit) input >> value;
  require_key("vehicle_width"); input >> result.vehicle_width;
  require_key("safety_scaling"); input >> result.safety_scaling;
  require_key("initial_speed"); input >> result.initial_speed;
  require_key("speed_max"); input >> result.speed_max;
  if constexpr (kX > 10) {
    std::array<int, 2> counts{};
    require_key("actuator_delay_steps"); input >> counts[0] >> counts[1];
    require_key("actuator_tau_s");
    input >> result.prediction_actuator.drive_tau_s >> result.prediction_actuator.steering_tau_s;
    if (counts[0] < 0 || counts[1] < 0 || counts[0] > 100 || counts[1] > 100 ||
        kX != 12 + counts[0] + counts[1])
      throw std::runtime_error("invalid predictor actuator dimensions");
    result.prediction_actuator.drive_delay_s = .01 * counts[0];
    result.prediction_actuator.steering_delay_s = .01 * counts[1];
    const ActuatorResponse validated(result.prediction_actuator, .01);
  }
  if (!input || result.horizon != kN || result.prediction_dt < .01 ||
      !std::isfinite(result.prediction_dt) ||
      !std::isfinite(result.preview_s) || result.preview_s < 1.2 - 1e-9 ||
      std::abs(result.horizon * result.prediction_dt - result.preview_s) > 1e-9 ||
      !std::isfinite(result.vehicle_width) || result.vehicle_width < 0 ||
      !std::isfinite(result.safety_scaling) || result.safety_scaling <= 0 ||
      !std::isfinite(result.initial_speed) || result.initial_speed <= 0 ||
      !std::isfinite(result.speed_max) || result.speed_max <= 0)
    throw std::runtime_error("invalid runtime parameters");
  for (double value : result.state_scale)
    if (!std::isfinite(value) || value <= 0)
      throw std::runtime_error("invalid state scale");
  for (double value : result.input_scale)
    if (!std::isfinite(value) || value <= 0)
      throw std::runtime_error("invalid input scale");
  for (double value : result.increment_limit)
    if (!std::isfinite(value) || value <= 0)
      throw std::runtime_error("invalid input increment limit");
  std::string extra;
  if (input >> extra) throw std::runtime_error("extra runtime parameters");
  return result;
}

namespace {
void plant20(const double* state, const double* command, double* next) {
  const double* arguments[2] = {state, command};
  double* results[1] = {next};
#if MPCC_STATE_DIM > 10
  casadi_real work[slash_prediction_step_20ms_SZ_W]{};
  if (slash_prediction_step_20ms(arguments, results, nullptr, work, 0))
#else
  casadi_real work[slash_plant_step_20ms_SZ_W]{};
  if (slash_plant_step_20ms(arguments, results, nullptr, work, 0))
#endif
    throw std::runtime_error("prediction plant step failed");
}

void plant10(const double* state, const double* command, double* next) {
  const double* arguments[2] = {state, command};
  double* results[1] = {next};
#if MPCC_STATE_DIM > 10
  casadi_real work[slash_prediction_step_10ms_SZ_W]{};
  if (slash_prediction_step_10ms(arguments, results, nullptr, work, 0))
#else
  casadi_real work[slash_plant_step_10ms_SZ_W]{};
  if (slash_plant_step_10ms(arguments, results, nullptr, work, 0))
#endif
    throw std::runtime_error("feedback plant step failed");
}
}

struct Core::Impl {
  Impl(const Track& route, const RuntimeParameters& runtime)
      : track(route), params(runtime),
        clearance(runtime.vehicle_width * runtime.safety_scaling / 2),
        scale(runtime.state_scale) {
    if (!std::isfinite(clearance) || clearance < 0)
      throw std::invalid_argument("invalid vehicle clearance");
    if (std::abs(scale[6] - route.length()) > 1e-8)
      throw std::invalid_argument("route length differs from generated solver");
    capsule = slash_mpcc_acados_create_capsule();
    if (!capsule) throw std::runtime_error("acados capsule allocation failed");
    if (slash_mpcc_acados_create(capsule)) {
      slash_mpcc_acados_free_capsule(capsule);
      capsule = nullptr;
      throw std::runtime_error("acados solver initialization failed");
    }
  }

  ~Impl() {
    if (capsule) {
      slash_mpcc_acados_free(capsule);
      slash_mpcc_acados_free_capsule(capsule);
    }
  }

  void initialize(const std::array<double, 7>& state,
                  const std::array<double, 3>& previous,
                  const std::array<double, kX-10>& memory) {
    double previous_yaw = 0;
    for (int i = 0; i <= kN; ++i) {
      const double progress = state[6] + i * params.prediction_dt * state[3];
      const auto reference = track.reference(progress);
      double yaw = std::atan2(reference.tangent[1], reference.tangent[0]);
      if (i == 0) yaw += 2 * kPi * std::round((state[2] - yaw) / (2 * kPi));
      else yaw = previous_yaw + std::remainder(yaw - previous_yaw, 2 * kPi);
      previous_yaw = yaw;
      double* x = prediction.data() + i * kX;
      x[0] = reference.center[0];
      x[1] = reference.center[1];
      x[2] = yaw;
      x[3] = state[3];
      x[4] = x[5] = 0;
      x[6] = progress;
      for (int j = 0; j < 3; ++j) x[7 + j] = previous[j];
      std::copy(memory.begin(), memory.end(), x + 10);
    }
    increments.fill(0);
    initialized = true;
  }

  void shift_full_stage() {
    AugmentedState terminal{};
    std::copy_n(prediction.data() + kN * kX, kX, terminal.data());
    std::memmove(prediction.data(), prediction.data() + kX,
                 kN * kX * sizeof(double));
    double* end = prediction.data() + kN * kX;
    plant20(terminal.data(), terminal.data() + 7, end);
    for (int j = 0; j < 3; ++j) end[7 + j] = terminal[7 + j];
    std::memmove(increments.data(), increments.data() + kU,
                 (kN - 1) * kU * sizeof(double));
    std::fill_n(increments.data() + (kN - 1) * kU, kU, 0.);
  }

  void shift_fractional(double elapsed_s) {
    const double alpha = elapsed_s / params.prediction_dt;
    old_prediction = prediction;
    old_increments = increments;
    for (int i = 0; i < kN; ++i) {
      for (int j = 0; j < kX; ++j)
        prediction[i * kX + j] = (1 - alpha) * old_prediction[i * kX + j] +
                                   alpha * old_prediction[(i + 1) * kX + j];
    }
    double* end = prediction.data() + kN * kX;
    const double* terminal = old_prediction.data() + kN * kX;
    plant10(terminal, terminal + 7, end);
    for (int j = 0; j < 3; ++j) end[7 + j] = terminal[7 + j];
    for (int i = 0; i < kN; ++i) {
      for (int j = 0; j < kU; ++j) {
        const double next = i + 1 < kN ? old_increments[(i + 1) * kU + j] : 0;
        increments[i * kU + j] = (1 - alpha) * old_increments[i * kU + j] +
                                  alpha * next;
      }
    }
  }

  const Track& track;
  const RuntimeParameters params;
  const double clearance;
  const AugmentedState scale;
  slash_mpcc_solver_capsule* capsule = nullptr;
  bool initialized = false;
  int solves = 0;
  int last_status = -1;
  bool last_qp_valid = false;
  double last_elapsed = 0;
  std::array<double, 3> first_lower{}, first_upper{};
  std::array<double, (kN + 1) * kX> prediction{};
  std::array<double, (kN + 1) * kX> old_prediction{};
  std::array<double, kN * kU> increments{};
  std::array<double, kN * kU> old_increments{};
  std::array<double, kN * kU> warm_increments{};
  std::array<double, (kN + 1) * kX> x_guess{};
  std::array<double, (kN + 1) * kX> x_solution{};
  std::array<double, kN * kU> u_solution{};
  std::array<double, (kN + 1) * 4> parameters{};
  std::array<double, kN + 1> lower_h{};
  std::array<double, kN + 1> upper_h{};
  std::array<StageReference, kN + 1> stages{};
};

Core::Core(const Track& track, const RuntimeParameters& parameters)
    : impl_(new Impl(track, parameters)) {}
Core::~Core() = default;
void Core::reset_prediction() { impl_->initialized = false; }
bool Core::diagnostics_available() const { return impl_->last_qp_valid; }

void Core::write_diagnostics(const std::string& prefix) const {
  const auto& self = *impl_;
  if (!self.last_qp_valid) throw std::logic_error("latest step has no solver QP to diagnose");
  std::ofstream stream(prefix + "_snapshot.json");
  std::ofstream qp_check(prefix + "_qp.json");
  if (!stream || !qp_check) throw std::runtime_error("cannot write diagnostics");
  qp_check.close();
  stream << std::setprecision(17) << "{\n\"nx\": " << kX
         << ",\n\"horizon\": " << kN << ",\n\"status\": " << self.last_status
         << ",\n\"solve_count\": " << self.solves
         << ",\n\"elapsed_s\": " << self.last_elapsed;
  const auto write = [&stream](const char* key, const auto& values) {
    stream << ",\n\"" << key << "\": [";
    bool comma = false;
    for (const auto value : values) {
      if (comma) stream << ',';
      if (std::isfinite(value)) stream << value;
      else stream << "null";
      comma = true;
    }
    stream << ']';
  };
  write("scale", self.scale);
  write("x_guess", self.x_guess);
  write("u_guess", self.warm_increments);
  write("parameters", self.parameters);
  write("lower_h", self.lower_h); write("upper_h", self.upper_h);
  write("first_lower_u", self.first_lower); write("first_upper_u", self.first_upper);
  write("x_solution", self.x_solution); write("u_solution", self.u_solution);
  int capacity = 0, columns = 0, iterations = 0;
  ocp_nlp_get(self.capsule->nlp_solver, "stat_m", &capacity);
  ocp_nlp_get(self.capsule->nlp_solver, "stat_n", &columns);
  ocp_nlp_get(self.capsule->nlp_solver, "nlp_iter", &iterations);
  const int rows = std::min(capacity, iterations + 1);
  std::vector<double> stats(rows * (columns+1));
  ocp_nlp_get(self.capsule->nlp_solver, "statistics", stats.data());
  stream << ",\n\"stat_m\": " << capacity << ",\n\"stat_n\": " << columns
         << ",\n\"nlp_iter\": " << iterations << ",\n\"statistics_rows\": " << rows
         << ",\n\"statistics_layout\": \"column-major, statistics_rows by (stat_n+1)\"";
  write("statistics", stats);
  stream << "\n}\n";
  stream.flush();
  if (!stream) throw std::runtime_error("diagnostic snapshot write failed");
  ocp_nlp_dump_last_qp_to_json(self.capsule->nlp_config, self.capsule->nlp_dims,
                             self.capsule->nlp_solver, (prefix + "_qp.json").c_str());
}

int Core::horizon() const { return kN; }
int Core::solve_count() const { return impl_->solves; }

AugmentedState Core::prediction_state(int index) const {
  if (index < 0 || index > kN) throw std::out_of_range("prediction stage");
  AugmentedState value{};
  std::copy_n(impl_->prediction.data() + index * kX, kX, value.data());
  return value;
}

std::array<double, 3> Core::prediction_increment(int index) const {
  if (index < 0 || index >= kN) throw std::out_of_range("increment stage");
  std::array<double, 3> value{};
  std::copy_n(impl_->increments.data() + index * kU, kU, value.data());
  return value;
}

AugmentedState Core::last_warm_state(int index) const {
  if (index < 0 || index > kN) throw std::out_of_range("warm stage");
  AugmentedState value{};
  for (int j = 0; j < kX; ++j)
    value[j] = impl_->x_guess[index * kX + j] * impl_->scale[j];
  return value;
}

std::array<double, 3> Core::last_warm_increment(int index) const {
  if (index < 0 || index >= kN) throw std::out_of_range("warm increment");
  std::array<double, 3> value{};
  std::copy_n(impl_->warm_increments.data() + index * kU, kU, value.data());
  return value;
}

const StageReference& Core::stage(int index) const {
  if (index < 0 || index > kN) throw std::out_of_range("stage index");
  return impl_->stages[index];
}

StepResult Core::step(const std::array<double, 7>& state,
                      const std::array<double, 3>& previous_input,
                      double elapsed_s, const ActuatorResponse* actuator) {
  const auto start = std::chrono::steady_clock::now();
  StepResult result;
  auto& self = *impl_;
  self.last_qp_valid = false;
  if (!std::isfinite(elapsed_s) || elapsed_s <= 0 ||
      !std::all_of(state.begin(), state.end(), [](double v) { return std::isfinite(v); }) ||
      !std::all_of(previous_input.begin(), previous_input.end(),
                   [](double v) { return std::isfinite(v); })) {
    result.status = -2;
    return result;
  }
  std::array<double, kX-10> memory{};
  if constexpr (kX > 10) {
    if (!actuator) throw std::invalid_argument("predictor requires current actuator state");
    const auto& actual = actuator->parameters();
    const auto& expected = self.params.prediction_actuator;
    if (std::abs(actual.drive_delay_s - expected.drive_delay_s) > 1e-12 ||
        std::abs(actual.steering_delay_s - expected.steering_delay_s) > 1e-12 ||
        std::abs(actual.drive_tau_s - expected.drive_tau_s) > 1e-12 ||
        std::abs(actual.steering_tau_s - expected.steering_tau_s) > 1e-12)
      throw std::invalid_argument("plant actuator parameters differ from generated predictor");
    memory = actuator->prediction_state<kX-10>();
  }
  if (!self.initialized) self.initialize(state, previous_input, memory);
  else if (std::abs(elapsed_s - self.params.prediction_dt) < 1e-12)
    self.shift_full_stage();
  else if (std::abs(elapsed_s - .01) < 1e-12) self.shift_fractional(elapsed_s);
  else throw std::invalid_argument("unsupported elapsed interval");
  std::copy(state.begin(), state.end(), self.prediction.begin());
  std::copy(previous_input.begin(), previous_input.end(), self.prediction.begin() + 7);
  std::copy(memory.begin(), memory.end(), self.prediction.begin() + 10);
  for (int i = 0; i <= kN; ++i) {
    const double* x = self.prediction.data() + i * kX;
    auto& stage = self.stages[i];
    const auto reference = self.track.reference(x[6]);
    stage.center = reference.center;
    stage.tangent = reference.tangent;
    stage.lower_h = -reference.width_right + self.clearance;
    stage.upper_h = reference.width_left - self.clearance;
    if (i > 0 && !(stage.lower_h < 0 && stage.upper_h > 0)) {
      result.status = -3;
      return result;
    }
    double* p = self.parameters.data() + i * 4;
    p[0] = stage.center[0];
    p[1] = stage.center[1];
    p[2] = -stage.tangent[1];
    p[3] = stage.tangent[0];
    self.lower_h[i] = stage.lower_h;
    self.upper_h[i] = stage.upper_h;
    for (int j = 0; j < kX; ++j) self.x_guess[i * kX + j] = x[j] / self.scale[j];
  }
  double timings[3]{};
  self.warm_increments = self.increments;
  std::array<double, 3> first_lower{}, first_upper{};
  const double fraction = std::min(1., elapsed_s / self.params.prediction_dt);
  for (int j = 0; j < 3; ++j) {
    first_upper[j] = fraction * self.params.increment_limit[j];
    first_lower[j] = -first_upper[j];
  }
  self.first_lower = first_lower;
  self.first_upper = first_upper;
  self.last_elapsed = elapsed_s;
  int status = -1;
  ++self.solves;
  const int bridge = slash_mpcc_c_batch_step_timed(
      self.capsule, kN, self.x_guess.data(), self.increments.data(),
      self.parameters.data(), self.lower_h.data(), self.upper_h.data(),
      first_lower.data(), first_upper.data(),
      self.x_solution.data(), self.u_solution.data(), timings, &status);
  result.solver_time_s = timings[1];
  result.status = bridge ? -1000 - bridge : status;
  self.last_qp_valid = !bridge;
  if (!bridge) {
    ocp_nlp_get(self.capsule->nlp_solver, "time_tot", &result.nlp_time_s);
    ocp_nlp_get(self.capsule->nlp_solver, "time_qp", &result.qp_time_s);
    ocp_nlp_get(self.capsule->nlp_solver, "time_lin", &result.linearization_time_s);
  }
  if (!bridge && status == 0 &&
      !valid_solution(self.x_solution.data(), self.x_solution.size(),
                      self.u_solution.data(), self.u_solution.size(),
                      self.scale.data(), kX)) {
    result.status = -4;
  }
  if (result.status == 0) {
    for (int i = 0; i <= kN; ++i)
      for (int j = 0; j < kX; ++j)
        self.prediction[i * kX + j] = self.x_solution[i * kX + j] * self.scale[j];
    self.increments = self.u_solution;
    for (int j = 0; j < 3; ++j)
      result.command[j] = previous_input[j] +
                          self.params.input_scale[j] * self.increments[j];
  }
  if (!std::all_of(result.command.begin(), result.command.end(),
                   [](double v) { return std::isfinite(v); })) {
    result.status = -4;
    result.command = {};
  }
  result.core_time_s = seconds(start, std::chrono::steady_clock::now());
  self.last_status = result.status;
  return result;
}

}  // namespace mpcc_cpp_virtual
