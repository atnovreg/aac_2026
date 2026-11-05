#include "mpcc_cpp_virtual/core.hpp"
#include "mpcc_cpp_virtual/actuator_response.hpp"
#include "mpcc_cpp_virtual/identity.hpp"
#include "mpcc_cpp_virtual/lap.hpp"
#include "mpcc_cpp_virtual/safety.hpp"
#include "plant_step_10ms.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace mpcc_cpp_virtual {
namespace {
using Clock = std::chrono::steady_clock;
constexpr double kFeedbackS = .01;

double seconds(Clock::time_point begin, Clock::time_point end) {
  return std::chrono::duration<double>(end - begin).count();
}

struct Options {
  std::string track;
  std::string runtime;
  std::string log;
  std::string actuator_config;
  double duration_s = 5;
  double joy_timeout_s = .5;
  int deadman_index = 1;  // Bluetooth DualSense X observed in today's /joy log.
  int inject_solver_after = -1;
  int inject_plant_after = -1;
  int inject_nonfinite_after = -1;
  int synthetic_release_after = -1;
  int synthetic_stop_after = -1;
  bool synthetic_deadman = false;
};

Options parse(int argc, char** argv) {
  Options result;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--synthetic-deadman") result.synthetic_deadman = true;
    else {
      if (++i >= argc) throw std::invalid_argument("missing option value: " + key);
      const std::string value = argv[i];
      if (key == "--track") result.track = value;
      else if (key == "--runtime") result.runtime = value;
      else if (key == "--log") result.log = value;
      else if (key == "--actuator-config") result.actuator_config = value;
      else if (key == "--duration") result.duration_s = std::stod(value);
      else if (key == "--joy-timeout") result.joy_timeout_s = std::stod(value);
      else if (key == "--deadman-index") result.deadman_index = std::stoi(value);
      else if (key == "--inject-solver-after") result.inject_solver_after = std::stoi(value);
      else if (key == "--inject-plant-after") result.inject_plant_after = std::stoi(value);
      else if (key == "--inject-nonfinite-after") result.inject_nonfinite_after = std::stoi(value);
      else if (key == "--synthetic-release-after") result.synthetic_release_after = std::stoi(value);
      else if (key == "--synthetic-stop-after") result.synthetic_stop_after = std::stoi(value);
      else throw std::invalid_argument("unknown option: " + key);
    }
  }
  if (result.track.empty() || result.runtime.empty() || result.log.empty() ||
      !std::isfinite(result.duration_s) || result.duration_s <= 0 ||
      result.deadman_index < 0)
    throw std::invalid_argument("required: --track, --runtime, --log, --duration > 0");
  return result;
}

struct TickLog {
  double scheduled_s = 0;
  double start_s = 0;
  double publish_s = 0;
  double callback_ms = 0;
  double solver_ms = 0;
  double core_ms = 0;
  double plant_ms = 0;
  double nlp_ms = 0;
  double qp_ms = 0;
  double linearization_ms = 0;
  std::array<double, 7> state{};
  std::array<double, 3> command{};
  std::array<double, 3> applied{};
  std::array<double, 2> actuator_endpoint{};
  SafetyStatus status = SafetyStatus::JoyTimeout;
  int skipped_releases = 0;
  int completed_laps = 0;
  double cumulative_progress = 0;
};
}

class VirtualNode : public rclcpp::Node {
 public:
    explicit VirtualNode(const Options& options)
      : Node("mpcc_cpp_virtual"), options_(options),
        track_(Track::load(options.track)),
        lap_counter_(track_.length()),
        parameters_(RuntimeParameters::load(options.runtime)),
        core_(track_, parameters_), gate_(options.joy_timeout_s),
        actuator_(ActuatorParameters::load(options.actuator_config), kFeedbackS) {
    verify_runtime_artifacts(options.track, options.runtime);
    const auto route = track_.reference(0.);
    state_ = {route.center[0], route.center[1],
              std::atan2(route.tangent[1], route.tangent[0]),
              parameters_.initial_speed, 0., 0., 0.};
    command_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/virtual_vehicle/mpcc_command", 10);
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/virtual_vehicle/pose", 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        "/virtual_vehicle/status", 10);
    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Joy::SharedPtr message) {
          last_joy_s_ = seconds(start_, Clock::now());
          deadman_pressed_ = message->buttons.size() >
                  static_cast<std::size_t>(options_.deadman_index) &&
              message->buttons[options_.deadman_index] == 1;
        });
    log_.reserve(static_cast<std::size_t>(options.duration_s * 110) + 100);
    start_ = Clock::now();
    expected_ = start_ + std::chrono::milliseconds(10);
    timer_ = create_wall_timer(std::chrono::milliseconds(10),
                               [this]() { tick(); });
    std::cout << "READY: C++ virtual MPCC; no actuator publishers" << std::endl;
  }

  void finish() {
    timer_->cancel();
    publish(SafetyStatus::DeadmanReleased, {});
    std::ofstream stream(options_.log);
    if (!stream) throw std::runtime_error("cannot write ROS timing log");
    stream << std::setprecision(17);
    stream << "scheduled_s,start_s,publish_s,callback_ms,solver_ms,core_ms,"
              "plant_ms,skipped_releases,status,completed_laps,"
              "cumulative_progress,x,y,yaw,vx,vy,yaw_rate,progress,"
              "current,steering,vtheta,nlp_ms,qp_ms,linearization_ms,"
              "applied_drive,applied_steering,applied_vtheta,"
              "drive_endpoint,steering_endpoint\n";
    for (const auto& row : log_) {
      stream << row.scheduled_s << ',' << row.start_s << ',' << row.publish_s
             << ',' << row.callback_ms << ',' << row.solver_ms << ','
             << row.core_ms << ',' << row.plant_ms << ','
             << row.skipped_releases << ',' << status_name(row.status)
             << ',' << row.completed_laps << ',' << row.cumulative_progress;
      for (double value : row.state) stream << ',' << value;
      for (double value : row.command) stream << ',' << value;
      stream << ',' << row.nlp_ms << ',' << row.qp_ms << ','
             << row.linearization_ms;
      for (double value : row.applied) stream << ',' << value;
      for (double value : row.actuator_endpoint) stream << ',' << value;
      stream << '\n';
    }
    std::cout << "ticks=" << log_.size() << " active=" << active_cycles_
              << " skipped_releases=" << skipped_releases_ << '\n';
  }

 private:
  void publish(SafetyStatus status, const std::array<double, 3>& command) {
    std_msgs::msg::Float64MultiArray command_message;
    command_message.data.assign(command.begin(), command.end());
    command_publisher_->publish(command_message);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "virtual_map";
    pose.pose.position.x = state_[0];
    pose.pose.position.y = state_[1];
    pose.pose.orientation.z = std::sin(state_[2] / 2);
    pose.pose.orientation.w = std::cos(state_[2] / 2);
    pose_publisher_->publish(pose);
    std_msgs::msg::String status_message;
    status_message.data = status_name(status);
    status_publisher_->publish(status_message);
  }

  void tick() {
    const auto began = Clock::now();
    const double now_s = seconds(start_, began);
    TickLog row;
    GateDecision decision;
    try {
    if (began >= expected_ + std::chrono::milliseconds(10)) {
      row.skipped_releases = static_cast<int>(
          seconds(expected_, began) / kFeedbackS);
      skipped_releases_ += row.skipped_releases;
      expected_ += std::chrono::milliseconds(10 * row.skipped_releases);
    }
    row.scheduled_s = seconds(start_, expected_);
    row.start_s = now_s;
    expected_ += std::chrono::milliseconds(10);
    if (options_.synthetic_deadman) {
      if (options_.synthetic_release_after >= 0 &&
          active_cycles_ >= options_.synthetic_release_after) {
        last_joy_s_ = now_s;
        deadman_pressed_ = false;
      } else if (options_.synthetic_stop_after < 0 ||
                 active_cycles_ < options_.synthetic_stop_after) {
        last_joy_s_ = now_s;
        deadman_pressed_ = true;
      }
    }
    std::array<double, 7> candidate = state_;
    auto candidate_actuator = actuator_;
    std::array<double, 3> candidate_applied{};
    if (latched_failure_) {
      decision = {latched_status_, {}, false};
    } else {
      decision = gate_.precheck(now_s, last_joy_s_, deadman_pressed_, state_);
      if (decision.status == SafetyStatus::Active) {
        if (options_.inject_nonfinite_after >= 0 &&
            active_cycles_ >= options_.inject_nonfinite_after) {
          candidate[0] = std::numeric_limits<double>::quiet_NaN();
          decision = gate_.postcheck({}, 0, true, candidate);
        } else if (options_.inject_solver_after >= 0 &&
                   active_cycles_ >= options_.inject_solver_after) {
          decision = gate_.postcheck({}, 1, true, candidate);
        } else {
          try {
            const auto solved = core_.step(state_, previous_, kFeedbackS, &actuator_);
            row.solver_ms = solved.solver_time_s * 1000;
            row.core_ms = solved.core_time_s * 1000;
            row.nlp_ms = solved.nlp_time_s * 1000;
            row.qp_ms = solved.qp_time_s * 1000;
            row.linearization_ms = solved.linearization_time_s * 1000;
            bool plant_ok = solved.status == 0;
            if (plant_ok) {
              const auto plant_start = Clock::now();
              candidate_applied = candidate_actuator.step(solved.command);
              const double* arguments[] = {state_.data(), candidate_applied.data()};
              double* results[] = {candidate.data()};
              casadi_real work[slash_plant_step_10ms_SZ_W]{};
              plant_ok = slash_plant_step_10ms(arguments, results, nullptr,
                                               work, 0) == 0;
              if (options_.inject_plant_after >= 0 &&
                  active_cycles_ >= options_.inject_plant_after) plant_ok = false;
              if (plant_ok) {
                candidate[6] = track_.project({candidate[0], candidate[1]},
                                              state_[6]);
              }
              row.plant_ms = seconds(plant_start, Clock::now()) * 1000;
            }
            decision = gate_.postcheck(solved.command, solved.status,
                                       plant_ok, candidate);
          } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "virtual MPCC failure: %s", error.what());
            decision = {SafetyStatus::SolverError, {}, false};
          }
        }
        if (decision.advance) {
          const bool crossed = lap_counter_.wrap(candidate);
          state_ = candidate;
          actuator_ = candidate_actuator;
          row.applied = candidate_applied;
          if (crossed) core_.reset_prediction();
          previous_ = decision.command;
          ++active_cycles_;
        } else {
          latched_failure_ = true;
          latched_status_ = decision.status;
          previous_ = {};
        }
      } else {
        previous_ = {};
        core_.reset_prediction();
      }
    }
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "virtual tick failure: %s", error.what());
      decision = {SafetyStatus::SolverError, {}, false};
      latched_failure_ = true;
      latched_status_ = decision.status;
      previous_ = {};
    }
    if (!decision.advance) actuator_.reset();
    publish(decision.status, decision.command);
    row.publish_s = seconds(start_, Clock::now());
    row.status = decision.status;
    row.command = decision.command;
    row.actuator_endpoint = actuator_.endpoint();
    row.state = state_;
    row.completed_laps = lap_counter_.count();
    row.cumulative_progress = lap_counter_.cumulative_progress(state_[6]);
    log_.push_back(row);
    log_.back().callback_ms = seconds(began, Clock::now()) * 1000;
  }

  const Options options_;
  Track track_;
  LapCounter lap_counter_;
  RuntimeParameters parameters_;
  Core core_;
  SafetyGate gate_;
  ActuatorResponse actuator_;
  std::array<double, 7> state_{};
  std::array<double, 3> previous_{};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  Clock::time_point start_{};
  Clock::time_point expected_{};
  double last_joy_s_ = std::numeric_limits<double>::quiet_NaN();
  bool deadman_pressed_ = false;
  bool latched_failure_ = false;
  SafetyStatus latched_status_ = SafetyStatus::SolverError;
  int active_cycles_ = 0;
  int skipped_releases_ = 0;
  std::vector<TickLog> log_;
};

}  // namespace mpcc_cpp_virtual

int main(int argc, char** argv) {
  try {
    const auto options = mpcc_cpp_virtual::parse(argc, argv);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mpcc_cpp_virtual::VirtualNode>(options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto finish = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(options.duration_s);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < finish)
      executor.spin_once(std::chrono::milliseconds(10));
    node->finish();
    executor.remove_node(node);
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "virtual node failed: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
