#include "mppi_planner_node/mppi_planner.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;

std::string escapeJsonString(const std::string & value)
{
  std::ostringstream stream;
  for (const auto character : value) {
    if (character == '\\') {
      stream << "\\\\";
    } else if (character == '"') {
      stream << "\\\"";
    } else {
      stream << character;
    }
  }
  return stream.str();
}
}  // namespace

MppiPlanner::MppiPlanner(const std::string & node_name)
: Node(node_name)
{
  getParams();
  resetControlSequence();

  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&MppiPlanner::onParamsChanged, this, std::placeholders::_1));

  const unsigned int seed = random_seed_ == 0 ?
    std::random_device{}() : static_cast<unsigned int>(random_seed_);
  rng_.seed(seed);

  sub_local_path_ = create_subscription<nav_msgs::msg::Path>(
    local_path_topic_, rclcpp::QoS(10),
    std::bind(&MppiPlanner::localPathCallback, this, std::placeholders::_1));

  pub_command_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    command_topic_, rclcpp::QoS(10));
  pub_trajectory_ = create_publisher<nav_msgs::msg::Path>(trajectory_topic_, rclcpp::QoS(10));
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic_, rclcpp::QoS(10));
  pub_debug_status_ = create_publisher<std_msgs::msg::String>(debug_topic_, rclcpp::QoS(10));

  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / std::max(update_rate_, 0.1)));
  timer_ = create_wall_timer(period_ms, std::bind(&MppiPlanner::timerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "MPPI planner ready: path=%s, cmd=%s, horizon=%d x %.2fs, batch=%d, nav2 deps=none",
    local_path_topic_.c_str(), command_topic_.c_str(), horizon_steps_, time_step_, batch_size_);
}

void MppiPlanner::getParams()
{
  declare_parameter<std::string>("local_path_topic", "/local_path/body");
  declare_parameter<std::string>("command_topic", "/ackermann_auto");
  declare_parameter<std::string>("command_frame", "base_footprint");
  declare_parameter<std::string>("trajectory_topic", "/mppi/optimal_trajectory");
  declare_parameter<std::string>("marker_topic", "/mppi/trajectory_markers");
  declare_parameter<std::string>("debug_topic", "/mppi/debug_status");
  declare_parameter<std::string>("tuning_profile", "config");

  declare_parameter<double>("update_rate", 30.0);
  declare_parameter<double>("path_timeout", 0.5);
  declare_parameter<double>("wheel_base", 0.80);
  declare_parameter<double>("time_step", 0.08);
  declare_parameter<int>("horizon_steps", 32);
  declare_parameter<int>("batch_size", 320);
  declare_parameter<int>("iteration_count", 2);
  declare_parameter<int>("random_seed", 7);

  declare_parameter<double>("target_speed", 0.9);
  declare_parameter<double>("min_speed", 0.0);
  declare_parameter<double>("tracking_min_command_speed", 0.25);
  declare_parameter<double>("max_speed", 1.1);
  declare_parameter<double>("max_steering_angle", 0.524);
  declare_parameter<double>("max_accel", 1.5);
  declare_parameter<double>("max_steering_rate", 4.0);
  declare_parameter<double>("speed_noise_std", 0.30);
  declare_parameter<double>("steering_noise_std", 0.34);
  declare_parameter<double>("temperature", 0.45);

  declare_parameter<double>("path_distance_weight", 9.5);
  declare_parameter<double>("heading_weight", 3.0);
  declare_parameter<double>("heading_lookahead_distance", 1.2);
  declare_parameter<double>("heading_lookahead_weight", 2.5);
  declare_parameter<double>("steering_feedforward_lookahead_distance", 1.2);
  declare_parameter<double>("steering_feedforward_weight", 0.0);
  declare_parameter<double>("steering_feedforward_command_gain", 0.0);
  declare_parameter<double>("curvature_lookahead_distance", 1.8);
  declare_parameter<double>("curvature_speed_weight", 0.8);
  declare_parameter<double>("curvature_speed_gain", 0.8);
  declare_parameter<double>("curvature_min_speed", 0.45);
  declare_parameter<double>("lateral_accel_limit", 0.45);
  declare_parameter<bool>("use_lateral_accel_speed_limit", true);
  declare_parameter<bool>("use_corner_speed_as_speed_target", true);
  declare_parameter<bool>("limit_command_speed_to_corner", true);
  declare_parameter<double>("corner_speed_margin", 0.05);
  declare_parameter<double>("terminal_distance_weight", 16.0);
  declare_parameter<double>("terminal_heading_weight", 5.0);
  declare_parameter<double>("speed_weight", 0.30);
  declare_parameter<double>("steering_weight", 0.04);
  declare_parameter<double>("steering_rate_weight", 0.08);
  declare_parameter<double>("accel_weight", 0.08);
  declare_parameter<double>("progress_weight", 0.7);

  declare_parameter<double>("min_path_segment_length", 0.02);
  declare_parameter<double>("min_path_forward_x", -0.30);
  declare_parameter<int>("min_path_points", 2);
  declare_parameter<bool>("visualize_markers", true);
  declare_parameter<bool>("visualize_rollouts", true);
  declare_parameter<bool>("publish_debug_status", true);
  declare_parameter<int>("max_visualized_rollouts", 48);
  declare_parameter<int>("rollout_visualization_stride", 6);
  declare_parameter<double>("marker_line_width", 0.045);
  declare_parameter<double>("rollout_line_width", 0.018);
  declare_parameter<double>("rollout_alpha", 0.22);
  declare_parameter<double>("marker_height", 0.05);

  local_path_topic_ = get_parameter("local_path_topic").as_string();
  command_topic_ = get_parameter("command_topic").as_string();
  command_frame_ = get_parameter("command_frame").as_string();
  trajectory_topic_ = get_parameter("trajectory_topic").as_string();
  marker_topic_ = get_parameter("marker_topic").as_string();
  debug_topic_ = get_parameter("debug_topic").as_string();
  tuning_profile_ = get_parameter("tuning_profile").as_string();

  update_rate_ = get_parameter("update_rate").as_double();
  path_timeout_ = get_parameter("path_timeout").as_double();
  wheel_base_ = get_parameter("wheel_base").as_double();
  time_step_ = get_parameter("time_step").as_double();
  horizon_steps_ = static_cast<int>(get_parameter("horizon_steps").as_int());
  batch_size_ = static_cast<int>(get_parameter("batch_size").as_int());
  iteration_count_ = static_cast<int>(get_parameter("iteration_count").as_int());
  random_seed_ = static_cast<int>(get_parameter("random_seed").as_int());

  target_speed_ = get_parameter("target_speed").as_double();
  min_speed_ = get_parameter("min_speed").as_double();
  tracking_min_command_speed_ = get_parameter("tracking_min_command_speed").as_double();
  max_speed_ = get_parameter("max_speed").as_double();
  max_steering_angle_ = get_parameter("max_steering_angle").as_double();
  max_accel_ = get_parameter("max_accel").as_double();
  max_steering_rate_ = get_parameter("max_steering_rate").as_double();
  speed_noise_std_ = get_parameter("speed_noise_std").as_double();
  steering_noise_std_ = get_parameter("steering_noise_std").as_double();
  temperature_ = get_parameter("temperature").as_double();

  path_distance_weight_ = get_parameter("path_distance_weight").as_double();
  heading_weight_ = get_parameter("heading_weight").as_double();
  heading_lookahead_distance_ = get_parameter("heading_lookahead_distance").as_double();
  heading_lookahead_weight_ = get_parameter("heading_lookahead_weight").as_double();
  steering_feedforward_lookahead_distance_ =
    get_parameter("steering_feedforward_lookahead_distance").as_double();
  steering_feedforward_weight_ = get_parameter("steering_feedforward_weight").as_double();
  steering_feedforward_command_gain_ =
    get_parameter("steering_feedforward_command_gain").as_double();
  curvature_lookahead_distance_ = get_parameter("curvature_lookahead_distance").as_double();
  curvature_speed_weight_ = get_parameter("curvature_speed_weight").as_double();
  curvature_speed_gain_ = get_parameter("curvature_speed_gain").as_double();
  curvature_min_speed_ = get_parameter("curvature_min_speed").as_double();
  lateral_accel_limit_ = get_parameter("lateral_accel_limit").as_double();
  use_lateral_accel_speed_limit_ =
    get_parameter("use_lateral_accel_speed_limit").as_bool();
  use_corner_speed_as_speed_target_ =
    get_parameter("use_corner_speed_as_speed_target").as_bool();
  limit_command_speed_to_corner_ = get_parameter("limit_command_speed_to_corner").as_bool();
  corner_speed_margin_ = get_parameter("corner_speed_margin").as_double();
  terminal_distance_weight_ = get_parameter("terminal_distance_weight").as_double();
  terminal_heading_weight_ = get_parameter("terminal_heading_weight").as_double();
  speed_weight_ = get_parameter("speed_weight").as_double();
  steering_weight_ = get_parameter("steering_weight").as_double();
  steering_rate_weight_ = get_parameter("steering_rate_weight").as_double();
  accel_weight_ = get_parameter("accel_weight").as_double();
  progress_weight_ = get_parameter("progress_weight").as_double();

  min_path_segment_length_ = get_parameter("min_path_segment_length").as_double();
  min_path_forward_x_ = get_parameter("min_path_forward_x").as_double();
  min_path_points_ = static_cast<int>(get_parameter("min_path_points").as_int());
  visualize_markers_ = get_parameter("visualize_markers").as_bool();
  visualize_rollouts_ = get_parameter("visualize_rollouts").as_bool();
  publish_debug_status_ = get_parameter("publish_debug_status").as_bool();
  max_visualized_rollouts_ = static_cast<int>(get_parameter("max_visualized_rollouts").as_int());
  rollout_visualization_stride_ = static_cast<int>(
    get_parameter("rollout_visualization_stride").as_int());
  marker_line_width_ = get_parameter("marker_line_width").as_double();
  rollout_line_width_ = get_parameter("rollout_line_width").as_double();
  rollout_alpha_ = get_parameter("rollout_alpha").as_double();
  marker_height_ = get_parameter("marker_height").as_double();

  update_rate_ = std::max(0.1, update_rate_);
  path_timeout_ = std::max(0.0, path_timeout_);
  wheel_base_ = std::max(0.05, wheel_base_);
  time_step_ = std::max(0.01, time_step_);
  horizon_steps_ = std::max(2, horizon_steps_);
  batch_size_ = std::max(1, batch_size_);
  iteration_count_ = std::max(1, iteration_count_);
  max_speed_ = std::max(0.0, max_speed_);
  min_speed_ = clampValue(min_speed_, 0.0, max_speed_);
  tracking_min_command_speed_ = clampValue(tracking_min_command_speed_, 0.0, max_speed_);
  target_speed_ = clampValue(target_speed_, min_speed_, max_speed_);
  max_steering_angle_ = std::max(0.01, max_steering_angle_);
  max_accel_ = std::max(0.0, max_accel_);
  max_steering_rate_ = std::max(0.0, max_steering_rate_);
  speed_noise_std_ = std::max(0.0, speed_noise_std_);
  steering_noise_std_ = std::max(0.0, steering_noise_std_);
  temperature_ = std::max(1.0e-6, temperature_);
  heading_lookahead_distance_ = std::max(0.0, heading_lookahead_distance_);
  heading_lookahead_weight_ = std::max(0.0, heading_lookahead_weight_);
  steering_feedforward_lookahead_distance_ =
    std::max(0.0, steering_feedforward_lookahead_distance_);
  steering_feedforward_weight_ = std::max(0.0, steering_feedforward_weight_);
  steering_feedforward_command_gain_ =
    clampValue(steering_feedforward_command_gain_, 0.0, 1.0);
  curvature_lookahead_distance_ = std::max(0.0, curvature_lookahead_distance_);
  curvature_speed_weight_ = std::max(0.0, curvature_speed_weight_);
  curvature_speed_gain_ = std::max(0.0, curvature_speed_gain_);
  curvature_min_speed_ = clampValue(curvature_min_speed_, min_speed_, target_speed_);
  lateral_accel_limit_ = std::max(0.0, lateral_accel_limit_);
  corner_speed_margin_ = std::max(0.0, corner_speed_margin_);
  min_path_segment_length_ = std::max(1.0e-4, min_path_segment_length_);
  min_path_points_ = std::max(2, min_path_points_);
  max_visualized_rollouts_ = std::max(0, max_visualized_rollouts_);
  rollout_visualization_stride_ = std::max(1, rollout_visualization_stride_);
  marker_line_width_ = std::max(0.005, marker_line_width_);
  rollout_line_width_ = std::max(0.003, rollout_line_width_);
  rollout_alpha_ = clampValue(rollout_alpha_, 0.01, 1.0);
}

rcl_interfaces::msg::SetParametersResult MppiPlanner::onParamsChanged(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  auto reject = [&result](const std::string & reason) {
      result.successful = false;
      result.reason = reason;
      return result;
    };

  auto read_double = [&reject](const rclcpp::Parameter & parameter, double & value) {
      const auto type = parameter.get_type();
      if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        value = parameter.as_double();
        return true;
      }
      if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        value = static_cast<double>(parameter.as_int());
        return true;
      }
      reject(parameter.get_name() + " must be a number");
      return false;
    };

  auto read_int = [&reject](const rclcpp::Parameter & parameter, int & value) {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        reject(parameter.get_name() + " must be an integer");
        return false;
      }
      value = static_cast<int>(parameter.as_int());
      return true;
    };

  auto read_bool = [&reject](const rclcpp::Parameter & parameter, bool & value) {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        reject(parameter.get_name() + " must be a bool");
        return false;
      }
      value = parameter.as_bool();
      return true;
    };

  double path_timeout = path_timeout_;
  double wheel_base = wheel_base_;
  double time_step = time_step_;
  int horizon_steps = horizon_steps_;
  int batch_size = batch_size_;
  int iteration_count = iteration_count_;
  int random_seed = random_seed_;

  double target_speed = target_speed_;
  double min_speed = min_speed_;
  double tracking_min_command_speed = tracking_min_command_speed_;
  double max_speed = max_speed_;
  double max_steering_angle = max_steering_angle_;
  double max_accel = max_accel_;
  double max_steering_rate = max_steering_rate_;
  double speed_noise_std = speed_noise_std_;
  double steering_noise_std = steering_noise_std_;
  double temperature = temperature_;

  double path_distance_weight = path_distance_weight_;
  double heading_weight = heading_weight_;
  double heading_lookahead_distance = heading_lookahead_distance_;
  double heading_lookahead_weight = heading_lookahead_weight_;
  double steering_feedforward_lookahead_distance = steering_feedforward_lookahead_distance_;
  double steering_feedforward_weight = steering_feedforward_weight_;
  double steering_feedforward_command_gain = steering_feedforward_command_gain_;
  double curvature_lookahead_distance = curvature_lookahead_distance_;
  double curvature_speed_weight = curvature_speed_weight_;
  double curvature_speed_gain = curvature_speed_gain_;
  double curvature_min_speed = curvature_min_speed_;
  double lateral_accel_limit = lateral_accel_limit_;
  bool use_lateral_accel_speed_limit = use_lateral_accel_speed_limit_;
  bool use_corner_speed_as_speed_target = use_corner_speed_as_speed_target_;
  bool limit_command_speed_to_corner = limit_command_speed_to_corner_;
  double corner_speed_margin = corner_speed_margin_;
  double terminal_distance_weight = terminal_distance_weight_;
  double terminal_heading_weight = terminal_heading_weight_;
  double speed_weight = speed_weight_;
  double steering_weight = steering_weight_;
  double steering_rate_weight = steering_rate_weight_;
  double accel_weight = accel_weight_;
  double progress_weight = progress_weight_;

  double min_path_segment_length = min_path_segment_length_;
  double min_path_forward_x = min_path_forward_x_;
  int min_path_points = min_path_points_;
  bool visualize_markers = visualize_markers_;
  bool visualize_rollouts = visualize_rollouts_;
  bool publish_debug_status = publish_debug_status_;
  int max_visualized_rollouts = max_visualized_rollouts_;
  int rollout_visualization_stride = rollout_visualization_stride_;
  double marker_line_width = marker_line_width_;
  double rollout_line_width = rollout_line_width_;
  double rollout_alpha = rollout_alpha_;
  double marker_height = marker_height_;
  std::string command_frame = command_frame_;
  std::string tuning_profile = tuning_profile_;

  bool reset_sequence = false;
  bool reseed_rng = false;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();

    if (
      name == "local_path_topic" ||
      name == "command_topic" ||
      name == "trajectory_topic" ||
      name == "marker_topic" ||
      name == "debug_topic" ||
      name == "update_rate")
    {
      return reject(name + " requires node restart");
    } else if (name == "command_frame") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("command_frame must be a string");
      }
      command_frame = parameter.as_string();
      if (command_frame.empty()) {
        return reject("command_frame must not be empty");
      }
    } else if (name == "tuning_profile") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("tuning_profile must be a string");
      }
      tuning_profile = parameter.as_string();
      if (tuning_profile.empty()) {
        return reject("tuning_profile must not be empty");
      }
    } else if (name == "path_timeout") {
      if (!read_double(parameter, path_timeout)) {return result;}
      if (path_timeout < 0.0) {return reject("path_timeout must be >= 0");}
    } else if (name == "wheel_base") {
      if (!read_double(parameter, wheel_base)) {return result;}
      if (wheel_base <= 0.0) {return reject("wheel_base must be > 0");}
    } else if (name == "time_step") {
      if (!read_double(parameter, time_step)) {return result;}
      if (time_step <= 0.0) {return reject("time_step must be > 0");}
      reset_sequence = true;
    } else if (name == "horizon_steps") {
      if (!read_int(parameter, horizon_steps)) {return result;}
      if (horizon_steps < 2) {return reject("horizon_steps must be >= 2");}
      reset_sequence = true;
    } else if (name == "batch_size") {
      if (!read_int(parameter, batch_size)) {return result;}
      if (batch_size < 1) {return reject("batch_size must be >= 1");}
    } else if (name == "iteration_count") {
      if (!read_int(parameter, iteration_count)) {return result;}
      if (iteration_count < 1) {return reject("iteration_count must be >= 1");}
    } else if (name == "random_seed") {
      if (!read_int(parameter, random_seed)) {return result;}
      if (random_seed < 0) {return reject("random_seed must be >= 0");}
      reseed_rng = true;
    } else if (name == "target_speed") {
      if (!read_double(parameter, target_speed)) {return result;}
      reset_sequence = true;
    } else if (name == "min_speed") {
      if (!read_double(parameter, min_speed)) {return result;}
      reset_sequence = true;
    } else if (name == "tracking_min_command_speed") {
      if (!read_double(parameter, tracking_min_command_speed)) {return result;}
    } else if (name == "max_speed") {
      if (!read_double(parameter, max_speed)) {return result;}
      reset_sequence = true;
    } else if (name == "max_steering_angle") {
      if (!read_double(parameter, max_steering_angle)) {return result;}
      if (max_steering_angle <= 0.0) {return reject("max_steering_angle must be > 0");}
    } else if (name == "max_accel") {
      if (!read_double(parameter, max_accel)) {return result;}
      if (max_accel < 0.0) {return reject("max_accel must be >= 0");}
    } else if (name == "max_steering_rate") {
      if (!read_double(parameter, max_steering_rate)) {return result;}
      if (max_steering_rate < 0.0) {return reject("max_steering_rate must be >= 0");}
    } else if (name == "speed_noise_std") {
      if (!read_double(parameter, speed_noise_std)) {return result;}
      if (speed_noise_std < 0.0) {return reject("speed_noise_std must be >= 0");}
    } else if (name == "steering_noise_std") {
      if (!read_double(parameter, steering_noise_std)) {return result;}
      if (steering_noise_std < 0.0) {return reject("steering_noise_std must be >= 0");}
    } else if (name == "temperature") {
      if (!read_double(parameter, temperature)) {return result;}
      if (temperature <= 0.0) {return reject("temperature must be > 0");}
    } else if (name == "path_distance_weight") {
      if (!read_double(parameter, path_distance_weight)) {return result;}
      if (path_distance_weight < 0.0) {return reject("path_distance_weight must be >= 0");}
    } else if (name == "heading_weight") {
      if (!read_double(parameter, heading_weight)) {return result;}
      if (heading_weight < 0.0) {return reject("heading_weight must be >= 0");}
    } else if (name == "heading_lookahead_distance") {
      if (!read_double(parameter, heading_lookahead_distance)) {return result;}
      if (heading_lookahead_distance < 0.0) {
        return reject("heading_lookahead_distance must be >= 0");
      }
    } else if (name == "heading_lookahead_weight") {
      if (!read_double(parameter, heading_lookahead_weight)) {return result;}
      if (heading_lookahead_weight < 0.0) {
        return reject("heading_lookahead_weight must be >= 0");
      }
    } else if (name == "steering_feedforward_lookahead_distance") {
      if (!read_double(parameter, steering_feedforward_lookahead_distance)) {return result;}
      if (steering_feedforward_lookahead_distance < 0.0) {
        return reject("steering_feedforward_lookahead_distance must be >= 0");
      }
    } else if (name == "steering_feedforward_weight") {
      if (!read_double(parameter, steering_feedforward_weight)) {return result;}
      if (steering_feedforward_weight < 0.0) {
        return reject("steering_feedforward_weight must be >= 0");
      }
    } else if (name == "steering_feedforward_command_gain") {
      if (!read_double(parameter, steering_feedforward_command_gain)) {return result;}
      if (steering_feedforward_command_gain < 0.0 || steering_feedforward_command_gain > 1.0) {
        return reject("steering_feedforward_command_gain must be in [0, 1]");
      }
    } else if (name == "curvature_lookahead_distance") {
      if (!read_double(parameter, curvature_lookahead_distance)) {return result;}
      if (curvature_lookahead_distance < 0.0) {
        return reject("curvature_lookahead_distance must be >= 0");
      }
      reset_sequence = true;
    } else if (name == "curvature_speed_weight") {
      if (!read_double(parameter, curvature_speed_weight)) {return result;}
      if (curvature_speed_weight < 0.0) {
        return reject("curvature_speed_weight must be >= 0");
      }
    } else if (name == "curvature_speed_gain") {
      if (!read_double(parameter, curvature_speed_gain)) {return result;}
      if (curvature_speed_gain < 0.0) {
        return reject("curvature_speed_gain must be >= 0");
      }
      reset_sequence = true;
    } else if (name == "curvature_min_speed") {
      if (!read_double(parameter, curvature_min_speed)) {return result;}
      if (curvature_min_speed < 0.0) {
        return reject("curvature_min_speed must be >= 0");
      }
      reset_sequence = true;
    } else if (name == "lateral_accel_limit") {
      if (!read_double(parameter, lateral_accel_limit)) {return result;}
      if (lateral_accel_limit < 0.0) {
        return reject("lateral_accel_limit must be >= 0");
      }
      reset_sequence = true;
    } else if (name == "use_lateral_accel_speed_limit") {
      if (!read_bool(parameter, use_lateral_accel_speed_limit)) {return result;}
      reset_sequence = true;
    } else if (name == "use_corner_speed_as_speed_target") {
      if (!read_bool(parameter, use_corner_speed_as_speed_target)) {return result;}
      reset_sequence = true;
    } else if (name == "limit_command_speed_to_corner") {
      if (!read_bool(parameter, limit_command_speed_to_corner)) {return result;}
    } else if (name == "corner_speed_margin") {
      if (!read_double(parameter, corner_speed_margin)) {return result;}
      if (corner_speed_margin < 0.0) {
        return reject("corner_speed_margin must be >= 0");
      }
      reset_sequence = true;
    } else if (name == "terminal_distance_weight") {
      if (!read_double(parameter, terminal_distance_weight)) {return result;}
      if (terminal_distance_weight < 0.0) {
        return reject("terminal_distance_weight must be >= 0");
      }
    } else if (name == "terminal_heading_weight") {
      if (!read_double(parameter, terminal_heading_weight)) {return result;}
      if (terminal_heading_weight < 0.0) {
        return reject("terminal_heading_weight must be >= 0");
      }
    } else if (name == "speed_weight") {
      if (!read_double(parameter, speed_weight)) {return result;}
      if (speed_weight < 0.0) {return reject("speed_weight must be >= 0");}
    } else if (name == "steering_weight") {
      if (!read_double(parameter, steering_weight)) {return result;}
      if (steering_weight < 0.0) {return reject("steering_weight must be >= 0");}
    } else if (name == "steering_rate_weight") {
      if (!read_double(parameter, steering_rate_weight)) {return result;}
      if (steering_rate_weight < 0.0) {return reject("steering_rate_weight must be >= 0");}
    } else if (name == "accel_weight") {
      if (!read_double(parameter, accel_weight)) {return result;}
      if (accel_weight < 0.0) {return reject("accel_weight must be >= 0");}
    } else if (name == "progress_weight") {
      if (!read_double(parameter, progress_weight)) {return result;}
      if (progress_weight < 0.0) {return reject("progress_weight must be >= 0");}
    } else if (name == "min_path_segment_length") {
      if (!read_double(parameter, min_path_segment_length)) {return result;}
      if (min_path_segment_length <= 0.0) {
        return reject("min_path_segment_length must be > 0");
      }
    } else if (name == "min_path_forward_x") {
      if (!read_double(parameter, min_path_forward_x)) {return result;}
    } else if (name == "min_path_points") {
      if (!read_int(parameter, min_path_points)) {return result;}
      if (min_path_points < 2) {return reject("min_path_points must be >= 2");}
    } else if (name == "visualize_markers") {
      if (!read_bool(parameter, visualize_markers)) {return result;}
    } else if (name == "visualize_rollouts") {
      if (!read_bool(parameter, visualize_rollouts)) {return result;}
    } else if (name == "publish_debug_status") {
      if (!read_bool(parameter, publish_debug_status)) {return result;}
    } else if (name == "max_visualized_rollouts") {
      if (!read_int(parameter, max_visualized_rollouts)) {return result;}
      if (max_visualized_rollouts < 0) {
        return reject("max_visualized_rollouts must be >= 0");
      }
    } else if (name == "rollout_visualization_stride") {
      if (!read_int(parameter, rollout_visualization_stride)) {return result;}
      if (rollout_visualization_stride < 1) {
        return reject("rollout_visualization_stride must be >= 1");
      }
    } else if (name == "marker_line_width") {
      if (!read_double(parameter, marker_line_width)) {return result;}
      if (marker_line_width <= 0.0) {return reject("marker_line_width must be > 0");}
    } else if (name == "rollout_line_width") {
      if (!read_double(parameter, rollout_line_width)) {return result;}
      if (rollout_line_width <= 0.0) {return reject("rollout_line_width must be > 0");}
    } else if (name == "rollout_alpha") {
      if (!read_double(parameter, rollout_alpha)) {return result;}
      if (rollout_alpha <= 0.0 || rollout_alpha > 1.0) {
        return reject("rollout_alpha must be in (0, 1]");
      }
    } else if (name == "marker_height") {
      if (!read_double(parameter, marker_height)) {return result;}
    }
  }

  if (min_speed < 0.0) {
    return reject("min_speed must be >= 0");
  }
  if (max_speed < 0.0) {
    return reject("max_speed must be >= 0");
  }
  if (min_speed > max_speed) {
    return reject("min_speed must be <= max_speed");
  }
  if (tracking_min_command_speed < 0.0) {
    return reject("tracking_min_command_speed must be >= 0");
  }
  if (tracking_min_command_speed > max_speed) {
    return reject("tracking_min_command_speed must be <= max_speed");
  }
  if (target_speed < min_speed || target_speed > max_speed) {
    return reject("target_speed must be between min_speed and max_speed");
  }
  curvature_min_speed = clampValue(curvature_min_speed, min_speed, target_speed);

  path_timeout_ = path_timeout;
  wheel_base_ = std::max(0.05, wheel_base);
  time_step_ = std::max(0.01, time_step);
  horizon_steps_ = horizon_steps;
  batch_size_ = batch_size;
  iteration_count_ = iteration_count;
  random_seed_ = random_seed;

  target_speed_ = target_speed;
  min_speed_ = min_speed;
  tracking_min_command_speed_ = tracking_min_command_speed;
  max_speed_ = max_speed;
  max_steering_angle_ = max_steering_angle;
  max_accel_ = max_accel;
  max_steering_rate_ = max_steering_rate;
  speed_noise_std_ = speed_noise_std;
  steering_noise_std_ = steering_noise_std;
  temperature_ = std::max(1.0e-6, temperature);

  path_distance_weight_ = path_distance_weight;
  heading_weight_ = heading_weight;
  heading_lookahead_distance_ = heading_lookahead_distance;
  heading_lookahead_weight_ = heading_lookahead_weight;
  steering_feedforward_lookahead_distance_ = steering_feedforward_lookahead_distance;
  steering_feedforward_weight_ = steering_feedforward_weight;
  steering_feedforward_command_gain_ = steering_feedforward_command_gain;
  curvature_lookahead_distance_ = curvature_lookahead_distance;
  curvature_speed_weight_ = curvature_speed_weight;
  curvature_speed_gain_ = curvature_speed_gain;
  curvature_min_speed_ = curvature_min_speed;
  lateral_accel_limit_ = lateral_accel_limit;
  use_lateral_accel_speed_limit_ = use_lateral_accel_speed_limit;
  use_corner_speed_as_speed_target_ = use_corner_speed_as_speed_target;
  limit_command_speed_to_corner_ = limit_command_speed_to_corner;
  corner_speed_margin_ = corner_speed_margin;
  terminal_distance_weight_ = terminal_distance_weight;
  terminal_heading_weight_ = terminal_heading_weight;
  speed_weight_ = speed_weight;
  steering_weight_ = steering_weight;
  steering_rate_weight_ = steering_rate_weight;
  accel_weight_ = accel_weight;
  progress_weight_ = progress_weight;

  min_path_segment_length_ = min_path_segment_length;
  min_path_forward_x_ = min_path_forward_x;
  min_path_points_ = min_path_points;
  visualize_markers_ = visualize_markers;
  visualize_rollouts_ = visualize_rollouts;
  publish_debug_status_ = publish_debug_status;
  max_visualized_rollouts_ = max_visualized_rollouts;
  rollout_visualization_stride_ = rollout_visualization_stride;
  marker_line_width_ = marker_line_width;
  rollout_line_width_ = rollout_line_width;
  rollout_alpha_ = rollout_alpha;
  marker_height_ = marker_height;
  command_frame_ = command_frame;
  tuning_profile_ = tuning_profile;

  if (reseed_rng) {
    const unsigned int seed = random_seed_ == 0 ?
      std::random_device{}() : static_cast<unsigned int>(random_seed_);
    rng_.seed(seed);
  }
  if (reset_sequence) {
    resetControlSequence();
  }

  RCLCPP_INFO(
    get_logger(),
    "Updated MPPI params: profile=%s, speed=%.2f m/s, floor=%.2f m/s, horizon=%d x %.2fs, heading_w=%.2f, lookahead_heading_w=%.2f, turn_speed_w=%.2f",
    tuning_profile_.c_str(),
    target_speed_, tracking_min_command_speed_, horizon_steps_, time_step_,
    heading_weight_, heading_lookahead_weight_,
    curvature_speed_weight_);

  return result;
}

void MppiPlanner::localPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  latest_path_ = *msg;
  has_path_ = true;
  last_path_time_ = now();
}

void MppiPlanner::timerCallback()
{
  if (!has_path_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Waiting for local path on '%s'", local_path_topic_.c_str());
    publishStop(command_frame_);
    return;
  }

  const std::string frame_id =
    !latest_path_.header.frame_id.empty() ? latest_path_.header.frame_id : command_frame_;

  if (path_timeout_ > 0.0) {
    const double age = (now() - last_path_time_).seconds();
    if (age > path_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Local path is stale: %.2f s old on '%s'", age, local_path_topic_.c_str());
      publishStop(frame_id);
      return;
    }
  }

  ackermann_msgs::msg::AckermannDriveStamped cmd;
  nav_msgs::msg::Path trajectory_path;
  if (!computeCommand(latest_path_, cmd, trajectory_path)) {
    publishStop(frame_id);
    resetControlSequence();
    return;
  }

  pub_command_->publish(cmd);
  pub_trajectory_->publish(trajectory_path);
  if (visualize_markers_) {
    publishMarkers(trajectory_path);
  }
  if (publish_debug_status_) {
    publishDebugStatus(cmd, trajectory_path);
  }

  previous_command_.speed = cmd.drive.speed;
  previous_command_.steering = cmd.drive.steering_angle;
  has_previous_command_ = true;
  shiftControlSequence();
}

bool MppiPlanner::computeCommand(
  const nav_msgs::msg::Path & path,
  ackermann_msgs::msg::AckermannDriveStamped & cmd,
  nav_msgs::msg::Path & trajectory_path)
{
  if (path.poses.size() < static_cast<size_t>(min_path_points_)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Received local path with fewer than %d poses", min_path_points_);
    return false;
  }

  std::vector<PathSegment> segments;
  if (!buildPathSegments(path, segments)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No valid forward path segments found for MPPI");
    return false;
  }
  updatePathDiagnostics(segments);

  ensureControlSequence();
  sampled_rollouts_.clear();
  best_sampled_rollout_.states.clear();
  best_sampled_rollout_.cost = std::numeric_limits<double>::max();
  has_best_sampled_rollout_ = false;
  last_best_sampled_rollout_cost_ = std::numeric_limits<double>::max();

  std::vector<std::vector<Control>> noises(
    static_cast<size_t>(batch_size_),
    std::vector<Control>(static_cast<size_t>(horizon_steps_)));
  std::vector<double> costs(static_cast<size_t>(batch_size_), 0.0);

  for (int iteration = 0; iteration < iteration_count_; ++iteration) {
    for (int sample = 0; sample < batch_size_; ++sample) {
      auto & sample_noise = noises[static_cast<size_t>(sample)];
      for (int step = 0; step < horizon_steps_; ++step) {
        sample_noise[static_cast<size_t>(step)].speed =
          speed_noise_std_ * normal_distribution_(rng_);
        sample_noise[static_cast<size_t>(step)].steering =
          steering_noise_std_ * normal_distribution_(rng_);
      }

      const bool collect_rollout =
        visualize_markers_ &&
        visualize_rollouts_ &&
        iteration + 1 == iteration_count_ &&
        max_visualized_rollouts_ > 0 &&
        sample % rollout_visualization_stride_ == 0 &&
        sampled_rollouts_.size() < static_cast<size_t>(max_visualized_rollouts_);
      const bool collect_best_sample =
        visualize_markers_ &&
        visualize_rollouts_ &&
        iteration + 1 == iteration_count_;

      std::vector<State> rollout_states;
      const double cost = rolloutTrajectory(
        segments,
        &sample_noise,
        collect_rollout ? &rollout_states : nullptr);
      costs[static_cast<size_t>(sample)] = cost;

      if (collect_rollout) {
        RolloutVisualization rollout;
        rollout.states = rollout_states;
        rollout.cost = cost;
        sampled_rollouts_.push_back(rollout);
      }
      if (
        collect_best_sample &&
        std::isfinite(cost) &&
        cost < best_sampled_rollout_.cost)
      {
        if (rollout_states.empty()) {
          std::vector<State> best_states;
          rolloutTrajectory(segments, &sample_noise, &best_states);
          best_sampled_rollout_.states = best_states;
        } else {
          best_sampled_rollout_.states = rollout_states;
        }
        best_sampled_rollout_.cost = cost;
        has_best_sampled_rollout_ = !best_sampled_rollout_.states.empty();
      }
    }

    double min_cost = std::numeric_limits<double>::max();
    for (const auto cost : costs) {
      min_cost = std::min(min_cost, cost);
    }

    std::vector<Control> weighted_noise(static_cast<size_t>(horizon_steps_));
    double weight_sum = 0.0;
    for (int sample = 0; sample < batch_size_; ++sample) {
      const double cost = costs[static_cast<size_t>(sample)];
      const double weight = std::exp(-(cost - min_cost) / temperature_);
      if (!std::isfinite(weight)) {
        continue;
      }

      weight_sum += weight;
      const auto & sample_noise = noises[static_cast<size_t>(sample)];
      for (int step = 0; step < horizon_steps_; ++step) {
        weighted_noise[static_cast<size_t>(step)].speed +=
          weight * sample_noise[static_cast<size_t>(step)].speed;
        weighted_noise[static_cast<size_t>(step)].steering +=
          weight * sample_noise[static_cast<size_t>(step)].steering;
      }
    }

    if (weight_sum > kEpsilon) {
      Control previous = has_previous_command_ ? previous_command_ : Control{target_speed_, 0.0};
      for (int step = 0; step < horizon_steps_; ++step) {
        auto & control = nominal_controls_[static_cast<size_t>(step)];
        control.speed += weighted_noise[static_cast<size_t>(step)].speed / weight_sum;
        control.steering += weighted_noise[static_cast<size_t>(step)].steering / weight_sum;
        control = boundedControl(control, previous);
        previous = control;
      }
    }
  }

  std::vector<State> selected_trajectory;
  double selected_cost = rolloutTrajectory(segments, nullptr, &selected_trajectory);
  if (!std::isfinite(selected_cost) || selected_trajectory.empty()) {
    return false;
  }

  const std::string frame_id = !path.header.frame_id.empty() ? path.header.frame_id : command_frame_;

  Control command = nominal_controls_.front();
  last_command_speed_before_limit_ = command.speed;
  last_command_speed_limited_ = false;
  last_command_speed_floor_applied_ = false;
  last_tracking_speed_floor_ = 0.0;
  if (limit_command_speed_to_corner_) {
    const double corner_speed_limit = clampValue(
      last_corner_speed_target_ + corner_speed_margin_,
      min_speed_,
      target_speed_);
    if (command.speed > corner_speed_limit) {
      Control limited_command = command;
      limited_command.speed = corner_speed_limit;
      const Control previous = has_previous_command_ ?
        previous_command_ : Control{target_speed_, 0.0};
      command = boundedControl(limited_command, previous);
      last_command_speed_limited_ =
        command.speed < last_command_speed_before_limit_ - kEpsilon;
    }
  }
  if (steering_feedforward_command_gain_ > kEpsilon) {
    Control feedforward_command = command;
    feedforward_command.steering =
      command.steering +
      steering_feedforward_command_gain_ * (last_steering_target_ - command.steering);
    const Control previous = has_previous_command_ ?
      previous_command_ : Control{target_speed_, 0.0};
    command = boundedControl(feedforward_command, previous);
  }
  if (tracking_min_command_speed_ > kEpsilon) {
    const double corner_speed_limit = limit_command_speed_to_corner_ ?
      clampValue(last_corner_speed_target_ + corner_speed_margin_, min_speed_, target_speed_) :
      target_speed_;
    const double target_speed_limit =
      clampValue(last_effective_speed_target_, min_speed_, target_speed_);
    const double tracking_speed_floor = std::min(
      clampValue(tracking_min_command_speed_, min_speed_, max_speed_),
      std::min(corner_speed_limit, target_speed_limit));
    last_tracking_speed_floor_ = tracking_speed_floor;

    if (tracking_speed_floor > min_speed_ + kEpsilon &&
      command.speed < tracking_speed_floor - kEpsilon)
    {
      const double speed_before_floor = command.speed;
      double floored_speed = tracking_speed_floor;
      if (max_accel_ > kEpsilon) {
        const double previous_speed = has_previous_command_ ?
          previous_command_.speed : command.speed;
        const double max_delta = max_accel_ * time_step_;
        floored_speed = clampValue(
          floored_speed, previous_speed - max_delta, previous_speed + max_delta);
      }
      command.speed = clampValue(floored_speed, min_speed_, max_speed_);
      last_command_speed_floor_applied_ = command.speed > speed_before_floor + kEpsilon;
    }
  }
  if (!nominal_controls_.empty()) {
    const Control planned_command = nominal_controls_.front();
    const bool command_changed =
      std::abs(command.speed - planned_command.speed) > kEpsilon ||
      std::abs(command.steering - planned_command.steering) > kEpsilon;
    if (command_changed) {
      nominal_controls_.front() = command;
      selected_trajectory.clear();
      selected_cost = rolloutTrajectory(segments, nullptr, &selected_trajectory);
      if (!std::isfinite(selected_cost) || selected_trajectory.empty()) {
        return false;
      }
    }
  }
  trajectory_path = statesToPath(selected_trajectory, frame_id);

  cmd.header.stamp = now();
  cmd.header.frame_id = frame_id;
  cmd.drive.speed = static_cast<float>(command.speed);
  cmd.drive.steering_angle = static_cast<float>(command.steering);
  last_command_steering_ = command.steering;

  last_selected_cost_ = selected_cost;
  last_best_sampled_rollout_cost_ = has_best_sampled_rollout_ ?
    best_sampled_rollout_.cost : selected_cost;
  last_best_rollout_cost_ = selected_cost;
  last_worst_rollout_cost_ = selected_cost;
  if (has_best_sampled_rollout_ && std::isfinite(best_sampled_rollout_.cost)) {
    last_best_rollout_cost_ = std::min(last_best_rollout_cost_, best_sampled_rollout_.cost);
  }
  for (const auto & rollout : sampled_rollouts_) {
    if (!std::isfinite(rollout.cost)) {
      continue;
    }
    last_best_rollout_cost_ = std::min(last_best_rollout_cost_, rollout.cost);
    last_worst_rollout_cost_ = std::max(last_worst_rollout_cost_, rollout.cost);
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "mppi cost=%.2f, speed=%.2f m/s, steer=%.1f deg",
    selected_cost, command.speed, command.steering * 180.0 / kPi);

  return true;
}

bool MppiPlanner::buildPathSegments(
  const nav_msgs::msg::Path & path,
  std::vector<PathSegment> & segments) const
{
  segments.clear();
  double cumulative_s = 0.0;

  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & start = path.poses[i].pose.position;
    const auto & end = path.poses[i + 1].pose.position;

    if (std::max(start.x, end.x) < min_path_forward_x_) {
      continue;
    }

    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length = std::hypot(dx, dy);
    if (length < min_path_segment_length_) {
      continue;
    }

    PathSegment segment;
    segment.start = start;
    segment.end = end;
    segment.yaw = std::atan2(dy, dx);
    segment.length = length;
    segment.cumulative_s = cumulative_s;
    segments.push_back(segment);

    cumulative_s += length;
  }

  return !segments.empty();
}

double MppiPlanner::rolloutTrajectory(
  const std::vector<PathSegment> & segments,
  const std::vector<Control> * noise,
  std::vector<State> * trajectory) const
{
  State state;
  Control previous = has_previous_command_ ? previous_command_ : Control{target_speed_, 0.0};
  double cost = 0.0;
  PathProjection projection;

  if (trajectory != nullptr) {
    trajectory->clear();
    trajectory->reserve(static_cast<size_t>(horizon_steps_));
  }

  for (int step = 0; step < horizon_steps_; ++step) {
    Control desired = nominal_controls_[static_cast<size_t>(step)];
    if (noise != nullptr) {
      desired.speed += (*noise)[static_cast<size_t>(step)].speed;
      desired.steering += (*noise)[static_cast<size_t>(step)].steering;
    }

    const Control control = boundedControl(desired, previous);
    state = simulateStep(state, control);
    projection = projectToPath(segments, state.x, state.y);

    double local_speed_target = target_speed_;
    if (!projection.valid) {
      cost += 1000.0;
    } else {
      const double heading_error = normalizeAngle(state.yaw - projection.heading);
      cost += path_distance_weight_ * projection.distance * projection.distance;
      cost += heading_weight_ * heading_error * heading_error;

      if (heading_lookahead_weight_ > kEpsilon && heading_lookahead_distance_ > kEpsilon) {
        const auto lookahead = samplePathAtProgress(
          segments,
          projection.progress + heading_lookahead_distance_);
        if (lookahead.valid) {
          const double lookahead_heading_error = normalizeAngle(state.yaw - lookahead.heading);
          cost += heading_lookahead_weight_ *
            lookahead_heading_error * lookahead_heading_error;
        }
      }

      if (
        steering_feedforward_weight_ > kEpsilon &&
        steering_feedforward_lookahead_distance_ > kEpsilon)
      {
        const auto lookahead = samplePathAtProgress(
          segments,
          projection.progress + steering_feedforward_lookahead_distance_);
        if (lookahead.valid) {
          const double actual_distance = std::max(
            kEpsilon,
            lookahead.progress - projection.progress);
          const double heading_change =
            normalizeAngle(lookahead.heading - projection.heading);
          const double signed_curvature = heading_change / actual_distance;
          const double steering_target = clampValue(
            std::atan(wheel_base_ * signed_curvature),
            -max_steering_angle_,
            max_steering_angle_);
          const double steering_target_error = control.steering - steering_target;
          cost += steering_feedforward_weight_ *
            steering_target_error * steering_target_error;
        }
      }

      if (curvature_speed_weight_ > kEpsilon && curvature_lookahead_distance_ > kEpsilon) {
        const auto lookahead = samplePathAtProgress(
          segments,
          projection.progress + curvature_lookahead_distance_);
        if (lookahead.valid) {
          const double heading_change = std::fabs(normalizeAngle(lookahead.heading - projection.heading));
          const double actual_distance = std::max(
            kEpsilon,
            lookahead.progress - projection.progress);
          const double corner_speed = cornerSpeedFromHeadingChange(
            heading_change,
            actual_distance);
          if (use_corner_speed_as_speed_target_) {
            local_speed_target = std::min(local_speed_target, corner_speed);
          }
          const double corner_speed_error = control.speed - corner_speed;
          cost += curvature_speed_weight_ * corner_speed_error * corner_speed_error;
        }
      }

      if (step + 1 == horizon_steps_) {
        cost += terminal_distance_weight_ * projection.distance * projection.distance;
        cost += terminal_heading_weight_ * heading_error * heading_error;
        cost -= progress_weight_ * projection.progress;
      }
    }

    const double speed_error = control.speed - local_speed_target;
    const double steering_delta = control.steering - previous.steering;
    const double speed_delta = control.speed - previous.speed;
    cost += speed_weight_ * speed_error * speed_error;
    cost += steering_weight_ * control.steering * control.steering;
    cost += steering_rate_weight_ * steering_delta * steering_delta;
    cost += accel_weight_ * speed_delta * speed_delta;

    if (trajectory != nullptr) {
      trajectory->push_back(state);
    }

    previous = control;
  }

  return cost;
}

void MppiPlanner::updatePathDiagnostics(const std::vector<PathSegment> & segments)
{
  last_path_distance_ = 0.0;
  last_path_progress_ = 0.0;
  last_path_heading_ = 0.0;
  last_heading_lookahead_error_ = 0.0;
  last_heading_lookahead_actual_distance_ = 0.0;
  last_curvature_heading_change_ = 0.0;
  last_curvature_heading_change_signed_ = 0.0;
  last_curvature_lookahead_actual_distance_ = 0.0;
  last_steering_target_ = 0.0;
  last_corner_speed_target_ = target_speed_;
  last_effective_speed_target_ = target_speed_;
  last_lateral_accel_speed_target_ = target_speed_;
  has_path_diagnostics_ = false;
  has_heading_lookahead_point_ = false;
  has_curvature_lookahead_point_ = false;

  const auto projection = projectToPath(segments, 0.0, 0.0);
  if (!projection.valid) {
    return;
  }

  has_path_diagnostics_ = true;
  last_projection_point_ = projection.point;
  last_path_distance_ = projection.distance;
  last_path_progress_ = projection.progress;
  last_path_heading_ = projection.heading;

  if (heading_lookahead_distance_ > kEpsilon) {
    const auto lookahead = samplePathAtProgress(
      segments,
      projection.progress + heading_lookahead_distance_);
    if (lookahead.valid) {
      has_heading_lookahead_point_ = true;
      last_heading_lookahead_point_ = lookahead.point;
      last_heading_lookahead_error_ = normalizeAngle(lookahead.heading);
      last_heading_lookahead_actual_distance_ =
        std::max(0.0, lookahead.progress - projection.progress);
    }
  }

  if (curvature_lookahead_distance_ > kEpsilon) {
    const auto lookahead = samplePathAtProgress(
      segments,
      projection.progress + curvature_lookahead_distance_);
    if (lookahead.valid) {
      has_curvature_lookahead_point_ = true;
      last_curvature_lookahead_point_ = lookahead.point;
      last_curvature_lookahead_actual_distance_ =
        std::max(0.0, lookahead.progress - projection.progress);
      last_curvature_heading_change_signed_ =
        normalizeAngle(lookahead.heading - projection.heading);
      last_curvature_heading_change_ = std::fabs(last_curvature_heading_change_signed_);
      if (last_curvature_lookahead_actual_distance_ > kEpsilon) {
        const double signed_curvature =
          last_curvature_heading_change_signed_ / last_curvature_lookahead_actual_distance_;
        last_steering_target_ = clampValue(
          std::atan(wheel_base_ * signed_curvature),
          -max_steering_angle_,
          max_steering_angle_);
      }
      last_corner_speed_target_ = cornerSpeedFromHeadingChange(
        last_curvature_heading_change_,
        last_curvature_lookahead_actual_distance_);
      if (
        use_lateral_accel_speed_limit_ &&
        lateral_accel_limit_ > kEpsilon &&
        last_curvature_lookahead_actual_distance_ > kEpsilon)
      {
        const double curvature =
          last_curvature_heading_change_ / last_curvature_lookahead_actual_distance_;
        if (curvature > kEpsilon) {
          const double min_corner_speed = clampValue(curvature_min_speed_, min_speed_, target_speed_);
          last_lateral_accel_speed_target_ = clampValue(
            std::sqrt(lateral_accel_limit_ / curvature),
            min_corner_speed,
            target_speed_);
        }
      }
      if (use_corner_speed_as_speed_target_) {
        last_effective_speed_target_ = last_corner_speed_target_;
      }
    }
  }
}

MppiPlanner::PathProjection MppiPlanner::projectToPath(
  const std::vector<PathSegment> & segments,
  const double x,
  const double y) const
{
  PathProjection best;
  double best_distance_sq = std::numeric_limits<double>::max();

  for (const auto & segment : segments) {
    const double dx = segment.end.x - segment.start.x;
    const double dy = segment.end.y - segment.start.y;
    const double length_sq = dx * dx + dy * dy;
    if (length_sq <= kEpsilon) {
      continue;
    }

    const double t = clampValue(
      ((x - segment.start.x) * dx + (y - segment.start.y) * dy) / length_sq,
      0.0,
      1.0);
    geometry_msgs::msg::Point point;
    point.x = segment.start.x + t * dx;
    point.y = segment.start.y + t * dy;
    point.z = segment.start.z + t * (segment.end.z - segment.start.z);

    const double error_x = x - point.x;
    const double error_y = y - point.y;
    const double distance_sq = error_x * error_x + error_y * error_y;
    if (distance_sq >= best_distance_sq) {
      continue;
    }

    best.valid = true;
    best.point = point;
    best.distance = std::sqrt(distance_sq);
    best.heading = segment.yaw;
    best.progress = segment.cumulative_s + t * segment.length;
    best_distance_sq = distance_sq;
  }

  return best;
}

MppiPlanner::PathProjection MppiPlanner::samplePathAtProgress(
  const std::vector<PathSegment> & segments,
  const double progress) const
{
  PathProjection sample;
  if (segments.empty()) {
    return sample;
  }

  const double target_progress = std::max(0.0, progress);
  for (const auto & segment : segments) {
    const double segment_end_s = segment.cumulative_s + segment.length;
    if (target_progress > segment_end_s) {
      continue;
    }

    const double t = segment.length > kEpsilon ?
      clampValue((target_progress - segment.cumulative_s) / segment.length, 0.0, 1.0) : 0.0;
    sample.valid = true;
    sample.point.x = segment.start.x + t * (segment.end.x - segment.start.x);
    sample.point.y = segment.start.y + t * (segment.end.y - segment.start.y);
    sample.point.z = segment.start.z + t * (segment.end.z - segment.start.z);
    sample.heading = segment.yaw;
    sample.progress = segment.cumulative_s + t * segment.length;
    return sample;
  }

  const auto & last = segments.back();
  sample.valid = true;
  sample.point = last.end;
  sample.heading = last.yaw;
  sample.progress = last.cumulative_s + last.length;
  return sample;
}

double MppiPlanner::cornerSpeedFromHeadingChange(
  const double heading_change,
  const double lookahead_distance) const
{
  const double min_corner_speed = clampValue(curvature_min_speed_, min_speed_, target_speed_);
  double corner_speed = clampValue(
    target_speed_ / (1.0 + curvature_speed_gain_ * heading_change),
    min_corner_speed,
    target_speed_);

  if (
    use_lateral_accel_speed_limit_ &&
    lateral_accel_limit_ > kEpsilon &&
    lookahead_distance > kEpsilon)
  {
    const double curvature = heading_change / lookahead_distance;
    if (curvature > kEpsilon) {
      const double lateral_speed = std::sqrt(lateral_accel_limit_ / curvature);
      corner_speed = std::min(corner_speed, lateral_speed);
    }
  }

  return clampValue(corner_speed, min_corner_speed, target_speed_);
}

MppiPlanner::Control MppiPlanner::boundedControl(
  const Control & desired,
  const Control & previous) const
{
  Control bounded;
  bounded.speed = clampValue(desired.speed, min_speed_, max_speed_);
  bounded.steering = clampValue(desired.steering, -max_steering_angle_, max_steering_angle_);

  if (max_accel_ > kEpsilon) {
    const double max_delta = max_accel_ * time_step_;
    bounded.speed = clampValue(bounded.speed, previous.speed - max_delta, previous.speed + max_delta);
  }

  if (max_steering_rate_ > kEpsilon) {
    const double max_delta = max_steering_rate_ * time_step_;
    bounded.steering = clampValue(
      bounded.steering, previous.steering - max_delta, previous.steering + max_delta);
  }

  bounded.speed = clampValue(bounded.speed, min_speed_, max_speed_);
  bounded.steering = clampValue(bounded.steering, -max_steering_angle_, max_steering_angle_);
  return bounded;
}

MppiPlanner::State MppiPlanner::simulateStep(const State & state, const Control & control) const
{
  State next = state;
  next.x += control.speed * std::cos(state.yaw) * time_step_;
  next.y += control.speed * std::sin(state.yaw) * time_step_;
  next.yaw = normalizeAngle(
    state.yaw + control.speed / wheel_base_ * std::tan(control.steering) * time_step_);
  return next;
}

void MppiPlanner::ensureControlSequence()
{
  if (nominal_controls_.size() == static_cast<size_t>(horizon_steps_)) {
    return;
  }

  resetControlSequence();
}

void MppiPlanner::resetControlSequence()
{
  nominal_controls_.assign(
    static_cast<size_t>(std::max(2, horizon_steps_)),
    Control{target_speed_, 0.0});
  has_previous_command_ = false;
  previous_command_ = Control{target_speed_, 0.0};
}

void MppiPlanner::shiftControlSequence()
{
  if (nominal_controls_.empty()) {
    return;
  }

  for (size_t i = 0; i + 1 < nominal_controls_.size(); ++i) {
    nominal_controls_[i] = nominal_controls_[i + 1];
  }

  nominal_controls_.back() = Control{target_speed_, 0.0};
}

nav_msgs::msg::Path MppiPlanner::statesToPath(
  const std::vector<State> & states,
  const std::string & frame_id) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = frame_id;
  path.poses.reserve(states.size());

  for (const auto & state : states) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.position.z = marker_height_;
    pose.pose.orientation = yawToQuaternion(state.yaw);
    path.poses.push_back(pose);
  }

  return path;
}

void MppiPlanner::publishStop(const std::string & frame_id)
{
  ackermann_msgs::msg::AckermannDriveStamped cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = frame_id;
  cmd.drive.speed = 0.0f;
  cmd.drive.steering_angle = 0.0f;
  pub_command_->publish(cmd);

  nav_msgs::msg::Path empty_path;
  empty_path.header.stamp = now();
  empty_path.header.frame_id = frame_id;
  pub_trajectory_->publish(empty_path);

  if (visualize_markers_) {
    clearMarkers(frame_id);
  }
}

void MppiPlanner::publishMarkers(const nav_msgs::msg::Path & trajectory_path) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header = trajectory_path.header;
  clear_marker.ns = "mppi";
  clear_marker.id = -1;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  double best_rollout_cost = std::numeric_limits<double>::max();
  double worst_rollout_cost = -std::numeric_limits<double>::max();
  for (const auto & rollout : sampled_rollouts_) {
    if (!std::isfinite(rollout.cost)) {
      continue;
    }
    best_rollout_cost = std::min(best_rollout_cost, rollout.cost);
    worst_rollout_cost = std::max(worst_rollout_cost, rollout.cost);
  }

  const double cost_span = std::max(kEpsilon, worst_rollout_cost - best_rollout_cost);
  for (size_t i = 0; i < sampled_rollouts_.size(); ++i) {
    const auto & rollout = sampled_rollouts_[i];
    if (rollout.states.empty()) {
      continue;
    }

    const double normalized_cost =
      std::isfinite(rollout.cost) ? clampValue((rollout.cost - best_rollout_cost) / cost_span, 0.0, 1.0) : 1.0;

    visualization_msgs::msg::Marker rollout_marker;
    rollout_marker.header = trajectory_path.header;
    rollout_marker.ns = "mppi_rollouts";
    rollout_marker.id = static_cast<int>(i);
    rollout_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    rollout_marker.action = visualization_msgs::msg::Marker::ADD;
    rollout_marker.pose.orientation.w = 1.0;
    rollout_marker.scale.x = rollout_line_width_;
    rollout_marker.color.r = 0.25 + 0.55 * normalized_cost;
    rollout_marker.color.g = 0.55 * (1.0 - normalized_cost);
    rollout_marker.color.b = 1.0;
    rollout_marker.color.a = rollout_alpha_;

    for (const auto & state : rollout.states) {
      geometry_msgs::msg::Point point;
      point.x = state.x;
      point.y = state.y;
      point.z = marker_height_;
      rollout_marker.points.push_back(point);
    }

    marker_array.markers.push_back(rollout_marker);
  }

  if (has_best_sampled_rollout_ && !best_sampled_rollout_.states.empty()) {
    visualization_msgs::msg::Marker best_marker;
    best_marker.header = trajectory_path.header;
    best_marker.ns = "mppi_best_sampled";
    best_marker.id = 0;
    best_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    best_marker.action = visualization_msgs::msg::Marker::ADD;
    best_marker.pose.orientation.w = 1.0;
    best_marker.scale.x = std::max(marker_line_width_ * 0.8, rollout_line_width_ * 1.8);
    best_marker.color.r = 1.0;
    best_marker.color.g = 0.88;
    best_marker.color.b = 0.05;
    best_marker.color.a = 0.95;

    for (const auto & state : best_sampled_rollout_.states) {
      geometry_msgs::msg::Point point;
      point.x = state.x;
      point.y = state.y;
      point.z = marker_height_ + 0.02;
      best_marker.points.push_back(point);
    }

    marker_array.markers.push_back(best_marker);
  }

  visualization_msgs::msg::Marker line_marker;
  line_marker.header = trajectory_path.header;
  line_marker.ns = "mppi_optimal";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.scale.x = marker_line_width_;
  line_marker.color.r = 0.0;
  line_marker.color.g = 0.85;
  line_marker.color.b = 0.35;
  line_marker.color.a = 0.95;

  for (const auto & pose : trajectory_path.poses) {
    geometry_msgs::msg::Point point;
    point.x = pose.pose.position.x;
    point.y = pose.pose.position.y;
    point.z = marker_height_;
    line_marker.points.push_back(point);
  }

  marker_array.markers.push_back(line_marker);

  if (has_path_diagnostics_) {
    visualization_msgs::msg::Marker projection_marker;
    projection_marker.header = trajectory_path.header;
    projection_marker.ns = "mppi_diagnostics";
    projection_marker.id = 0;
    projection_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    projection_marker.action = visualization_msgs::msg::Marker::ADD;
    projection_marker.pose.orientation.w = 1.0;
    projection_marker.scale.x = std::max(rollout_line_width_, 0.015);
    projection_marker.color.r = 1.0;
    projection_marker.color.g = 1.0;
    projection_marker.color.b = 1.0;
    projection_marker.color.a = 0.75;

    geometry_msgs::msg::Point origin;
    origin.z = marker_height_ + 0.02;
    geometry_msgs::msg::Point projection = last_projection_point_;
    projection.z = marker_height_ + 0.02;
    projection_marker.points.push_back(origin);
    projection_marker.points.push_back(projection);
    marker_array.markers.push_back(projection_marker);

    auto make_sphere = [this, &trajectory_path](
        const int id,
        const geometry_msgs::msg::Point & point,
        const double red,
        const double green,
        const double blue) {
        visualization_msgs::msg::Marker marker;
        marker.header = trajectory_path.header;
        marker.ns = "mppi_diagnostics";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = point;
        marker.pose.position.z = marker_height_ + 0.06;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.16;
        marker.scale.y = 0.16;
        marker.scale.z = 0.16;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = 0.9;
        return marker;
      };

    marker_array.markers.push_back(
      make_sphere(1, last_projection_point_, 1.0, 1.0, 1.0));
    if (has_heading_lookahead_point_) {
      marker_array.markers.push_back(
        make_sphere(2, last_heading_lookahead_point_, 1.0, 0.85, 0.05));
    }
    if (has_curvature_lookahead_point_) {
      marker_array.markers.push_back(
        make_sphere(3, last_curvature_lookahead_point_, 1.0, 0.35, 0.05));
    }

    visualization_msgs::msg::Marker text_marker;
    text_marker.header = trajectory_path.header;
    text_marker.ns = "mppi_diagnostics";
    text_marker.id = 4;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = 0.7;
    text_marker.pose.position.y = 0.0;
    text_marker.pose.position.z = marker_height_ + 0.45;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.22;
    const bool speed_adjusted = last_command_speed_limited_ || last_command_speed_floor_applied_;
    text_marker.color.r = speed_adjusted ? 1.0 : 0.75;
    text_marker.color.g = speed_adjusted ? 0.45 : 1.0;
    text_marker.color.b = 0.15;
    text_marker.color.a = 0.95;

    std::ostringstream text;
    text << std::fixed << std::setprecision(2)
         << "corner " << last_curvature_heading_change_ * 180.0 / kPi << " deg"
         << "  target " << last_corner_speed_target_ << " m/s"
         << "  lat " << last_lateral_accel_speed_target_
         << "  cost_target " << last_effective_speed_target_
         << "  cmd " << last_command_speed_before_limit_;
    if (last_command_speed_limited_) {
      text << " -> limited";
    }
    if (last_command_speed_floor_applied_) {
      text << " -> floor " << last_tracking_speed_floor_;
    }
    text << "\nsteer target " << last_steering_target_ * 180.0 / kPi
         << " deg  cmd " << last_command_steering_ * 180.0 / kPi
         << " deg  lookahead " << last_curvature_lookahead_actual_distance_ << " m";
    text_marker.text = text.str();
    marker_array.markers.push_back(text_marker);
  }

  pub_markers_->publish(marker_array);
}

void MppiPlanner::clearMarkers(const std::string & frame_id) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = now();
  marker.header.frame_id = frame_id;
  marker.ns = "mppi";
  marker.id = -1;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
  pub_markers_->publish(marker_array);
}

void MppiPlanner::publishDebugStatus(
  const ackermann_msgs::msg::AckermannDriveStamped & cmd,
  const nav_msgs::msg::Path & trajectory_path) const
{
  std_msgs::msg::String msg;
  std::ostringstream stream;
  const double curvature_estimate =
    last_curvature_lookahead_actual_distance_ > kEpsilon ?
    last_curvature_heading_change_ / last_curvature_lookahead_actual_distance_ : 0.0;
  const double command_lateral_accel_estimate =
    cmd.drive.speed * cmd.drive.speed * curvature_estimate;

  stream << std::fixed << std::setprecision(3)
         << "{"
         << "\"tuning_profile\":\"" << escapeJsonString(tuning_profile_) << "\","
         << "\"speed\":" << cmd.drive.speed << ","
         << "\"steering_deg\":" << cmd.drive.steering_angle * 180.0 / kPi << ","
         << "\"selected_cost\":" << last_selected_cost_ << ","
         << "\"best_sampled_rollout_cost\":" << last_best_sampled_rollout_cost_ << ","
         << "\"selected_minus_best_sampled_cost\":"
         << last_selected_cost_ - last_best_sampled_rollout_cost_ << ","
         << "\"best_rollout_cost\":" << last_best_rollout_cost_ << ","
         << "\"worst_rollout_cost\":" << last_worst_rollout_cost_ << ","
         << "\"rollouts\":" << sampled_rollouts_.size() << ","
         << "\"trajectory_points\":" << trajectory_path.poses.size() << ","
         << "\"target_speed\":" << target_speed_ << ","
         << "\"max_speed\":" << max_speed_ << ","
         << "\"max_accel\":" << max_accel_ << ","
         << "\"max_steering_rate\":" << max_steering_rate_ << ","
         << "\"iteration_count\":" << iteration_count_ << ","
         << "\"temperature\":" << temperature_ << ","
         << "\"path_distance\":" << last_path_distance_ << ","
         << "\"path_progress\":" << last_path_progress_ << ","
         << "\"path_heading_deg\":" << last_path_heading_ * 180.0 / kPi << ","
         << "\"lookahead_heading_deg\":"
         << last_heading_lookahead_error_ * 180.0 / kPi << ","
         << "\"heading_lookahead_actual_distance\":"
         << last_heading_lookahead_actual_distance_ << ","
         << "\"curvature_heading_change_deg\":"
         << last_curvature_heading_change_ * 180.0 / kPi << ","
         << "\"curvature_heading_change_signed_deg\":"
         << last_curvature_heading_change_signed_ * 180.0 / kPi << ","
         << "\"curvature_lookahead_actual_distance\":"
         << last_curvature_lookahead_actual_distance_ << ","
         << "\"curvature_estimate\":" << curvature_estimate << ","
         << "\"command_lateral_accel_estimate\":"
         << command_lateral_accel_estimate << ","
         << "\"steering_target_deg\":" << last_steering_target_ * 180.0 / kPi << ","
         << "\"steering_error_deg\":"
         << (last_steering_target_ - cmd.drive.steering_angle) * 180.0 / kPi << ","
         << "\"corner_speed_target\":" << last_corner_speed_target_ << ","
         << "\"lateral_accel_speed_target\":" << last_lateral_accel_speed_target_ << ","
         << "\"lateral_accel_limit\":" << lateral_accel_limit_ << ","
         << "\"use_lateral_accel_speed_limit\":"
         << (use_lateral_accel_speed_limit_ ? "true" : "false") << ","
         << "\"effective_speed_target\":" << last_effective_speed_target_ << ","
         << "\"tracking_min_command_speed\":" << tracking_min_command_speed_ << ","
         << "\"tracking_speed_floor\":" << last_tracking_speed_floor_ << ","
         << "\"use_corner_speed_as_speed_target\":"
         << (use_corner_speed_as_speed_target_ ? "true" : "false") << ","
         << "\"command_speed_before_limit\":" << last_command_speed_before_limit_ << ","
         << "\"command_speed_limited\":"
         << (last_command_speed_limited_ ? "true" : "false") << ","
         << "\"command_speed_floor_applied\":"
         << (last_command_speed_floor_applied_ ? "true" : "false") << ","
         << "\"limit_command_speed_to_corner\":"
         << (limit_command_speed_to_corner_ ? "true" : "false") << ","
         << "\"corner_speed_margin\":" << corner_speed_margin_ << ","
         << "\"heading_weight\":" << heading_weight_ << ","
         << "\"heading_lookahead_weight\":" << heading_lookahead_weight_ << ","
         << "\"heading_lookahead_distance\":" << heading_lookahead_distance_ << ","
         << "\"steering_rate_weight\":" << steering_rate_weight_ << ","
         << "\"accel_weight\":" << accel_weight_ << ","
         << "\"steering_feedforward_lookahead_distance\":"
         << steering_feedforward_lookahead_distance_ << ","
         << "\"steering_feedforward_weight\":" << steering_feedforward_weight_ << ","
         << "\"steering_feedforward_command_gain\":"
         << steering_feedforward_command_gain_ << ","
         << "\"curvature_speed_weight\":" << curvature_speed_weight_ << ","
         << "\"curvature_speed_gain\":" << curvature_speed_gain_ << ","
         << "\"curvature_lookahead_distance\":" << curvature_lookahead_distance_
         << "}";
  msg.data = stream.str();
  pub_debug_status_->publish(msg);
}

double MppiPlanner::normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::msg::Quaternion MppiPlanner::yawToQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
