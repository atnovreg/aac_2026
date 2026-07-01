#include "pure_pursuit/pure_pursuit.h"

#include <cmath>
#include <functional>
#include <limits>

PurePursuit::PurePursuit(const std::string & node_name)
: Node(node_name)
{
  getParams();

  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&PurePursuit::onParamsChanged, this, std::placeholders::_1));

  sub_local_path_ = create_subscription<nav_msgs::msg::Path>(
    local_path_topic_, rclcpp::QoS(10),
    std::bind(&PurePursuit::localPathCallback, this, std::placeholders::_1));
  sub_global_path_ = create_subscription<nav_msgs::msg::Path>(
    global_path_topic_, rclcpp::QoS(10),
    std::bind(&PurePursuit::globalPathCallback, this, std::placeholders::_1));

  pub_command_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    command_topic_, rclcpp::QoS(10));
  pub_target_marker_ = create_publisher<visualization_msgs::msg::Marker>(
    target_marker_topic_, rclcpp::QoS(10));
  pub_curvature_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    curvature_marker_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(
    get_logger(),
    "PurePursuit ready — callback-driven path=%s, cmd=%s, lookahead=%.2f m, max_speed=%.2f m/s",
    local_path_topic_.c_str(), command_topic_.c_str(), lookahead_distance_, max_speed_);
}

void PurePursuit::getParams()
{
  declare_parameter<double>("wheel_base", 0.80);
  declare_parameter<double>("lookahead_distance", 1.0);
  declare_parameter<double>("max_steering_angle", 0.524);
  declare_parameter<double>("max_speed", 1.5);
  declare_parameter<double>("min_speed", 0.0);
  declare_parameter<double>("lateral_accel_limit", 0.8);
  declare_parameter<double>("curvature_epsilon", 1.0e-4);
  declare_parameter<double>("curvature_preview_distance", 4.0);
  declare_parameter<double>("target_marker_scale", 0.18);
  declare_parameter<double>("curvature_marker_scale", 0.10);
  declare_parameter<double>("curvature_marker_height", 0.06);
  declare_parameter<double>("max_visualized_curvature", 2.0);
  declare_parameter<double>("min_forward_x", 0.05);
  declare_parameter<int>("curvature_preview_points", 30);
  declare_parameter<bool>("visualize_markers", true);
  declare_parameter<std::string>("local_path_topic", "/local_path/body");
  declare_parameter<std::string>("global_path_topic", "/local_path/global");
  declare_parameter<std::string>("command_topic", "/ackermann_auto");
  declare_parameter<std::string>("command_frame", "base_footprint");
  declare_parameter<std::string>("target_marker_topic", "/pure_pursuit/target_point");
  declare_parameter<std::string>("curvature_marker_topic", "/pure_pursuit/curvature_markers");

  wheel_base_ = get_parameter("wheel_base").as_double();
  lookahead_distance_ = get_parameter("lookahead_distance").as_double();
  max_steering_angle_ = get_parameter("max_steering_angle").as_double();
  max_speed_ = get_parameter("max_speed").as_double();
  min_speed_ = get_parameter("min_speed").as_double();
  lateral_accel_limit_ = get_parameter("lateral_accel_limit").as_double();
  curvature_epsilon_ = get_parameter("curvature_epsilon").as_double();
  curvature_preview_distance_ = get_parameter("curvature_preview_distance").as_double();
  target_marker_scale_ = get_parameter("target_marker_scale").as_double();
  curvature_marker_scale_ = get_parameter("curvature_marker_scale").as_double();
  curvature_marker_height_ = get_parameter("curvature_marker_height").as_double();
  max_visualized_curvature_ = get_parameter("max_visualized_curvature").as_double();
  min_forward_x_ = get_parameter("min_forward_x").as_double();
  curvature_preview_points_ = get_parameter("curvature_preview_points").as_int();
  visualize_markers_ = get_parameter("visualize_markers").as_bool();
  local_path_topic_ = get_parameter("local_path_topic").as_string();
  global_path_topic_ = get_parameter("global_path_topic").as_string();
  command_topic_ = get_parameter("command_topic").as_string();
  command_frame_ = get_parameter("command_frame").as_string();
  target_marker_topic_ = get_parameter("target_marker_topic").as_string();
  curvature_marker_topic_ = get_parameter("curvature_marker_topic").as_string();

  max_speed_ = std::max(0.0, max_speed_);
  min_speed_ = clampValue(min_speed_, 0.0, max_speed_);
  lateral_accel_limit_ = std::max(0.0, lateral_accel_limit_);
  curvature_epsilon_ = std::max(1.0e-9, curvature_epsilon_);
  target_marker_scale_ = std::max(0.01, target_marker_scale_);
  curvature_marker_scale_ = std::max(0.01, curvature_marker_scale_);
  max_visualized_curvature_ = std::max(curvature_epsilon_, max_visualized_curvature_);
  curvature_preview_points_ = std::max(3, curvature_preview_points_);
}

rcl_interfaces::msg::SetParametersResult PurePursuit::onParamsChanged(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  double wheel_base = wheel_base_;
  double lookahead_distance = lookahead_distance_;
  double max_steering_angle = max_steering_angle_;
  double max_speed = max_speed_;
  double min_speed = min_speed_;
  double lateral_accel_limit = lateral_accel_limit_;
  double curvature_epsilon = curvature_epsilon_;
  double curvature_preview_distance = curvature_preview_distance_;
  double target_marker_scale = target_marker_scale_;
  double curvature_marker_scale = curvature_marker_scale_;
  double curvature_marker_height = curvature_marker_height_;
  double max_visualized_curvature = max_visualized_curvature_;
  double min_forward_x = min_forward_x_;
  int curvature_preview_points = curvature_preview_points_;
  bool visualize_markers = visualize_markers_;
  std::string command_frame = command_frame_;

  auto reject = [&result](const std::string & reason) {
      result.successful = false;
      result.reason = reason;
      return result;
    };

  auto read_double = [&reject](const rclcpp::Parameter & parameter, double & value) {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        reject(parameter.get_name() + " must be a double");
        return false;
      }
      value = parameter.as_double();
      return true;
    };

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();

    if (name == "wheel_base") {
      if (!read_double(parameter, wheel_base)) {
        return result;
      }
      if (wheel_base <= 0.0) {
        return reject("wheel_base must be greater than 0");
      }
    } else if (name == "lookahead_distance") {
      if (!read_double(parameter, lookahead_distance)) {
        return result;
      }
      if (lookahead_distance <= 0.0) {
        return reject("lookahead_distance must be greater than 0");
      }
    } else if (name == "max_steering_angle") {
      if (!read_double(parameter, max_steering_angle)) {
        return result;
      }
      if (max_steering_angle <= 0.0) {
        return reject("max_steering_angle must be greater than 0");
      }
    } else if (name == "max_speed") {
      if (!read_double(parameter, max_speed)) {
        return result;
      }
      if (max_speed < 0.0) {
        return reject("max_speed must be greater than or equal to 0");
      }
    } else if (name == "min_speed") {
      if (!read_double(parameter, min_speed)) {
        return result;
      }
      if (min_speed < 0.0) {
        return reject("min_speed must be greater than or equal to 0");
      }
    } else if (name == "lateral_accel_limit") {
      if (!read_double(parameter, lateral_accel_limit)) {
        return result;
      }
      if (lateral_accel_limit < 0.0) {
        return reject("lateral_accel_limit must be greater than or equal to 0");
      }
    } else if (name == "curvature_epsilon") {
      if (!read_double(parameter, curvature_epsilon)) {
        return result;
      }
      if (curvature_epsilon <= 0.0) {
        return reject("curvature_epsilon must be greater than 0");
      }
    } else if (name == "curvature_preview_distance") {
      if (!read_double(parameter, curvature_preview_distance)) {
        return result;
      }
      if (curvature_preview_distance < 0.0) {
        return reject("curvature_preview_distance must be greater than or equal to 0");
      }
    } else if (name == "target_marker_scale") {
      if (!read_double(parameter, target_marker_scale)) {
        return result;
      }
      if (target_marker_scale <= 0.0) {
        return reject("target_marker_scale must be greater than 0");
      }
    } else if (name == "curvature_marker_scale") {
      if (!read_double(parameter, curvature_marker_scale)) {
        return result;
      }
      if (curvature_marker_scale <= 0.0) {
        return reject("curvature_marker_scale must be greater than 0");
      }
    } else if (name == "curvature_marker_height") {
      if (!read_double(parameter, curvature_marker_height)) {
        return result;
      }
    } else if (name == "max_visualized_curvature") {
      if (!read_double(parameter, max_visualized_curvature)) {
        return result;
      }
      if (max_visualized_curvature <= 0.0) {
        return reject("max_visualized_curvature must be greater than 0");
      }
    } else if (name == "min_forward_x") {
      if (!read_double(parameter, min_forward_x)) {
        return result;
      }
      if (min_forward_x < 0.0) {
        return reject("min_forward_x must be greater than or equal to 0");
      }
    } else if (name == "curvature_preview_points") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        return reject("curvature_preview_points must be an integer");
      }
      curvature_preview_points = static_cast<int>(parameter.as_int());
      if (curvature_preview_points < 3) {
        return reject("curvature_preview_points must be at least 3");
      }
    } else if (name == "visualize_markers") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        return reject("visualize_markers must be a bool");
      }
      visualize_markers = parameter.as_bool();
    } else if (name == "command_frame") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("command_frame must be a string");
      }
      command_frame = parameter.as_string();
      if (command_frame.empty()) {
        return reject("command_frame must not be empty");
      }
    } else if (
      name == "local_path_topic" ||
      name == "global_path_topic" ||
      name == "command_topic" ||
      name == "target_marker_topic" ||
      name == "curvature_marker_topic")
    {
      return reject(name + " requires node restart");
    }
  }

  if (min_speed > max_speed) {
    return reject("min_speed must be less than or equal to max_speed");
  }

  wheel_base_ = wheel_base;
  lookahead_distance_ = lookahead_distance;
  max_steering_angle_ = max_steering_angle;
  max_speed_ = max_speed;
  min_speed_ = min_speed;
  lateral_accel_limit_ = lateral_accel_limit;
  curvature_epsilon_ = curvature_epsilon;
  curvature_preview_distance_ = curvature_preview_distance;
  target_marker_scale_ = target_marker_scale;
  curvature_marker_scale_ = curvature_marker_scale;
  curvature_marker_height_ = curvature_marker_height;
  max_visualized_curvature_ = std::max(curvature_epsilon_, max_visualized_curvature);
  min_forward_x_ = min_forward_x;
  curvature_preview_points_ = curvature_preview_points;
  visualize_markers_ = visualize_markers;
  command_frame_ = command_frame;

  RCLCPP_INFO(
    get_logger(),
    "Updated pure pursuit params: lookahead=%.2f m, max_speed=%.2f m/s, lateral_accel=%.2f m/s^2",
    lookahead_distance_, max_speed_, lateral_accel_limit_);

  return result;
}

void PurePursuit::localPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  local_path_ = *msg;
  const std::string frame_id =
    !local_path_.header.frame_id.empty() ? local_path_.header.frame_id : command_frame_;

  if (local_path_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Received empty local body path on '%s'", local_path_topic_.c_str());
    publishStop(frame_id);
    return;
  }

  ackermann_msgs::msg::AckermannDriveStamped cmd;
  if (!makeCommand(cmd)) {
    publishStop(frame_id);
    return;
  }

  pub_command_->publish(cmd);
}

void PurePursuit::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  global_path_ = *msg;
  has_global_path_ = !global_path_.poses.empty();

  if (!visualize_markers_ || local_path_.poses.empty()) {
    return;
  }

  geometry_msgs::msg::Point target;
  size_t target_index = 0;
  if (!selectTargetPoint(target, target_index)) {
    return;
  }

  const std::string fallback_frame =
    !local_path_.header.frame_id.empty() ? local_path_.header.frame_id : command_frame_;
  publishTargetMarker(target, target_index, fallback_frame);
  publishCurvatureMarkers(fallback_frame);
}

bool PurePursuit::makeCommand(ackermann_msgs::msg::AckermannDriveStamped & cmd)
{
  geometry_msgs::msg::Point target;
  size_t target_index = 0;
  if (!selectTargetPoint(target, target_index)) {
    return false;
  }

  const std::string frame_id =
    !local_path_.header.frame_id.empty() ? local_path_.header.frame_id : command_frame_;

  cmd.header.stamp = now();
  cmd.header.frame_id = frame_id;
  cmd.drive.steering_angle = static_cast<float>(computeSteeringAngle(target));
  cmd.drive.speed = static_cast<float>(computeCurvatureLimitedSpeed());

  if (visualize_markers_) {
    publishTargetMarker(target, target_index, frame_id);
    publishCurvatureMarkers(frame_id);
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "target=(%.2f, %.2f), steer=%.1f deg, speed=%.2f m/s",
    target.x, target.y,
    static_cast<double>(cmd.drive.steering_angle) * 180.0 / M_PI,
    static_cast<double>(cmd.drive.speed));

  return true;
}

bool PurePursuit::selectTargetPoint(geometry_msgs::msg::Point & target, size_t & target_index) const
{
  bool has_forward_point = false;
  geometry_msgs::msg::Point last_forward_point;
  size_t last_forward_index = 0;

  for (size_t i = 0; i < local_path_.poses.size(); ++i) {
    const auto & pose = local_path_.poses[i];
    const auto & point = pose.pose.position;
    if (point.x < min_forward_x_) {
      continue;
    }

    has_forward_point = true;
    last_forward_point = point;
    last_forward_index = i;

    const double dist = std::hypot(point.x, point.y);
    if (dist >= lookahead_distance_) {
      target = point;
      target_index = i;
      return true;
    }
  }

  if (has_forward_point) {
    target = last_forward_point;
    target_index = last_forward_index;
    return true;
  }

  return false;
}

double PurePursuit::computeSteeringAngle(const geometry_msgs::msg::Point & target) const
{
  const double lookahead_sq = target.x * target.x + target.y * target.y;
  if (lookahead_sq <= 1.0e-9) {
    return 0.0;
  }

  const double curvature = 2.0 * target.y / lookahead_sq;
  const double steering = std::atan(wheel_base_ * curvature);
  return clampValue(steering, -max_steering_angle_, max_steering_angle_);
}

double PurePursuit::computeCurvatureLimitedSpeed() const
{
  if (local_path_.poses.size() < 3 || lateral_accel_limit_ <= 0.0) {
    return max_speed_;
  }

  double max_curvature = 0.0;
  for (const auto & sample : collectCurvatureSamples()) {
    max_curvature = std::max(max_curvature, sample.curvature);
  }

  if (max_curvature <= curvature_epsilon_) {
    return max_speed_;
  }

  const double curve_speed = std::sqrt(lateral_accel_limit_ / max_curvature);
  return clampValue(curve_speed, min_speed_, max_speed_);
}

std::vector<PurePursuit::CurvatureSample> PurePursuit::collectCurvatureSamples() const
{
  std::vector<CurvatureSample> samples;
  if (local_path_.poses.size() < 3) {
    return samples;
  }

  double accumulated_dist = 0.0;
  int used_points = 0;

  for (size_t i = 1; i + 1 < local_path_.poses.size(); ++i) {
    accumulated_dist += poseDistance(local_path_.poses[i - 1], local_path_.poses[i]);
    if (accumulated_dist > curvature_preview_distance_) {
      break;
    }

    const double x = local_path_.poses[i].pose.position.x;
    if (x < min_forward_x_) {
      continue;
    }

    CurvatureSample sample;
    sample.point = local_path_.poses[i].pose.position;
    sample.point.z = curvature_marker_height_;
    sample.curvature =
      pointCurvature(local_path_.poses[i - 1], local_path_.poses[i], local_path_.poses[i + 1]);
    sample.index = i;
    samples.push_back(sample);

    ++used_points;
    if (used_points >= curvature_preview_points_) {
      break;
    }
  }

  return samples;
}

bool PurePursuit::canUseGlobalVisualization() const
{
  return has_global_path_ &&
    !global_path_.poses.empty() &&
    !global_path_.header.frame_id.empty();
}

bool PurePursuit::getGlobalVisualizationPoint(
  size_t index,
  geometry_msgs::msg::Point & point) const
{
  if (!canUseGlobalVisualization() || index >= global_path_.poses.size()) {
    return false;
  }

  point = global_path_.poses[index].pose.position;
  return true;
}

std::string PurePursuit::visualizationFrameId(const std::string & fallback_frame) const
{
  if (canUseGlobalVisualization()) {
    return global_path_.header.frame_id;
  }
  return fallback_frame;
}

double PurePursuit::pointCurvature(
  const geometry_msgs::msg::PoseStamped & prev,
  const geometry_msgs::msg::PoseStamped & current,
  const geometry_msgs::msg::PoseStamped & next) const
{
  const double x1 = prev.pose.position.x;
  const double y1 = prev.pose.position.y;
  const double x2 = current.pose.position.x;
  const double y2 = current.pose.position.y;
  const double x3 = next.pose.position.x;
  const double y3 = next.pose.position.y;

  const double s12 = std::hypot(x1 - x2, y1 - y2);
  const double s23 = std::hypot(x2 - x3, y2 - y3);
  const double s13 = std::hypot(x1 - x3, y1 - y3);

  if (s12 <= 1.0e-9 || s23 <= 1.0e-9 || s13 <= 1.0e-9) {
    return 0.0;
  }

  const double xd = (x3 - x1) / s13;
  const double xd12 = (x2 - x1) / s12;
  const double xd23 = (x3 - x2) / s23;
  const double xdd = (xd23 - xd12) / s13;

  const double yd = (y3 - y1) / s13;
  const double yd12 = (y2 - y1) / s12;
  const double yd23 = (y3 - y2) / s23;
  const double ydd = (yd23 - yd12) / s13;

  const double denom_base = xd * xd + yd * yd;
  if (denom_base <= 1.0e-12) {
    return 0.0;
  }

  return std::fabs(xd * ydd - yd * xdd) / std::pow(std::sqrt(denom_base), 3.0);
}

double PurePursuit::poseDistance(
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & to) const
{
  const double dx = to.pose.position.x - from.pose.position.x;
  const double dy = to.pose.position.y - from.pose.position.y;
  return std::hypot(dx, dy);
}

void PurePursuit::publishStop(const std::string & frame_id)
{
  ackermann_msgs::msg::AckermannDriveStamped cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = frame_id;
  cmd.drive.speed = 0.0f;
  cmd.drive.steering_angle = 0.0f;
  pub_command_->publish(cmd);
  if (visualize_markers_) {
    clearVisualization(frame_id);
  }
}

void PurePursuit::publishTargetMarker(
  const geometry_msgs::msg::Point & target,
  size_t target_index,
  const std::string & frame_id) const
{
  geometry_msgs::msg::Point marker_point = target;
  std::string marker_frame = frame_id;
  if (getGlobalVisualizationPoint(target_index, marker_point)) {
    marker_frame = visualizationFrameId(frame_id);
  }

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = now();
  marker.header.frame_id = marker_frame;
  marker.ns = "pure_pursuit_target";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = marker_point;
  marker.pose.position.z = target_marker_scale_ * 0.5;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = target_marker_scale_;
  marker.scale.y = target_marker_scale_;
  marker.scale.z = target_marker_scale_;
  marker.color.r = 1.0;
  marker.color.g = 0.9;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  pub_target_marker_->publish(marker);
}

void PurePursuit::publishCurvatureMarkers(const std::string & frame_id) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  const bool use_global_frame = canUseGlobalVisualization();
  const std::string marker_frame = use_global_frame ? visualizationFrameId(frame_id) : frame_id;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.stamp = now();
  clear_marker.header.frame_id = marker_frame;
  clear_marker.ns = "pure_pursuit_curvature";
  clear_marker.id = -1;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  int id = 0;
  for (const auto & sample : collectCurvatureSamples()) {
    geometry_msgs::msg::Point marker_point = sample.point;
    if (use_global_frame && !getGlobalVisualizationPoint(sample.index, marker_point)) {
      continue;
    }
    marker_point.z = curvature_marker_height_;

    const double ratio = clampValue(
      std::fabs(sample.curvature) / max_visualized_curvature_, 0.0, 1.0);
    const double scale = curvature_marker_scale_ * (0.8 + 1.5 * ratio);

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = marker_frame;
    marker.ns = "pure_pursuit_curvature";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = marker_point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = ratio;
    marker.color.g = 1.0 - ratio;
    marker.color.b = 0.1;
    marker.color.a = 0.9;
    marker_array.markers.push_back(marker);
  }

  pub_curvature_markers_->publish(marker_array);
}

void PurePursuit::clearVisualization(const std::string & frame_id) const
{
  visualization_msgs::msg::Marker target_marker;
  target_marker.header.stamp = now();
  target_marker.header.frame_id = frame_id;
  target_marker.ns = "pure_pursuit_target";
  target_marker.id = 0;
  target_marker.action = visualization_msgs::msg::Marker::DELETE;
  pub_target_marker_->publish(target_marker);

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.stamp = now();
  clear_marker.header.frame_id = frame_id;
  clear_marker.ns = "pure_pursuit_curvature";
  clear_marker.id = -1;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  pub_curvature_markers_->publish(marker_array);
}
