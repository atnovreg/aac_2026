#include "stanley/stanley.h"

#include <cmath>
#include <functional>
#include <limits>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;
}  // namespace

void Stanley::PidController::reset()
{
  integral = 0.0;
  previous_error = 0.0;
  has_previous = false;
}

double Stanley::PidController::update(const double error, const double dt)
{
  if (dt <= kEpsilon || !std::isfinite(dt)) {
    return kp * error;
  }

  integral += error * dt;
  if (integral_limit > 0.0) {
    integral = std::max(-integral_limit, std::min(integral, integral_limit));
  }

  double derivative = 0.0;
  if (has_previous) {
    derivative = (error - previous_error) / dt;
  }

  previous_error = error;
  has_previous = true;

  return kp * error + ki * integral + kd * derivative;
}

Stanley::Stanley(const std::string & node_name)
: Node(node_name)
{
  getParams();

  sub_local_path_ = create_subscription<nav_msgs::msg::Path>(
    local_path_topic_, rclcpp::QoS(10),
    std::bind(&Stanley::localPathCallback, this, std::placeholders::_1));

  pub_command_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    command_topic_, rclcpp::QoS(10));
  pub_error_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    error_marker_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(
    get_logger(),
    "Stanley ready: callback-driven path=%s, cmd=%s, target_speed=%.2f m/s",
    local_path_topic_.c_str(), command_topic_.c_str(), target_speed_);
}

void Stanley::getParams()
{
  declare_parameter<std::string>("local_path_topic", "/local_path/body");
  declare_parameter<std::string>("command_topic", "/ackermann_auto");
  declare_parameter<std::string>("command_frame", "base_footprint");
  declare_parameter<std::string>("error_marker_topic", "/stanley/error_markers");

  declare_parameter<double>("stanley_gain", 1.0);
  declare_parameter<double>("softening_speed", 0.20);
  declare_parameter<double>("max_steering_angle", 0.524);

  declare_parameter<double>("target_speed", 1.0);
  declare_parameter<double>("max_speed", 1.0);
  declare_parameter<double>("min_speed", 0.0);
  declare_parameter<double>("cte_speed_reduction_gain", 0.35);
  declare_parameter<double>("heading_speed_reduction_gain", 0.40);

  declare_parameter<double>("cte_kp", 1.0);
  declare_parameter<double>("cte_ki", 0.0);
  declare_parameter<double>("cte_kd", 0.0);
  declare_parameter<double>("cte_integral_limit", 1.0);
  declare_parameter<double>("heading_kp", 1.0);
  declare_parameter<double>("heading_ki", 0.0);
  declare_parameter<double>("heading_kd", 0.0);
  declare_parameter<double>("heading_integral_limit", 0.6);

  declare_parameter<double>("default_dt", 0.02);
  declare_parameter<double>("max_dt", 0.20);
  declare_parameter<double>("min_segment_length", 0.02);
  declare_parameter<double>("min_segment_forward_x", -0.20);

  declare_parameter<bool>("visualize_markers", true);
  declare_parameter<double>("marker_scale", 0.12);
  declare_parameter<double>("marker_height", 0.05);

  local_path_topic_ = get_parameter("local_path_topic").as_string();
  command_topic_ = get_parameter("command_topic").as_string();
  command_frame_ = get_parameter("command_frame").as_string();
  error_marker_topic_ = get_parameter("error_marker_topic").as_string();

  stanley_gain_ = get_parameter("stanley_gain").as_double();
  softening_speed_ = get_parameter("softening_speed").as_double();
  max_steering_angle_ = get_parameter("max_steering_angle").as_double();

  target_speed_ = get_parameter("target_speed").as_double();
  max_speed_ = get_parameter("max_speed").as_double();
  min_speed_ = get_parameter("min_speed").as_double();
  cte_speed_reduction_gain_ = get_parameter("cte_speed_reduction_gain").as_double();
  heading_speed_reduction_gain_ = get_parameter("heading_speed_reduction_gain").as_double();

  cross_track_pid_.kp = get_parameter("cte_kp").as_double();
  cross_track_pid_.ki = get_parameter("cte_ki").as_double();
  cross_track_pid_.kd = get_parameter("cte_kd").as_double();
  cross_track_pid_.integral_limit = get_parameter("cte_integral_limit").as_double();

  heading_pid_.kp = get_parameter("heading_kp").as_double();
  heading_pid_.ki = get_parameter("heading_ki").as_double();
  heading_pid_.kd = get_parameter("heading_kd").as_double();
  heading_pid_.integral_limit = get_parameter("heading_integral_limit").as_double();

  default_dt_ = get_parameter("default_dt").as_double();
  max_dt_ = get_parameter("max_dt").as_double();
  min_segment_length_ = get_parameter("min_segment_length").as_double();
  min_segment_forward_x_ = get_parameter("min_segment_forward_x").as_double();

  visualize_markers_ = get_parameter("visualize_markers").as_bool();
  marker_scale_ = get_parameter("marker_scale").as_double();
  marker_height_ = get_parameter("marker_height").as_double();

  stanley_gain_ = std::max(0.0, stanley_gain_);
  softening_speed_ = std::max(0.0, softening_speed_);
  max_steering_angle_ = std::max(0.01, max_steering_angle_);
  max_speed_ = std::max(0.0, max_speed_);
  min_speed_ = clampValue(min_speed_, 0.0, max_speed_);
  target_speed_ = clampValue(target_speed_, min_speed_, max_speed_);
  cte_speed_reduction_gain_ = std::max(0.0, cte_speed_reduction_gain_);
  heading_speed_reduction_gain_ = std::max(0.0, heading_speed_reduction_gain_);
  cross_track_pid_.integral_limit = std::max(0.0, cross_track_pid_.integral_limit);
  heading_pid_.integral_limit = std::max(0.0, heading_pid_.integral_limit);
  default_dt_ = std::max(1.0e-3, default_dt_);
  max_dt_ = std::max(default_dt_, max_dt_);
  min_segment_length_ = std::max(1.0e-3, min_segment_length_);
  marker_scale_ = std::max(0.01, marker_scale_);
}

void Stanley::localPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  const std::string frame_id =
    !msg->header.frame_id.empty() ? msg->header.frame_id : command_frame_;

  if (msg->poses.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Received local path with fewer than 2 poses on '%s'", local_path_topic_.c_str());
    publishStop(frame_id);
    resetControlState();
    return;
  }

  ackermann_msgs::msg::AckermannDriveStamped cmd;
  TrackingError error;
  if (!makeCommand(*msg, cmd, error)) {
    publishStop(frame_id);
    resetControlState();
    return;
  }

  pub_command_->publish(cmd);

  if (visualize_markers_) {
    publishErrorMarkers(error, frame_id);
  }
}

bool Stanley::makeCommand(
  const nav_msgs::msg::Path & path,
  ackermann_msgs::msg::AckermannDriveStamped & cmd,
  TrackingError & error)
{
  if (!computeTrackingError(path, error)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No valid forward local path segment found on '%s'", local_path_topic_.c_str());
    return false;
  }

  const double dt = computeControlDt();
  const double speed = computeCommandSpeed(error);
  const double cte_control = cross_track_pid_.update(error.cross_track_error, dt);
  const double heading_control = heading_pid_.update(error.heading_error, dt);
  const double stanley_correction =
    std::atan2(stanley_gain_ * cte_control, std::fabs(speed) + softening_speed_);
  const double steering =
    clampValue(heading_control + stanley_correction, -max_steering_angle_, max_steering_angle_);

  const std::string frame_id =
    !path.header.frame_id.empty() ? path.header.frame_id : command_frame_;

  cmd.header.stamp = now();
  cmd.header.frame_id = frame_id;
  cmd.drive.steering_angle = static_cast<float>(steering);
  cmd.drive.speed = static_cast<float>(speed);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "cte=%.3f m, heading=%.1f deg, steer=%.1f deg, speed=%.2f m/s",
    error.cross_track_error,
    error.heading_error * 180.0 / kPi,
    steering * 180.0 / kPi,
    speed);

  return true;
}

bool Stanley::computeTrackingError(const nav_msgs::msg::Path & path, TrackingError & error) const
{
  const double min_segment_length_sq = min_segment_length_ * min_segment_length_;
  double best_distance_sq = std::numeric_limits<double>::max();
  bool found = false;

  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & p0 = path.poses[i].pose.position;
    const auto & p1 = path.poses[i + 1].pose.position;

    if (std::max(p0.x, p1.x) < min_segment_forward_x_) {
      continue;
    }

    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double segment_length_sq = dx * dx + dy * dy;
    if (segment_length_sq <= min_segment_length_sq) {
      continue;
    }

    const double t = clampValue(
      -(p0.x * dx + p0.y * dy) / segment_length_sq,
      0.0,
      1.0);

    geometry_msgs::msg::Point projection;
    projection.x = p0.x + t * dx;
    projection.y = p0.y + t * dy;
    projection.z = p0.z + t * (p1.z - p0.z);

    const double distance_sq = projection.x * projection.x + projection.y * projection.y;
    if (distance_sq >= best_distance_sq) {
      continue;
    }

    const double segment_length = std::sqrt(segment_length_sq);
    const double tangent_x = dx / segment_length;
    const double tangent_y = dy / segment_length;

    error.projection = projection;
    error.segment_start = p0;
    error.segment_end = p1;
    error.segment_index = i;
    error.distance = std::sqrt(distance_sq);
    error.path_yaw = std::atan2(dy, dx);
    error.heading_error = normalizeAngle(error.path_yaw);
    error.cross_track_error = tangent_x * projection.y - tangent_y * projection.x;

    best_distance_sq = distance_sq;
    found = true;
  }

  return found;
}

double Stanley::computeCommandSpeed(const TrackingError & error) const
{
  const double slowdown =
    1.0 +
    cte_speed_reduction_gain_ * std::fabs(error.cross_track_error) +
    heading_speed_reduction_gain_ * std::fabs(error.heading_error);

  const double speed = slowdown > kEpsilon ? target_speed_ / slowdown : target_speed_;
  return clampValue(speed, min_speed_, max_speed_);
}

double Stanley::computeControlDt()
{
  const rclcpp::Time current_time = now();
  double dt = default_dt_;

  if (has_last_control_time_) {
    dt = (current_time - last_control_time_).seconds();
    if (dt <= 0.0 || dt > max_dt_ || !std::isfinite(dt)) {
      dt = default_dt_;
    }
  }

  last_control_time_ = current_time;
  has_last_control_time_ = true;
  return dt;
}

void Stanley::resetControlState()
{
  cross_track_pid_.reset();
  heading_pid_.reset();
  has_last_control_time_ = false;
}

void Stanley::publishStop(const std::string & frame_id)
{
  ackermann_msgs::msg::AckermannDriveStamped cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = frame_id;
  cmd.drive.speed = 0.0f;
  cmd.drive.steering_angle = 0.0f;
  pub_command_->publish(cmd);

  if (visualize_markers_) {
    clearErrorMarkers(frame_id);
  }
}

void Stanley::publishErrorMarkers(const TrackingError & error, const std::string & frame_id) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.stamp = now();
  clear_marker.header.frame_id = frame_id;
  clear_marker.ns = "stanley_error";
  clear_marker.id = -1;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  visualization_msgs::msg::Marker segment_marker;
  segment_marker.header.stamp = now();
  segment_marker.header.frame_id = frame_id;
  segment_marker.ns = "stanley_error";
  segment_marker.id = 0;
  segment_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  segment_marker.action = visualization_msgs::msg::Marker::ADD;
  segment_marker.pose.orientation.w = 1.0;
  segment_marker.scale.x = marker_scale_ * 0.35;
  segment_marker.color.r = 0.1;
  segment_marker.color.g = 0.4;
  segment_marker.color.b = 1.0;
  segment_marker.color.a = 0.9;
  segment_marker.points.push_back(error.segment_start);
  segment_marker.points.push_back(error.segment_end);
  marker_array.markers.push_back(segment_marker);

  visualization_msgs::msg::Marker projection_marker;
  projection_marker.header.stamp = now();
  projection_marker.header.frame_id = frame_id;
  projection_marker.ns = "stanley_error";
  projection_marker.id = 1;
  projection_marker.type = visualization_msgs::msg::Marker::SPHERE;
  projection_marker.action = visualization_msgs::msg::Marker::ADD;
  projection_marker.pose.position = error.projection;
  projection_marker.pose.position.z = marker_height_;
  projection_marker.pose.orientation.w = 1.0;
  projection_marker.scale.x = marker_scale_;
  projection_marker.scale.y = marker_scale_;
  projection_marker.scale.z = marker_scale_;
  projection_marker.color.r = 0.1;
  projection_marker.color.g = 1.0;
  projection_marker.color.b = 0.3;
  projection_marker.color.a = 0.95;
  marker_array.markers.push_back(projection_marker);

  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = marker_height_;

  geometry_msgs::msg::Point projection = error.projection;
  projection.z = marker_height_;

  visualization_msgs::msg::Marker cte_marker;
  cte_marker.header.stamp = now();
  cte_marker.header.frame_id = frame_id;
  cte_marker.ns = "stanley_error";
  cte_marker.id = 2;
  cte_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  cte_marker.action = visualization_msgs::msg::Marker::ADD;
  cte_marker.pose.orientation.w = 1.0;
  cte_marker.scale.x = marker_scale_ * 0.25;
  cte_marker.color.r = 1.0;
  cte_marker.color.g = 0.25;
  cte_marker.color.b = 0.15;
  cte_marker.color.a = 0.95;
  cte_marker.points.push_back(origin);
  cte_marker.points.push_back(projection);
  marker_array.markers.push_back(cte_marker);

  geometry_msgs::msg::Point heading_tip;
  heading_tip.x = std::cos(error.path_yaw) * 0.8;
  heading_tip.y = std::sin(error.path_yaw) * 0.8;
  heading_tip.z = marker_height_;

  visualization_msgs::msg::Marker heading_marker;
  heading_marker.header.stamp = now();
  heading_marker.header.frame_id = frame_id;
  heading_marker.ns = "stanley_error";
  heading_marker.id = 3;
  heading_marker.type = visualization_msgs::msg::Marker::ARROW;
  heading_marker.action = visualization_msgs::msg::Marker::ADD;
  heading_marker.pose.orientation.w = 1.0;
  heading_marker.scale.x = marker_scale_ * 0.45;
  heading_marker.scale.y = marker_scale_ * 0.85;
  heading_marker.scale.z = marker_scale_ * 0.85;
  heading_marker.color.r = 1.0;
  heading_marker.color.g = 0.75;
  heading_marker.color.b = 0.1;
  heading_marker.color.a = 0.95;
  heading_marker.points.push_back(origin);
  heading_marker.points.push_back(heading_tip);
  marker_array.markers.push_back(heading_marker);

  pub_error_markers_->publish(marker_array);
}

void Stanley::clearErrorMarkers(const std::string & frame_id) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.stamp = now();
  clear_marker.header.frame_id = frame_id;
  clear_marker.ns = "stanley_error";
  clear_marker.id = -1;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  pub_error_markers_->publish(marker_array);
}

double Stanley::normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}
