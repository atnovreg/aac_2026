#include "local_path_planner_node/local_path_planner.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace std::chrono_literals;

namespace
{
double shortestAngularDistance(double from, double to)
{
  const double diff = to - from;
  return std::atan2(std::sin(diff), std::cos(diff));
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

LocalPathPlanner::LocalPathPlanner(const std::string & node_name)
: Node(node_name)
{
  declare_parameter<double>("lookahead_dist",   5.0);
  declare_parameter<double>("heading_error_weight", 0.5);
  declare_parameter<double>("update_rate",      10.0);
  declare_parameter<std::string>("global_frame",       "map");
  declare_parameter<std::string>("body_frame",         "base_footprint");
  declare_parameter<std::string>("odom_topic",         "");
  declare_parameter<std::string>("global_path_topic",  "/path/center");
  declare_parameter<std::string>("local_global_topic", "/local_path/global");
  declare_parameter<std::string>("local_body_topic",   "/local_path/body");

  lookahead_dist_        = get_parameter("lookahead_dist").as_double();
  heading_error_weight_  = get_parameter("heading_error_weight").as_double();
  update_rate_           = get_parameter("update_rate").as_double();
  global_frame_          = get_parameter("global_frame").as_string();
  body_frame_            = get_parameter("body_frame").as_string();
  odom_topic_            = get_parameter("odom_topic").as_string();
  global_path_topic_     = get_parameter("global_path_topic").as_string();
  local_global_topic_    = get_parameter("local_global_topic").as_string();
  local_body_topic_      = get_parameter("local_body_topic").as_string();

  // Transient local so we receive the latched path even if we start after the publisher
  rclcpp::QoS latched_qos(1);
  latched_qos.transient_local();

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LocalPathPlanner::onOdom, this, std::placeholders::_1));

  sub_global_path_ = create_subscription<nav_msgs::msg::Path>(
    global_path_topic_, latched_qos,
    std::bind(&LocalPathPlanner::onGlobalPath, this, std::placeholders::_1));

  pub_local_global_ = create_publisher<nav_msgs::msg::Path>(local_global_topic_, 10);
  pub_local_body_   = create_publisher<nav_msgs::msg::Path>(local_body_topic_,   10);

  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / std::max(update_rate_, 0.1)));
  timer_ = create_wall_timer(
    period_ms, std::bind(&LocalPathPlanner::timerCallback, this));

  RCLCPP_INFO(get_logger(),
    "LocalPathPlanner ready — lookahead=%.1f m, heading_weight=%.2f, "
    "rate=%.1f Hz, odom=%s, body_frame=%s",
    lookahead_dist_, heading_error_weight_, update_rate_,
    odom_topic_.c_str(), body_frame_.c_str());
}

void LocalPathPlanner::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = *msg;
  has_odom_ = true;
}

void LocalPathPlanner::onGlobalPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  global_path_       = *msg;
  has_path_          = !global_path_.poses.empty();
  last_nearest_idx_  = 0;

  RCLCPP_INFO(get_logger(), "Received global path: %zu waypoints.",
    global_path_.poses.size());
}

void LocalPathPlanner::timerCallback()
{
  if (!has_path_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "Waiting for global path on '%s'…", global_path_topic_.c_str());
    return;
  }

  if (!has_odom_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "Waiting for odometry on '%s'…", odom_topic_.c_str());
    return;
  }

  const auto & current_pose = current_odom_.pose.pose;
  const double rx   = current_pose.position.x;
  const double ry   = current_pose.position.y;
  const double ryaw = yawFromQuaternion(current_pose.orientation);

  const std::string odom_frame = current_odom_.header.frame_id;
  const std::string path_frame = global_path_.header.frame_id;
  if (!odom_frame.empty() && !path_frame.empty() && odom_frame != path_frame) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "Odometry frame '%s' differs from global path frame '%s'. "
      "No TF transform is applied; odom pose is assumed to already be in the path frame.",
      odom_frame.c_str(), path_frame.c_str());
  }

  const std::string output_global_frame =
    !path_frame.empty() ? path_frame : (!odom_frame.empty() ? odom_frame : global_frame_);
  const std::string output_body_frame =
    !current_odom_.child_frame_id.empty() ? current_odom_.child_frame_id : body_frame_;

  const size_t closest = findClosestIndex(rx, ry, ryaw);
  last_nearest_idx_ = closest;

  nav_msgs::msg::Path path_global;
  nav_msgs::msg::Path path_body;
  extractLocalPath(
    closest, rx, ry, ryaw, output_global_frame, output_body_frame,
    path_global, path_body);

  pub_local_global_->publish(path_global);
  pub_local_body_->publish(path_body);
}

size_t LocalPathPlanner::findClosestIndex(double rx, double ry, double ryaw) const
{
  const size_t n = global_path_.poses.size();

  size_t closest = last_nearest_idx_;
  double min_cost = std::numeric_limits<double>::max();

  for (size_t i = 0; i < n; ++i) {
    const double dx = global_path_.poses[i].pose.position.x - rx;
    const double dy = global_path_.poses[i].pose.position.y - ry;
    const double distance = std::hypot(dx, dy);
    const double path_yaw = yawFromQuaternion(global_path_.poses[i].pose.orientation);
    const double heading_error = std::fabs(shortestAngularDistance(ryaw, path_yaw));
    const double cost = distance + heading_error_weight_ * heading_error;

    if (cost < min_cost) {
      min_cost = cost;
      closest = i;
    }
  }

  return closest;
}

void LocalPathPlanner::extractLocalPath(
  size_t start_idx, double rx, double ry, double ryaw,
  const std::string & global_frame, const std::string & body_frame,
  nav_msgs::msg::Path & out_global,
  nav_msgs::msg::Path & out_body) const
{
  const size_t n  = global_path_.poses.size();
  const auto stamp = now();

  out_global.header.stamp    = stamp;
  out_global.header.frame_id = global_frame;
  out_body.header.stamp      = stamp;
  out_body.header.frame_id   = body_frame;

  out_global.poses.clear();
  out_body.poses.clear();

  // Precompute rotation components for body transform (R(-yaw))
  const double cos_yaw = std::cos(ryaw);
  const double sin_yaw = std::sin(ryaw);

  double accumulated = 0.0;

  for (size_t k = 0; k < n; ++k) {
    const size_t cur  = (start_idx + k)     % n;
    const size_t next = (start_idx + k + 1) % n;

    const auto & pose = global_path_.poses[cur];
    const double px   = pose.pose.position.x;
    const double py   = pose.pose.position.y;
    const double pyaw = yawFromQuaternion(pose.pose.orientation);

    // — Global frame pose (copy as-is, update header only) —
    geometry_msgs::msg::PoseStamped ps_global;
    ps_global.header = out_global.header;
    ps_global.pose   = pose.pose;
    out_global.poses.push_back(ps_global);

    // — Body frame pose (translate then rotate) —
    const double dx = px - rx;
    const double dy = py - ry;
    geometry_msgs::msg::PoseStamped ps_body;
    ps_body.header          = out_body.header;
    ps_body.pose.position.x =  cos_yaw * dx + sin_yaw * dy;
    ps_body.pose.position.y = -sin_yaw * dx + cos_yaw * dy;
    ps_body.pose.position.z = 0.0;
    const double byaw = pyaw - ryaw;
    ps_body.pose.orientation.x = 0.0;
    ps_body.pose.orientation.y = 0.0;
    ps_body.pose.orientation.z = std::sin(byaw * 0.5);
    ps_body.pose.orientation.w = std::cos(byaw * 0.5);
    out_body.poses.push_back(ps_body);

    // Accumulate arc length to the next waypoint
    const double nx = global_path_.poses[next].pose.position.x;
    const double ny = global_path_.poses[next].pose.position.y;
    accumulated += std::hypot(nx - px, ny - py);

    if (accumulated >= lookahead_dist_) { break; }
  }
}
