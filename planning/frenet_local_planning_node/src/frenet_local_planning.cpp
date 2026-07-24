#include "frenet_local_planning_node/frenet_local_planning.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <tuple>

#include "frenet_utils.h"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

namespace
{
constexpr double kDuplicatePointEpsilon = 1.0e-4;
constexpr double kPi = 3.14159265358979323846;
}

FrenetLocalPlanning::FrenetLocalPlanning(const std::string &node_name)
    : Node(node_name)
{
  getParams();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  local_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      local_path_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FrenetLocalPlanning::localPathCallback, this, std::placeholders::_1));
  if (publish_map_visualization_) {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&FrenetLocalPlanning::odomCallback, this, std::placeholders::_1));
  }
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FrenetLocalPlanning::scanCallback, this, std::placeholders::_1));

  motion_primitives_pub_ =
      create_publisher<aac_msgs::msg::PathWithCostArray>(motion_primitives_topic_, 10);
  optimal_path_pub_ = create_publisher<nav_msgs::msg::Path>(optimal_path_topic_, 10);
  candidate_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(candidate_cloud_topic_, 10);
  reference_boundary_pub_ =
      create_publisher<geometry_msgs::msg::PolygonStamped>(reference_boundary_topic_, 10);
  if (publish_map_visualization_) {
    const auto visualization_qos = rclcpp::QoS(1).reliable().transient_local();
    optimal_path_visualization_pub_ =
        create_publisher<nav_msgs::msg::Path>(
            optimal_path_visualization_topic_, visualization_qos);
    candidate_cloud_visualization_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(
            candidate_cloud_visualization_topic_, visualization_qos);
    reference_boundary_visualization_pub_ =
        create_publisher<geometry_msgs::msg::PolygonStamped>(
            reference_boundary_visualization_topic_, visualization_qos);
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(update_rate_, 1.0e-3)));
  timer_ = create_wall_timer(period, std::bind(&FrenetLocalPlanning::timerCallback, this));
}

void FrenetLocalPlanning::getParams()
{
  declare_parameter<std::string>("local_path_topic", "/local_path/body");
  declare_parameter<std::string>("odom_topic", "/odom_gt");
  declare_parameter<std::string>("scan_topic", "/scan");
  declare_parameter<std::string>("motion_primitives_topic", "/frenet/motion_primitives");
  declare_parameter<std::string>("optimal_path_topic", "/frenet/optimal_path");
  declare_parameter<std::string>("candidate_cloud_topic", "/frenet/candidate_cloud");
  declare_parameter<std::string>("reference_boundary_topic", "/frenet/reference_boundary");
  declare_parameter<bool>("publish_map_visualization", true);
  declare_parameter<std::string>("visualization_frame_id", "map");
  declare_parameter<std::string>("optimal_path_visualization_topic",
                                 "/frenet/map_visualization/optimal_path");
  declare_parameter<std::string>("candidate_cloud_visualization_topic",
                                 "/frenet/map_visualization/candidate_cloud");
  declare_parameter<std::string>("reference_boundary_visualization_topic",
                                 "/frenet/map_visualization/reference_boundary");
  declare_parameter<bool>("republish_cached_map_visualization", false);
  declare_parameter<std::string>("fallback_frame_id", "base_footprint");
  declare_parameter<double>("update_rate", 50.0);
  declare_parameter<int>("input_stride", 2);
  declare_parameter<int>("min_path_points", 4);
  declare_parameter<int>("parallel_path_count", 1);
  declare_parameter<double>("parallel_scale", 0.0);
  declare_parameter<double>("left_width_margin", -0.5);
  declare_parameter<double>("right_width_margin", 0.5);
  declare_parameter<double>("width_delta", 0.25);
  declare_parameter<double>("min_prediction_time", 3.0);
  declare_parameter<double>("max_prediction_time", 3.21);
  declare_parameter<double>("prediction_time_step", 0.2);
  declare_parameter<double>("planning_speed_mps", 1.5);
  declare_parameter<double>("visualization_x_offset", 0.0);
  declare_parameter<bool>("require_scan_for_planning", true);
  declare_parameter<double>("obstacle_psi", 0.45);
  declare_parameter<double>("obstacle_influence_distance", 1.20);
  declare_parameter<double>("robot_collision_radius", 0.34);
  declare_parameter<double>("collision_penalty", 1000.0);
  declare_parameter<double>("scan_timeout", 0.50);
  declare_parameter<double>("obstacle_max_range", 4.0);
  declare_parameter<double>("obstacle_min_x", -0.30);
  declare_parameter<int>("scan_stride", 2);
  declare_parameter<double>("self_filter_min_x", -0.35);
  declare_parameter<double>("self_filter_max_x", 0.35);
  declare_parameter<double>("self_filter_half_width", 0.28);
  declare_parameter<double>("obstacle_cost_weight", 50.0);
  declare_parameter<double>("reference_cost_weight", 1.0);
  declare_parameter<double>("curvature_cost_weight", 0.10);
  declare_parameter<double>("transient_cost_weight", 2.0);
  declare_parameter<double>("origin_cost_weight", 0.10);

  local_path_topic_ = get_parameter("local_path_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  scan_topic_ = get_parameter("scan_topic").as_string();
  motion_primitives_topic_ = get_parameter("motion_primitives_topic").as_string();
  optimal_path_topic_ = get_parameter("optimal_path_topic").as_string();
  candidate_cloud_topic_ = get_parameter("candidate_cloud_topic").as_string();
  reference_boundary_topic_ = get_parameter("reference_boundary_topic").as_string();
  publish_map_visualization_ = get_parameter("publish_map_visualization").as_bool();
  visualization_frame_id_ = get_parameter("visualization_frame_id").as_string();
  optimal_path_visualization_topic_ =
      get_parameter("optimal_path_visualization_topic").as_string();
  candidate_cloud_visualization_topic_ =
      get_parameter("candidate_cloud_visualization_topic").as_string();
  reference_boundary_visualization_topic_ =
      get_parameter("reference_boundary_visualization_topic").as_string();
  republish_cached_map_visualization_ =
      get_parameter("republish_cached_map_visualization").as_bool();
  fallback_frame_id_ = get_parameter("fallback_frame_id").as_string();
  update_rate_ = get_parameter("update_rate").as_double();
  input_stride_ = std::max(1, static_cast<int>(get_parameter("input_stride").as_int()));
  min_path_points_ = std::max(2, static_cast<int>(get_parameter("min_path_points").as_int()));
  parallel_path_count_ =
      std::max(1, static_cast<int>(get_parameter("parallel_path_count").as_int()));
  parallel_scale_ = std::max(0.0, get_parameter("parallel_scale").as_double());
  left_width_margin_ = get_parameter("left_width_margin").as_double();
  right_width_margin_ = get_parameter("right_width_margin").as_double();
  width_delta_ = std::abs(get_parameter("width_delta").as_double());
  min_prediction_time_ = get_parameter("min_prediction_time").as_double();
  max_prediction_time_ = get_parameter("max_prediction_time").as_double();
  prediction_time_step_ = std::max(1.0e-3, get_parameter("prediction_time_step").as_double());
  planning_speed_mps_ = std::max(0.05, get_parameter("planning_speed_mps").as_double());
  visualization_x_offset_ = get_parameter("visualization_x_offset").as_double();
  require_scan_for_planning_ = get_parameter("require_scan_for_planning").as_bool();
  obstacle_psi_ = std::max(1.0e-3, get_parameter("obstacle_psi").as_double());
  obstacle_influence_distance_ =
      std::max(obstacle_psi_, get_parameter("obstacle_influence_distance").as_double());
  robot_collision_radius_ =
      std::max(0.0, get_parameter("robot_collision_radius").as_double());
  collision_penalty_ = std::max(1.0, get_parameter("collision_penalty").as_double());
  scan_timeout_ = std::max(0.05, get_parameter("scan_timeout").as_double());
  obstacle_max_range_ = std::max(0.1, get_parameter("obstacle_max_range").as_double());
  obstacle_min_x_ = get_parameter("obstacle_min_x").as_double();
  scan_stride_ = std::max(1, static_cast<int>(get_parameter("scan_stride").as_int()));
  self_filter_min_x_ = get_parameter("self_filter_min_x").as_double();
  self_filter_max_x_ = get_parameter("self_filter_max_x").as_double();
  self_filter_half_width_ =
      std::max(0.0, get_parameter("self_filter_half_width").as_double());
  obstacle_cost_weight_ =
      std::max(0.0, get_parameter("obstacle_cost_weight").as_double());
  reference_cost_weight_ =
      std::max(0.0, get_parameter("reference_cost_weight").as_double());
  curvature_cost_weight_ =
      std::max(0.0, get_parameter("curvature_cost_weight").as_double());
  transient_cost_weight_ =
      std::max(0.0, get_parameter("transient_cost_weight").as_double());
  origin_cost_weight_ =
      std::max(0.0, get_parameter("origin_cost_weight").as_double());

  if (left_width_margin_ > right_width_margin_) {
    std::swap(left_width_margin_, right_width_margin_);
  }
  if (max_prediction_time_ < min_prediction_time_) {
    std::swap(max_prediction_time_, min_prediction_time_);
  }
}

void FrenetLocalPlanning::localPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  latest_local_path_ = *msg;
  has_local_path_ = true;
}

void FrenetLocalPlanning::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = *msg;
  has_odom_ = true;
}

void FrenetLocalPlanning::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = *msg;
  has_scan_ = true;
}

void FrenetLocalPlanning::publishCachedMapVisualization()
{
  if (!publish_map_visualization_) {
    return;
  }
  if (!republish_cached_map_visualization_) {
    return;
  }

  nav_msgs::msg::Path optimal_path;
  sensor_msgs::msg::PointCloud2 candidate_cloud;
  geometry_msgs::msg::PolygonStamped reference_boundary;
  {
    std::lock_guard<std::mutex> lock(map_visualization_mutex_);
    if (!has_last_map_visualization_) {
      return;
    }
    optimal_path = last_optimal_path_visualization_;
    candidate_cloud = last_candidate_cloud_visualization_;
    reference_boundary = last_reference_boundary_visualization_;
  }

  const auto stamp = now();
  optimal_path.header.stamp = stamp;
  for (auto &pose : optimal_path.poses) {
    pose.header.stamp = stamp;
  }
  candidate_cloud.header.stamp = stamp;
  reference_boundary.header.stamp = stamp;

  optimal_path_visualization_pub_->publish(optimal_path);
  candidate_cloud_visualization_pub_->publish(candidate_cloud);
  reference_boundary_visualization_pub_->publish(reference_boundary);
}

void FrenetLocalPlanning::timerCallback()
{
  nav_msgs::msg::Path local_path;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!has_local_path_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for local path on %s", local_path_topic_.c_str());
      publishCachedMapVisualization();
      return;
    }
    local_path = latest_local_path_;
  }

  std::vector<double> local_x;
  std::vector<double> local_y;
  if (!extractPathXY(local_path, local_x, local_y)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Local path does not have enough usable points");
    publishCachedMapVisualization();
    return;
  }

  if (!planning_started_logged_) {
    RCLCPP_INFO(get_logger(),
                "Received valid local path on %s. Frenet local planning started.",
                local_path_topic_.c_str());
    planning_started_logged_ = true;
  }

  std_msgs::msg::Header header = local_path.header;
  header.stamp = now();
  if (header.frame_id.empty()) {
    header.frame_id = fallback_frame_id_;
  }

  std::vector<geometry_msgs::msg::Point> obstacles;
  if (!makeObstaclePoints(header.frame_id, obstacles) && require_scan_for_planning_) {
    nav_msgs::msg::Path empty_path;
    empty_path.header = header;
    optimal_path_pub_->publish(empty_path);
    publishCachedMapVisualization();
    return;
  }

  const auto reference_boundary = makeReferenceBoundary(local_x, local_y, header);

  std::vector<FrenetPathPtr> frenet_paths;
  for (int parallel_step = 0; parallel_step < parallel_path_count_; ++parallel_step) {
    const double parallel_offset =
        parallel_path_count_ == 1
            ? 0.0
            : -parallel_scale_ +
                  parallel_step * parallel_scale_ * 2.0 / (parallel_path_count_ - 1);

    std::vector<double> parallel_x;
    std::vector<double> parallel_y;
    frenet_utils::GetParallelRefPath(local_x, local_y, parallel_offset,
                                     parallel_x, parallel_y);

    auto generated_paths = generateForReferencePath(parallel_x, parallel_y);
    frenet_paths.insert(frenet_paths.end(), generated_paths.begin(), generated_paths.end());
  }

  aac_msgs::msg::PathWithCostArray path_array;
  path_array.header = header;
  path_array.motion_primitives.reserve(frenet_paths.size());

  double best_score = std::numeric_limits<double>::infinity();
  double best_lateral = 0.0;
  double best_obstacle_cost = 0.0;
  nav_msgs::msg::Path best_path;
  bool has_best_path = false;

  int index = 0;
  for (const auto &frenet_path : frenet_paths) {
    if (!frenet_path || frenet_path->points_x().size() < 2) {
      continue;
    }

    auto path_with_cost = makePathWithCost(*frenet_path, header, index, obstacles);
    const double score = pathScore(path_with_cost);
    const bool in_collision =
        path_with_cost.collision_cost.data >= collision_penalty_;
    if (!in_collision && score < best_score) {
      best_score = score;
      best_path = path_with_cost.primitive;
      best_lateral = frenet_path->points_d().empty()
                         ? 0.0
                         : frenet_path->points_d().back();
      best_obstacle_cost = path_with_cost.collision_cost.data;
      has_best_path = true;
    }
    path_array.motion_primitives.push_back(path_with_cost);
    ++index;
  }

  motion_primitives_pub_->publish(path_array);
  candidate_cloud_pub_->publish(makeCandidateCloud(frenet_paths, header));
  reference_boundary_pub_->publish(reference_boundary);
  if (has_best_path) {
    optimal_path_pub_->publish(best_path);
    last_selected_lateral_ = best_lateral;
    has_last_selected_lateral_ = true;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Selected Frenet path: lateral=%.2f m, obstacle_potential=%.3f, score=%.2f",
        best_lateral, best_obstacle_cost, best_score);
  } else {
    nav_msgs::msg::Path empty_path;
    empty_path.header = header;
    optimal_path_pub_->publish(empty_path);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "All %zu Frenet candidates are in collision; publishing an empty path",
        path_array.motion_primitives.size());
  }

  if (publish_map_visualization_) {
    nav_msgs::msg::Odometry odom;
    bool has_odom = false;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      has_odom = has_odom_;
      if (has_odom) {
        odom = latest_odom_;
      }
    }

    if (!has_odom) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for odometry on %s before publishing map visualization",
          odom_topic_.c_str());
      publishCachedMapVisualization();
    } else if (!has_best_path) {
      publishCachedMapVisualization();
    } else {
      const auto visualization_header = makeVisualizationHeader(header);

      const auto candidate_cloud_visualization =
          makeVisualizationCandidateCloud(frenet_paths, odom, visualization_header);

      const auto reference_boundary_visualization =
          makeVisualizationBoundary(reference_boundary, odom, visualization_header);

      const auto optimal_path_visualization =
          makeVisualizationPath(best_path, odom, visualization_header);

      optimal_path_visualization_pub_->publish(optimal_path_visualization);
      candidate_cloud_visualization_pub_->publish(candidate_cloud_visualization);
      reference_boundary_visualization_pub_->publish(reference_boundary_visualization);

      {
        std::lock_guard<std::mutex> lock(map_visualization_mutex_);
        last_optimal_path_visualization_ = optimal_path_visualization;
        last_candidate_cloud_visualization_ = candidate_cloud_visualization;
        last_reference_boundary_visualization_ = reference_boundary_visualization;
        has_last_map_visualization_ = true;
      }
    }
  }

  RCLCPP_DEBUG(get_logger(), "Published %zu Frenet primitives",
               path_array.motion_primitives.size());
}

bool FrenetLocalPlanning::extractPathXY(const nav_msgs::msg::Path &path,
                                        std::vector<double> &points_x,
                                        std::vector<double> &points_y) const
{
  points_x.clear();
  points_y.clear();

  for (std::size_t i = 0; i < path.poses.size(); i += static_cast<std::size_t>(input_stride_)) {
    const auto &position = path.poses[i].pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
      continue;
    }
    if (!points_x.empty() &&
        std::hypot(position.x - points_x.back(), position.y - points_y.back()) <
            kDuplicatePointEpsilon) {
      continue;
    }
    points_x.push_back(position.x);
    points_y.push_back(position.y);
  }

  if (!path.poses.empty()) {
    const auto &position = path.poses.back().pose.position;
    if (std::isfinite(position.x) && std::isfinite(position.y) &&
        (points_x.empty() ||
         std::hypot(position.x - points_x.back(), position.y - points_y.back()) >=
             kDuplicatePointEpsilon)) {
      points_x.push_back(position.x);
      points_y.push_back(position.y);
    }
  }

  return static_cast<int>(points_x.size()) >= min_path_points_;
}

std::vector<FrenetLocalPlanning::FrenetPathPtr>
FrenetLocalPlanning::generateForReferencePath(const std::vector<double> &ref_x,
                                              const std::vector<double> &ref_y)
{
  if (ref_x.size() < static_cast<std::size_t>(min_path_points_) ||
      ref_x.size() != ref_y.size()) {
    return {};
  }

  std::vector<double> cumulative_s(ref_x.size(), 0.0);
  int closest_idx = 0;
  double closest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < ref_x.size(); ++i) {
    if (i > 0) {
      cumulative_s[i] =
          cumulative_s[i - 1] + std::hypot(ref_x[i] - ref_x[i - 1], ref_y[i] - ref_y[i - 1]);
    }
    const double dist = std::hypot(ref_x[i], ref_y[i]);
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_idx = static_cast<int>(i);
    }
  }

  try {
    FrenetPathGenerator generator;
    std::vector<double> mutable_ref_x = ref_x;
    std::vector<double> mutable_ref_y = ref_y;
    auto spline_result = generator.apply_cubic_spliner(mutable_ref_x, mutable_ref_y);
    auto spline = std::get<4>(spline_result);

    const double current_position_d = -ref_y[closest_idx];
    const double current_position_s = cumulative_s[closest_idx];

    auto generated = generator.calc_frenet_paths_v2(
        current_position_d, current_position_s, 0.0, planning_speed_mps_, 0.0,
        spline, min_prediction_time_, max_prediction_time_, prediction_time_step_,
        left_width_margin_, right_width_margin_, width_delta_, 0);

    return std::get<1>(generated);
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Failed to generate Frenet paths: %s", e.what());
    return {};
  }
}

geometry_msgs::msg::PolygonStamped FrenetLocalPlanning::makeReferenceBoundary(
    const std::vector<double> &ref_x,
    const std::vector<double> &ref_y,
    const std_msgs::msg::Header &header) const
{
  geometry_msgs::msg::PolygonStamped boundary;
  boundary.header = header;

  if (ref_x.size() < 2 || ref_x.size() != ref_y.size()) {
    return boundary;
  }

  std::vector<double> left_x;
  std::vector<double> left_y;
  std::vector<double> right_x;
  std::vector<double> right_y;

  const double left_offset = std::min(left_width_margin_, right_width_margin_);
  const double right_offset = std::max(left_width_margin_, right_width_margin_);

  frenet_utils::GetParallelRefPath(ref_x, ref_y, left_offset, left_x, left_y);
  frenet_utils::GetParallelRefPath(ref_x, ref_y, right_offset, right_x, right_y);

  for (std::size_t i = 0; i < left_x.size() && i < left_y.size(); ++i) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(left_x[i]);
    point.y = static_cast<float>(left_y[i]);
    boundary.polygon.points.push_back(point);
  }

  const std::size_t right_count = std::min(right_x.size(), right_y.size());
  for (std::size_t i = right_count; i > 0; --i) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(right_x[i - 1]);
    point.y = static_cast<float>(right_y[i - 1]);
    boundary.polygon.points.push_back(point);
  }

  return boundary;
}

bool FrenetLocalPlanning::makeObstaclePoints(
    const std::string &target_frame,
    std::vector<geometry_msgs::msg::Point> &obstacles)
{
  obstacles.clear();

  sensor_msgs::msg::LaserScan scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (!has_scan_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for obstacle LaserScan on %s", scan_topic_.c_str());
      return false;
    }
    scan = latest_scan_;
  }

  const rclcpp::Time scan_stamp(scan.header.stamp);
  if (scan_stamp.nanoseconds() > 0) {
    const double age = (now() - scan_stamp).seconds();
    if (age > scan_timeout_ || age < -0.1) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Obstacle scan is stale (age %.2f s, limit %.2f s)", age, scan_timeout_);
      return false;
    }
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
        target_frame, scan.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &error) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform obstacle scan from %s to %s: %s",
        scan.header.frame_id.c_str(), target_frame.c_str(), error.what());
    return false;
  }

  const double transform_yaw = yawFromQuaternion(transform.transform.rotation);
  const double cos_yaw = std::cos(transform_yaw);
  const double sin_yaw = std::sin(transform_yaw);
  const double tx = transform.transform.translation.x;
  const double ty = transform.transform.translation.y;

  obstacles.reserve(scan.ranges.size() / static_cast<std::size_t>(scan_stride_) + 1);
  for (std::size_t i = 0; i < scan.ranges.size();
       i += static_cast<std::size_t>(scan_stride_)) {
    const double range = static_cast<double>(scan.ranges[i]);
    if (!std::isfinite(range) || range < static_cast<double>(scan.range_min) ||
        range > static_cast<double>(scan.range_max) || range > obstacle_max_range_) {
      continue;
    }

    const double angle = static_cast<double>(scan.angle_min) +
                         static_cast<double>(i) * static_cast<double>(scan.angle_increment);
    const double scan_x = range * std::cos(angle);
    const double scan_y = range * std::sin(angle);

    geometry_msgs::msg::Point point;
    point.x = tx + cos_yaw * scan_x - sin_yaw * scan_y;
    point.y = ty + sin_yaw * scan_x + cos_yaw * scan_y;
    point.z = 0.0;

    const bool is_robot_self_return =
        point.x >= self_filter_min_x_ && point.x <= self_filter_max_x_ &&
        std::abs(point.y) <= self_filter_half_width_;
    if (is_robot_self_return || point.x < obstacle_min_x_) {
      continue;
    }
    obstacles.push_back(point);
  }

  RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Using %zu transformed LaserScan obstacle points", obstacles.size());
  return true;
}

double FrenetLocalPlanning::obstaclePotentialCost(
    const FrenetPath &frenet_path,
    const std::vector<geometry_msgs::msg::Point> &obstacles,
    bool &in_collision,
    double &minimum_clearance) const
{
  in_collision = false;
  minimum_clearance = std::numeric_limits<double>::infinity();

  const auto &points_x = frenet_path.points_x();
  const auto &points_y = frenet_path.points_y();
  const std::size_t path_size = std::min(points_x.size(), points_y.size());
  if (path_size == 0 || obstacles.empty()) {
    return 0.0;
  }

  double minimum_distance_squared = std::numeric_limits<double>::infinity();
  for (const auto &obstacle : obstacles) {
    if (path_size == 1) {
      const double dx = obstacle.x - points_x.front();
      const double dy = obstacle.y - points_y.front();
      minimum_distance_squared =
          std::min(minimum_distance_squared, dx * dx + dy * dy);
      continue;
    }

    for (std::size_t i = 1; i < path_size; ++i) {
      const double x0 = points_x[i - 1];
      const double y0 = points_y[i - 1];
      const double segment_x = points_x[i] - x0;
      const double segment_y = points_y[i] - y0;
      const double segment_length_squared =
          segment_x * segment_x + segment_y * segment_y;

      double ratio = 0.0;
      if (segment_length_squared > 1.0e-9) {
        ratio = ((obstacle.x - x0) * segment_x +
                 (obstacle.y - y0) * segment_y) /
                segment_length_squared;
        ratio = std::clamp(ratio, 0.0, 1.0);
      }
      const double closest_x = x0 + ratio * segment_x;
      const double closest_y = y0 + ratio * segment_y;
      const double dx = obstacle.x - closest_x;
      const double dy = obstacle.y - closest_y;
      minimum_distance_squared =
          std::min(minimum_distance_squared, dx * dx + dy * dy);
    }
  }

  if (!std::isfinite(minimum_distance_squared)) {
    return 0.0;
  }

  minimum_clearance =
      std::sqrt(minimum_distance_squared) - robot_collision_radius_;
  if (minimum_clearance <= 0.0) {
    in_collision = true;
    return collision_penalty_;
  }
  if (minimum_clearance >= obstacle_influence_distance_) {
    return 0.0;
  }

  const double normalized_clearance = minimum_clearance / obstacle_psi_;
  return std::exp(-0.5 * normalized_clearance * normalized_clearance);
}

aac_msgs::msg::PathWithCost
FrenetLocalPlanning::makePathWithCost(
    const FrenetPath &frenet_path,
    const std_msgs::msg::Header &header,
    int index,
    const std::vector<geometry_msgs::msg::Point> &obstacles) const
{
  aac_msgs::msg::PathWithCost path_with_cost;
  path_with_cost.header = header;
  path_with_cost.primitive = makeRosPath(frenet_path, header);
  path_with_cost.idx.data = index;
  bool in_collision = false;
  double minimum_clearance = std::numeric_limits<double>::infinity();
  path_with_cost.collision_cost.data = obstaclePotentialCost(
      frenet_path, obstacles, in_collision, minimum_clearance);
  path_with_cost.close_to_ref_path_cost.data =
      frenet_path.points_d().empty() ? 0.0 : std::abs(frenet_path.points_d().back());
  path_with_cost.curvature_cost.data = maxAbsCurvature(frenet_path);
  const double candidate_lateral =
      frenet_path.points_d().empty() ? 0.0 : frenet_path.points_d().back();
  const double lateral_change = candidate_lateral - last_selected_lateral_;
  path_with_cost.transient_cost.data =
      has_last_selected_lateral_ ? lateral_change * lateral_change : 0.0;
  path_with_cost.origin_to_path_cost.data = minDistanceToOrigin(frenet_path);
  return path_with_cost;
}

nav_msgs::msg::Path FrenetLocalPlanning::makeRosPath(
    const FrenetPath &frenet_path, const std_msgs::msg::Header &header) const
{
  nav_msgs::msg::Path path;
  path.header = header;

  const auto &points_x = frenet_path.points_x();
  const auto &points_y = frenet_path.points_y();
  const auto &yaws = frenet_path.yaw();
  path.poses.reserve(std::min(points_x.size(), points_y.size()));

  for (std::size_t i = 0; i < points_x.size() && i < points_y.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = points_x[i];
    pose.pose.position.y = points_y[i];
    pose.pose.position.z = 0.0;

    double yaw = 0.0;
    if (i < yaws.size()) {
      yaw = yaws[i];
    } else if (i + 1 < points_x.size()) {
      yaw = std::atan2(points_y[i + 1] - points_y[i], points_x[i + 1] - points_x[i]);
    }
    pose.pose.orientation = yawToQuaternion(yaw);
    path.poses.push_back(pose);
  }

  return path;
}

sensor_msgs::msg::PointCloud2 FrenetLocalPlanning::makeCandidateCloud(
    const std::vector<FrenetPathPtr> &frenet_paths,
    const std_msgs::msg::Header &header) const
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;

  std::size_t point_count = 0;
  for (const auto &path : frenet_paths) {
    if (path) {
      point_count += std::min(path->points_x().size(), path->points_y().size());
    }
  }

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(4,
                                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "intensity", 1,
                                sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(point_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

  int path_index = 0;
  for (const auto &path : frenet_paths) {
    if (!path) {
      continue;
    }
    const auto &points_x = path->points_x();
    const auto &points_y = path->points_y();
    for (std::size_t i = 0; i < points_x.size() && i < points_y.size(); ++i) {
      *iter_x = static_cast<float>(points_x[i] + visualization_x_offset_);
      *iter_y = static_cast<float>(points_y[i]);
      *iter_z = 0.0F;
      *iter_intensity = static_cast<float>(path_index);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
    ++path_index;
  }

  return cloud;
}

std_msgs::msg::Header FrenetLocalPlanning::makeVisualizationHeader(
    const std_msgs::msg::Header &source_header) const
{
  auto header = source_header;
  header.frame_id = visualization_frame_id_;
  return header;
}

nav_msgs::msg::Path FrenetLocalPlanning::makeVisualizationPath(
    const nav_msgs::msg::Path &source_path,
    const nav_msgs::msg::Odometry &odom,
    const std_msgs::msg::Header &visualization_header) const
{
  nav_msgs::msg::Path path = source_path;
  path.header = visualization_header;
  for (auto &pose : path.poses) {
    pose.header = visualization_header;
    const auto map_point = transformBodyPointToMap(
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, odom);
    pose.pose.position = map_point;
    const double map_yaw =
        normalizeAngle(yawFromQuaternion(odom.pose.pose.orientation) +
                       yawFromQuaternion(pose.pose.orientation));
    pose.pose.orientation = yawToQuaternion(map_yaw);
  }
  return path;
}

sensor_msgs::msg::PointCloud2 FrenetLocalPlanning::makeVisualizationCandidateCloud(
    const std::vector<FrenetPathPtr> &frenet_paths,
    const nav_msgs::msg::Odometry &odom,
    const std_msgs::msg::Header &visualization_header) const
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = visualization_header;

  std::size_t point_count = 0;
  for (const auto &path : frenet_paths) {
    if (path) {
      point_count += std::min(path->points_x().size(), path->points_y().size());
    }
  }

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(4,
                                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                "intensity", 1,
                                sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(point_count);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

  int path_index = 0;
  for (const auto &path : frenet_paths) {
    if (!path) {
      continue;
    }
    const auto &points_x = path->points_x();
    const auto &points_y = path->points_y();
    for (std::size_t i = 0; i < points_x.size() && i < points_y.size(); ++i) {
      const auto map_point = transformBodyPointToMap(
          points_x[i] + visualization_x_offset_, points_y[i], 0.0, odom);
      *iter_x = static_cast<float>(map_point.x);
      *iter_y = static_cast<float>(map_point.y);
      *iter_z = static_cast<float>(map_point.z);
      *iter_intensity = static_cast<float>(path_index);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
    ++path_index;
  }

  return cloud;
}

geometry_msgs::msg::PolygonStamped FrenetLocalPlanning::makeVisualizationBoundary(
    const geometry_msgs::msg::PolygonStamped &source_boundary,
    const nav_msgs::msg::Odometry &odom,
    const std_msgs::msg::Header &visualization_header) const
{
  auto boundary = source_boundary;
  boundary.header = visualization_header;
  for (auto &point : boundary.polygon.points) {
    point = transformBodyPointToMap(point, odom);
  }
  return boundary;
}

geometry_msgs::msg::Quaternion FrenetLocalPlanning::yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double FrenetLocalPlanning::yawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double FrenetLocalPlanning::normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

geometry_msgs::msg::Point FrenetLocalPlanning::transformBodyPointToMap(
    double body_x,
    double body_y,
    double body_z,
    const nav_msgs::msg::Odometry &odom)
{
  const double yaw = yawFromQuaternion(odom.pose.pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  geometry_msgs::msg::Point map_point;
  map_point.x = odom.pose.pose.position.x + cos_yaw * body_x - sin_yaw * body_y;
  map_point.y = odom.pose.pose.position.y + sin_yaw * body_x + cos_yaw * body_y;
  map_point.z = odom.pose.pose.position.z + body_z;
  return map_point;
}

geometry_msgs::msg::Point32 FrenetLocalPlanning::transformBodyPointToMap(
    const geometry_msgs::msg::Point32 &body_point,
    const nav_msgs::msg::Odometry &odom)
{
  const auto map_point =
      transformBodyPointToMap(body_point.x, body_point.y, body_point.z, odom);
  geometry_msgs::msg::Point32 map_point32;
  map_point32.x = static_cast<float>(map_point.x);
  map_point32.y = static_cast<float>(map_point.y);
  map_point32.z = static_cast<float>(map_point.z);
  return map_point32;
}

double FrenetLocalPlanning::maxAbsCurvature(const FrenetPath &frenet_path)
{
  double max_curvature = 0.0;
  for (const double curvature : frenet_path.curvature()) {
    max_curvature = std::max(max_curvature, std::abs(curvature));
  }
  return max_curvature;
}

double FrenetLocalPlanning::minDistanceToOrigin(const FrenetPath &frenet_path)
{
  const auto &points_x = frenet_path.points_x();
  const auto &points_y = frenet_path.points_y();
  double min_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points_x.size() && i < points_y.size(); ++i) {
    min_dist = std::min(min_dist, std::hypot(points_x[i], points_y[i]));
  }
  return std::isfinite(min_dist) ? min_dist : 0.0;
}

double FrenetLocalPlanning::pathScore(
    const aac_msgs::msg::PathWithCost &path_with_cost) const
{
  return path_with_cost.collision_cost.data * obstacle_cost_weight_ +
         path_with_cost.close_to_ref_path_cost.data * reference_cost_weight_ +
         path_with_cost.curvature_cost.data * curvature_cost_weight_ +
         path_with_cost.origin_to_path_cost.data * origin_cost_weight_ +
         path_with_cost.transient_cost.data * transient_cost_weight_;
}
