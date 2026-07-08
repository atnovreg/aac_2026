#include "hybrid_a_star_planner_node/hybrid_a_star_planner.hpp"

#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_set>

namespace hybrid_a_star_planner_node
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr std::int64_t kInvalidKey = -1;

std::string normalizeFrameId(std::string frame_id)
{
  if (!frame_id.empty() && frame_id.front() == '/') {
    frame_id.erase(frame_id.begin());
  }
  return frame_id;
}

std::string toLower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}
}  // namespace

HybridAStarPlanner::HybridAStarPlanner(const std::string & node_name)
: Node(node_name),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  declare_parameter<std::string>("global_frame", "map");
  declare_parameter<std::string>("base_frame", "base_footprint");
  declare_parameter<std::string>("start_pose_source", "tf");
  declare_parameter<std::string>("goal_topic", "/goal_pose");
  declare_parameter<std::string>("initial_pose_topic", "/initialpose");
  declare_parameter<std::string>("map_topic", "/map");
  declare_parameter<std::string>("path_topic", "/hybrid_astar/path");
  declare_parameter<std::string>("marker_topic", "/hybrid_astar/markers");
  declare_parameter<std::string>("analytic_expansion_type", "reed_shepp");

  declare_parameter<bool>("use_occupancy_grid", true);
  declare_parameter<bool>("allow_unknown", true);
  declare_parameter<bool>("reverse_enabled", true);
  declare_parameter<bool>("publish_markers", true);
  declare_parameter<bool>("publish_debug_markers", true);

  declare_parameter<int>("occupied_threshold", 50);
  declare_parameter<int>("heading_bins", 72);
  declare_parameter<int>("steering_sample_count", 7);
  declare_parameter<int>("max_iterations", 50000);
  declare_parameter<int>("max_debug_points", 3000);
  declare_parameter<int>("max_debug_edges", 3000);

  declare_parameter<double>("transform_timeout", 0.2);
  declare_parameter<double>("map_x_min", -5.5);
  declare_parameter<double>("map_x_max", 5.5);
  declare_parameter<double>("map_y_min", -8.0);
  declare_parameter<double>("map_y_max", 8.0);
  declare_parameter<double>("fallback_resolution", 0.2);
  declare_parameter<double>("step_size", 0.35);
  declare_parameter<double>("wheel_base", 0.8);
  declare_parameter<double>("max_steering_angle", 0.52);
  declare_parameter<double>("collision_radius", 0.35);
  declare_parameter<double>("collision_check_step", 0.08);
  declare_parameter<double>("goal_xy_tolerance", 0.35);
  declare_parameter<double>("goal_yaw_tolerance", 0.45);
  declare_parameter<double>("analytic_expansion_distance", 0.8);
  declare_parameter<double>("analytic_expansion_yaw_tolerance", 0.7);
  declare_parameter<double>("analytic_expansion_max_length", 8.0);
  declare_parameter<double>("analytic_curve_sample_distance", 0.08);
  declare_parameter<double>("minimum_turning_radius", 0.0);
  declare_parameter<double>("path_interpolation_distance", 0.1);
  declare_parameter<double>("reverse_penalty", 2.0);
  declare_parameter<double>("direction_change_penalty", 1.0);
  declare_parameter<double>("steering_penalty", 0.05);
  declare_parameter<double>("steering_change_penalty", 0.15);
  declare_parameter<double>("heuristic_weight", 1.15);
  declare_parameter<double>("yaw_heuristic_weight", 0.15);
  declare_parameter<std::vector<double>>("steering_angles", std::vector<double>{});

  global_frame_ = normalizeFrameId(get_parameter("global_frame").as_string());
  base_frame_ = normalizeFrameId(get_parameter("base_frame").as_string());
  start_pose_source_ = get_parameter("start_pose_source").as_string();
  goal_topic_ = get_parameter("goal_topic").as_string();
  initial_pose_topic_ = get_parameter("initial_pose_topic").as_string();
  map_topic_ = get_parameter("map_topic").as_string();
  path_topic_ = get_parameter("path_topic").as_string();
  marker_topic_ = get_parameter("marker_topic").as_string();
  analytic_expansion_type_ = toLower(get_parameter("analytic_expansion_type").as_string());

  use_occupancy_grid_ = get_parameter("use_occupancy_grid").as_bool();
  allow_unknown_ = get_parameter("allow_unknown").as_bool();
  reverse_enabled_ = get_parameter("reverse_enabled").as_bool();
  publish_markers_ = get_parameter("publish_markers").as_bool();
  publish_debug_markers_ = get_parameter("publish_debug_markers").as_bool();

  occupied_threshold_ = get_parameter("occupied_threshold").as_int();
  heading_bins_ = std::max(8, static_cast<int>(get_parameter("heading_bins").as_int()));
  steering_sample_count_ = std::max(1, static_cast<int>(get_parameter("steering_sample_count").as_int()));
  max_iterations_ = std::max(1, static_cast<int>(get_parameter("max_iterations").as_int()));
  max_debug_points_ = std::max(0, static_cast<int>(get_parameter("max_debug_points").as_int()));
  max_debug_edges_ = std::max(0, static_cast<int>(get_parameter("max_debug_edges").as_int()));

  transform_timeout_ = std::max(0.0, get_parameter("transform_timeout").as_double());
  map_x_min_ = get_parameter("map_x_min").as_double();
  map_x_max_ = get_parameter("map_x_max").as_double();
  map_y_min_ = get_parameter("map_y_min").as_double();
  map_y_max_ = get_parameter("map_y_max").as_double();
  fallback_resolution_ = std::max(0.02, get_parameter("fallback_resolution").as_double());
  step_size_ = std::max(0.05, get_parameter("step_size").as_double());
  wheel_base_ = std::max(0.05, get_parameter("wheel_base").as_double());
  max_steering_angle_ = std::max(0.01, get_parameter("max_steering_angle").as_double());
  collision_radius_ = std::max(0.0, get_parameter("collision_radius").as_double());
  collision_check_step_ = std::max(0.02, get_parameter("collision_check_step").as_double());
  goal_xy_tolerance_ = std::max(0.02, get_parameter("goal_xy_tolerance").as_double());
  goal_yaw_tolerance_ = std::max(0.0, get_parameter("goal_yaw_tolerance").as_double());
  analytic_expansion_distance_ = std::max(0.0, get_parameter("analytic_expansion_distance").as_double());
  analytic_expansion_yaw_tolerance_ =
    std::max(0.0, get_parameter("analytic_expansion_yaw_tolerance").as_double());
  analytic_expansion_max_length_ =
    std::max(0.0, get_parameter("analytic_expansion_max_length").as_double());
  analytic_curve_sample_distance_ =
    std::max(0.02, get_parameter("analytic_curve_sample_distance").as_double());
  minimum_turning_radius_ = std::max(0.0, get_parameter("minimum_turning_radius").as_double());
  path_interpolation_distance_ =
    std::max(0.02, get_parameter("path_interpolation_distance").as_double());
  reverse_penalty_ = std::max(1.0, get_parameter("reverse_penalty").as_double());
  direction_change_penalty_ = std::max(0.0, get_parameter("direction_change_penalty").as_double());
  steering_penalty_ = std::max(0.0, get_parameter("steering_penalty").as_double());
  steering_change_penalty_ = std::max(0.0, get_parameter("steering_change_penalty").as_double());
  heuristic_weight_ = std::max(0.0, get_parameter("heuristic_weight").as_double());
  yaw_heuristic_weight_ = std::max(0.0, get_parameter("yaw_heuristic_weight").as_double());
  configured_steering_angles_ = get_parameter("steering_angles").as_double_array();
  steering_samples_ = buildSteeringSamples();

  sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(10),
    std::bind(&HybridAStarPlanner::onGoal, this, std::placeholders::_1));
  sub_initial_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initial_pose_topic_, rclcpp::QoS(10),
    std::bind(&HybridAStarPlanner::onInitialPose, this, std::placeholders::_1));

  rclcpp::QoS map_qos(1);
  map_qos.transient_local().reliable();
  sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&HybridAStarPlanner::onMap, this, std::placeholders::_1));

  rclcpp::QoS latched_qos(1);
  latched_qos.transient_local().reliable();
  pub_path_ = create_publisher<nav_msgs::msg::Path>(path_topic_, latched_qos);
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, latched_qos);

  RCLCPP_INFO(
    get_logger(),
    "HybridAStarPlanner ready: goal=%s, start_source=%s, output=%s, frame=%s, "
    "step=%.2f m, heading_bins=%d, steering_samples=%zu, analytic=%s, reverse=%s",
    goal_topic_.c_str(), start_pose_source_.c_str(), path_topic_.c_str(), global_frame_.c_str(),
    step_size_, heading_bins_, steering_samples_.size(), analytic_expansion_type_.c_str(),
    reverse_enabled_ ? "true" : "false");
}

void HybridAStarPlanner::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  Pose2D goal;
  if (!poseStampedToGlobal(*msg, goal)) {
    publishEmptyPath();
    return;
  }

  Pose2D start;
  std::string start_source;
  if (!resolveStartPose(start, start_source)) {
    publishEmptyPath();
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Planning Hybrid A*: start=(%.2f, %.2f, %.1f deg, %s), goal=(%.2f, %.2f, %.1f deg)",
    start.x, start.y, start.yaw * 180.0 / kPi, start_source.c_str(),
    goal.x, goal.y, goal.yaw * 180.0 / kPi);

  std::vector<Pose2D> path;
  std::vector<Pose2D> expanded;
  std::vector<DebugEdge> debug_edges;
  const auto t0 = std::chrono::steady_clock::now();
  const bool ok = makePlan(start, goal, path, expanded, debug_edges);
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();

  if (!ok) {
    RCLCPP_WARN(
      get_logger(), "Hybrid A* failed after %ld ms. Expanded states: %zu",
      elapsed_ms, expanded.size());
    publishEmptyPath();
    publishMarkers(start, goal, path, expanded, debug_edges);
    return;
  }

  publishPath(path);
  publishMarkers(start, goal, path, expanded, debug_edges);

  RCLCPP_INFO(
    get_logger(), "Hybrid A* path published: %zu poses, expanded=%zu, elapsed=%ld ms",
    path.size(), expanded.size(), elapsed_ms);
}

void HybridAStarPlanner::onInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  Pose2D pose;
  if (!poseWithCovarianceToGlobal(*msg, pose)) {
    RCLCPP_WARN(get_logger(), "Ignored /initialpose because it could not be transformed to %s.",
      global_frame_.c_str());
    return;
  }

  latest_initial_pose_ = pose;
  RCLCPP_INFO(
    get_logger(), "Stored initial pose: x=%.2f, y=%.2f, yaw=%.1f deg",
    pose.x, pose.y, pose.yaw * 180.0 / kPi);
}

void HybridAStarPlanner::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = msg;
  const std::string map_frame = normalizeFrameId(msg->header.frame_id);
  if (!map_frame.empty() && map_frame != global_frame_) {
    RCLCPP_WARN(
      get_logger(),
      "Received map in frame '%s' while global_frame is '%s'. Occupancy coordinates are assumed "
      "to already match the global frame.",
      map_frame.c_str(), global_frame_.c_str());
  }

  RCLCPP_INFO(
    get_logger(), "Received occupancy grid: %ux%u @ %.3f m/cell",
    msg->info.width, msg->info.height, msg->info.resolution);
}

bool HybridAStarPlanner::resolveStartPose(Pose2D & start_pose, std::string & source_name)
{
  const auto try_tf = [&]() -> bool {
    try {
      const auto tf = tf_buffer_.lookupTransform(
        global_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(transform_timeout_));
      start_pose.x = tf.transform.translation.x;
      start_pose.y = tf.transform.translation.y;
      start_pose.yaw = yawFromQuaternion(tf.transform.rotation);
      source_name = "tf";
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot use TF start pose %s -> %s: %s",
        global_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  };

  const auto try_initial_pose = [&]() -> bool {
    if (!latest_initial_pose_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No /initialpose has been received for Hybrid A* start pose.");
      return false;
    }
    start_pose = *latest_initial_pose_;
    source_name = "initialpose";
    return true;
  };

  if (start_pose_source_ == "tf") {
    return try_tf();
  }
  if (start_pose_source_ == "initialpose") {
    return try_initial_pose();
  }
  if (start_pose_source_ != "auto") {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Unknown start_pose_source='%s'. Using auto fallback.", start_pose_source_.c_str());
  }

  return try_tf() || try_initial_pose();
}

bool HybridAStarPlanner::poseStampedToGlobal(
  const geometry_msgs::msg::PoseStamped & input, Pose2D & pose)
{
  geometry_msgs::msg::PoseStamped source = input;
  source.header.frame_id = normalizeFrameId(source.header.frame_id);
  if (source.header.frame_id.empty()) {
    source.header.frame_id = global_frame_;
  }

  geometry_msgs::msg::PoseStamped transformed;
  if (source.header.frame_id == global_frame_) {
    transformed = source;
  } else {
    try {
      const auto tf = tf_buffer_.lookupTransform(
        global_frame_, source.header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_));
      tf2::doTransform(source, transformed, tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "Cannot transform pose from '%s' to '%s': %s",
        source.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
      return false;
    }
  }

  pose.x = transformed.pose.position.x;
  pose.y = transformed.pose.position.y;
  pose.yaw = yawFromQuaternion(transformed.pose.orientation);
  return true;
}

bool HybridAStarPlanner::poseWithCovarianceToGlobal(
  const geometry_msgs::msg::PoseWithCovarianceStamped & input, Pose2D & pose)
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header = input.header;
  stamped.pose = input.pose.pose;
  return poseStampedToGlobal(stamped, pose);
}

bool HybridAStarPlanner::makePlan(
  const Pose2D & start,
  const Pose2D & goal,
  std::vector<Pose2D> & path,
  std::vector<Pose2D> & expanded,
  std::vector<DebugEdge> & debug_edges)
{
  Bounds bounds;
  if (!buildBounds(bounds)) {
    return false;
  }

  if (!isStateValid(bounds, start)) {
    RCLCPP_WARN(
      get_logger(), "Start pose is outside the planning bounds or in collision: x=%.2f y=%.2f",
      start.x, start.y);
    return false;
  }
  if (!isStateValid(bounds, goal)) {
    RCLCPP_WARN(
      get_logger(), "Goal pose is outside the planning bounds or in collision: x=%.2f y=%.2f",
      goal.x, goal.y);
    return false;
  }

  const std::int64_t start_key = stateKey(bounds, start);
  if (start_key == kInvalidKey) {
    return false;
  }

  std::unordered_map<std::int64_t, SearchNode> nodes;
  std::unordered_set<std::int64_t> closed;
  nodes.reserve(static_cast<size_t>(std::min(max_iterations_ * 2, 200000)));
  closed.reserve(static_cast<size_t>(std::min(max_iterations_ * 2, 200000)));

  std::priority_queue<QueueItem> open;
  SearchNode start_node;
  start_node.pose = start;
  start_node.g = 0.0;
  start_node.h = heuristic(start, goal);
  start_node.parent_key = kInvalidKey;
  start_node.steering = 0.0;
  start_node.direction = 1;
  nodes[start_key] = start_node;
  open.push(QueueItem{start_key, heuristic_weight_ * start_node.h, 0.0});

  const std::vector<int> directions = reverse_enabled_ ? std::vector<int>{1, -1} : std::vector<int>{1};
  std::int64_t reached_key = kInvalidKey;
  bool append_exact_goal = false;
  std::vector<Pose2D> analytic_tail;
  int iterations = 0;

  while (!open.empty() && iterations < max_iterations_) {
    const QueueItem item = open.top();
    open.pop();

    auto current_it = nodes.find(item.key);
    if (current_it == nodes.end()) {
      continue;
    }
    if (closed.find(item.key) != closed.end()) {
      continue;
    }
    if (item.g > current_it->second.g + 1.0e-9) {
      continue;
    }

    const SearchNode current = current_it->second;
    closed.insert(item.key);
    ++iterations;

    if (publish_debug_markers_) {
      expanded.push_back(current.pose);
    }

    if (goalSatisfied(current.pose, goal)) {
      reached_key = item.key;
      append_exact_goal = true;
      break;
    }

    const double distance_to_goal = std::hypot(goal.x - current.pose.x, goal.y - current.pose.y);
    std::vector<Pose2D> candidate_tail;
    if (
      analytic_expansion_distance_ > 0.0 &&
      distance_to_goal <= analytic_expansion_distance_ &&
      tryAnalyticExpansion(bounds, current.pose, goal, candidate_tail))
    {
      reached_key = item.key;
      analytic_tail = std::move(candidate_tail);
      break;
    }

    for (const int direction : directions) {
      for (const double steering : steering_samples_) {
        if (!isSegmentValid(bounds, current.pose, steering, direction)) {
          continue;
        }

        const Pose2D child_pose = propagate(current.pose, steering, direction, step_size_);
        const std::int64_t child_key = stateKey(bounds, child_pose);
        if (child_key == kInvalidKey || closed.find(child_key) != closed.end()) {
          continue;
        }

        const double direction_cost = direction < 0 ? reverse_penalty_ : 1.0;
        const double steering_cost =
          steering_penalty_ * std::fabs(steering) / std::max(max_steering_angle_, 1.0e-6);
        const double steering_change_cost =
          steering_change_penalty_ * std::fabs(steering - current.steering) /
          std::max(max_steering_angle_, 1.0e-6);
        const double direction_change_cost =
          current.direction == direction ? 0.0 : direction_change_penalty_;
        const double tentative_g =
          current.g + step_size_ * direction_cost +
          steering_cost + steering_change_cost + direction_change_cost;

        auto child_it = nodes.find(child_key);
        if (child_it != nodes.end() && tentative_g >= child_it->second.g) {
          continue;
        }

        SearchNode child;
        child.pose = child_pose;
        child.g = tentative_g;
        child.h = heuristic(child_pose, goal);
        child.parent_key = item.key;
        child.steering = steering;
        child.direction = direction;
        nodes[child_key] = child;
        open.push(QueueItem{child_key, tentative_g + heuristic_weight_ * child.h, tentative_g});

        if (publish_debug_markers_ && static_cast<int>(debug_edges.size()) < max_debug_edges_) {
          debug_edges.emplace_back(current.pose, child_pose);
        }
      }
    }
  }

  if (reached_key == kInvalidKey) {
    RCLCPP_WARN(
      get_logger(), "Hybrid A* search exhausted: iterations=%d, open=%zu, closed=%zu",
      iterations, open.size(), closed.size());
    return false;
  }

  std::vector<Pose2D> coarse_path = reconstructPath(nodes, reached_key);
  if (coarse_path.empty()) {
    return false;
  }

  if (append_exact_goal) {
    const Pose2D last = coarse_path.back();
    if (std::hypot(goal.x - last.x, goal.y - last.y) > 1.0e-6 ||
      std::fabs(shortestAngularDistance(last.yaw, goal.yaw)) > 1.0e-6)
    {
      coarse_path.push_back(goal);
    }
  } else if (!analytic_tail.empty()) {
    const size_t first_tail_index = analytic_tail.size() > 1 ? 1 : 0;
    coarse_path.insert(
      coarse_path.end(), analytic_tail.begin() + static_cast<std::ptrdiff_t>(first_tail_index),
      analytic_tail.end());
  }

  path = interpolatePath(coarse_path);
  return path.size() >= 2;
}

bool HybridAStarPlanner::buildBounds(Bounds & bounds) const
{
  if (use_occupancy_grid_ && map_) {
    if (map_->info.resolution <= 0.0 || map_->info.width == 0 || map_->info.height == 0) {
      RCLCPP_WARN(get_logger(), "Occupancy grid metadata is invalid.");
      return false;
    }

    bounds.x_min = map_->info.origin.position.x;
    bounds.y_min = map_->info.origin.position.y;
    bounds.resolution = map_->info.resolution;
    bounds.width = static_cast<int>(map_->info.width);
    bounds.height = static_cast<int>(map_->info.height);
    bounds.from_map = true;
    return true;
  }

  if (map_x_max_ <= map_x_min_ || map_y_max_ <= map_y_min_) {
    RCLCPP_WARN(get_logger(), "Fallback planning bounds are invalid.");
    return false;
  }

  bounds.x_min = map_x_min_;
  bounds.y_min = map_y_min_;
  bounds.resolution = fallback_resolution_;
  bounds.width = static_cast<int>(std::ceil((map_x_max_ - map_x_min_) / fallback_resolution_));
  bounds.height = static_cast<int>(std::ceil((map_y_max_ - map_y_min_) / fallback_resolution_));
  bounds.from_map = false;
  return bounds.width > 0 && bounds.height > 0;
}

bool HybridAStarPlanner::worldToGrid(
  const Bounds & bounds, double x, double y, int & ix, int & iy) const
{
  if (x < bounds.x_min || y < bounds.y_min || x >= bounds.x_max() || y >= bounds.y_max()) {
    return false;
  }

  ix = static_cast<int>(std::floor((x - bounds.x_min) / bounds.resolution));
  iy = static_cast<int>(std::floor((y - bounds.y_min) / bounds.resolution));
  return ix >= 0 && iy >= 0 && ix < bounds.width && iy < bounds.height;
}

std::int64_t HybridAStarPlanner::stateKey(const Bounds & bounds, const Pose2D & pose) const
{
  int ix = 0;
  int iy = 0;
  if (!worldToGrid(bounds, pose.x, pose.y, ix, iy)) {
    return kInvalidKey;
  }

  const int yaw_bin = yawToBin(pose.yaw);
  return (
    (static_cast<std::int64_t>(yaw_bin) * bounds.height + iy) *
    bounds.width + ix);
}

int HybridAStarPlanner::yawToBin(double yaw) const
{
  double positive = std::fmod(normalizeAngle(yaw) + 2.0 * kPi, 2.0 * kPi);
  if (positive < 0.0) {
    positive += 2.0 * kPi;
  }
  int bin = static_cast<int>(std::floor(positive / (2.0 * kPi) * heading_bins_));
  if (bin >= heading_bins_) {
    bin = heading_bins_ - 1;
  }
  return std::max(0, bin);
}

bool HybridAStarPlanner::isStateValid(const Bounds & bounds, const Pose2D & pose) const
{
  int center_ix = 0;
  int center_iy = 0;
  if (!worldToGrid(bounds, pose.x, pose.y, center_ix, center_iy)) {
    return false;
  }

  if (
    pose.x - collision_radius_ < bounds.x_min ||
    pose.x + collision_radius_ >= bounds.x_max() ||
    pose.y - collision_radius_ < bounds.y_min ||
    pose.y + collision_radius_ >= bounds.y_max())
  {
    return false;
  }

  if (!bounds.from_map) {
    return true;
  }

  const int radius_cells = static_cast<int>(std::ceil(collision_radius_ / bounds.resolution));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double cx = static_cast<double>(dx) * bounds.resolution;
      const double cy = static_cast<double>(dy) * bounds.resolution;
      if (cx * cx + cy * cy > collision_radius_ * collision_radius_) {
        continue;
      }
      if (isCellOccupied(center_ix + dx, center_iy + dy)) {
        return false;
      }
    }
  }

  return true;
}

bool HybridAStarPlanner::isCellOccupied(int ix, int iy) const
{
  if (!map_) {
    return false;
  }
  if (ix < 0 || iy < 0 || ix >= static_cast<int>(map_->info.width) ||
    iy >= static_cast<int>(map_->info.height))
  {
    return true;
  }

  const auto index = static_cast<size_t>(iy) * map_->info.width + static_cast<size_t>(ix);
  if (index >= map_->data.size()) {
    return true;
  }

  const int value = map_->data[index];
  if (value < 0) {
    return !allow_unknown_;
  }
  return value >= occupied_threshold_;
}

bool HybridAStarPlanner::isSegmentValid(
  const Bounds & bounds, const Pose2D & start, double steering, int direction) const
{
  const int steps = std::max(1, static_cast<int>(std::ceil(step_size_ / collision_check_step_)));
  for (int i = 1; i <= steps; ++i) {
    const double distance = step_size_ * static_cast<double>(i) / static_cast<double>(steps);
    const Pose2D pose = propagate(start, steering, direction, distance);
    if (!isStateValid(bounds, pose)) {
      return false;
    }
  }
  return true;
}

bool HybridAStarPlanner::isLineValid(
  const Bounds & bounds, const Pose2D & start, const Pose2D & goal) const
{
  const double distance = std::hypot(goal.x - start.x, goal.y - start.y);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / collision_check_step_)));
  const double dyaw = shortestAngularDistance(start.yaw, goal.yaw);

  for (int i = 1; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    Pose2D pose;
    pose.x = start.x + (goal.x - start.x) * t;
    pose.y = start.y + (goal.y - start.y) * t;
    pose.yaw = normalizeAngle(start.yaw + dyaw * t);
    if (!isStateValid(bounds, pose)) {
      return false;
    }
  }
  return true;
}

bool HybridAStarPlanner::sampleLineCurve(
  const Bounds & bounds,
  const Pose2D & start,
  const Pose2D & goal,
  std::vector<Pose2D> & curve) const
{
  curve.clear();
  const double yaw_error = std::fabs(shortestAngularDistance(start.yaw, goal.yaw));
  if (yaw_error > analytic_expansion_yaw_tolerance_) {
    return false;
  }

  const double distance = std::hypot(goal.x - start.x, goal.y - start.y);
  const int steps = std::max(
    1, static_cast<int>(std::ceil(distance / analytic_curve_sample_distance_)));
  const double dyaw = shortestAngularDistance(start.yaw, goal.yaw);

  curve.reserve(static_cast<size_t>(steps + 1));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    Pose2D pose;
    pose.x = start.x + (goal.x - start.x) * t;
    pose.y = start.y + (goal.y - start.y) * t;
    pose.yaw = normalizeAngle(start.yaw + dyaw * t);
    if (!isStateValid(bounds, pose)) {
      curve.clear();
      return false;
    }
    curve.push_back(pose);
  }
  return curve.size() >= 2;
}

bool HybridAStarPlanner::tryAnalyticExpansion(
  const Bounds & bounds,
  const Pose2D & start,
  const Pose2D & goal,
  std::vector<Pose2D> & curve) const
{
  curve.clear();

  std::string type = analytic_expansion_type_;
  if (type == "auto") {
    type = reverse_enabled_ ? "reed_shepp" : "dubins";
  }
  if (type == "none") {
    return false;
  }
  if (type == "straight") {
    return sampleLineCurve(bounds, start, goal, curve);
  }

  const double auto_turning_radius =
    wheel_base_ / std::tan(std::max(max_steering_angle_, 1.0e-6));
  const double turning_radius =
    minimum_turning_radius_ > 0.0 ? minimum_turning_radius_ : auto_turning_radius;

  if (!std::isfinite(turning_radius) || turning_radius <= 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid turning radius %.3f for analytic expansion.", turning_radius);
    return false;
  }

  ompl::base::StateSpacePtr state_space;
  if (type == "reed_shepp" || type == "reeds_shepp" || type == "reeds-shepp") {
    state_space = std::make_shared<ompl::base::ReedsSheppStateSpace>(turning_radius);
  } else if (type == "dubins") {
    state_space = std::make_shared<ompl::base::DubinsStateSpace>(turning_radius, false);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Unknown analytic_expansion_type='%s'. Use none, straight, dubins, reed_shepp, or auto.",
      analytic_expansion_type_.c_str());
    return false;
  }

  ompl::base::ScopedState<> from(state_space);
  ompl::base::ScopedState<> to(state_space);
  ompl::base::ScopedState<> sampled(state_space);

  auto * from_se2 = from->as<ompl::base::SE2StateSpace::StateType>();
  auto * to_se2 = to->as<ompl::base::SE2StateSpace::StateType>();
  from_se2->setXY(start.x, start.y);
  from_se2->setYaw(start.yaw);
  to_se2->setXY(goal.x, goal.y);
  to_se2->setYaw(goal.yaw);

  const double curve_length = state_space->distance(from.get(), to.get());
  if (!std::isfinite(curve_length) || curve_length <= 0.0) {
    return false;
  }
  if (analytic_expansion_max_length_ > 0.0 && curve_length > analytic_expansion_max_length_) {
    return false;
  }

  const int steps = std::max(
    2, static_cast<int>(std::ceil(curve_length / analytic_curve_sample_distance_)));
  curve.reserve(static_cast<size_t>(steps + 1));

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    state_space->interpolate(from.get(), to.get(), t, sampled.get());
    const auto * se2 = sampled->as<ompl::base::SE2StateSpace::StateType>();

    Pose2D pose;
    pose.x = se2->getX();
    pose.y = se2->getY();
    pose.yaw = normalizeAngle(se2->getYaw());
    if (!isStateValid(bounds, pose)) {
      curve.clear();
      return false;
    }
    curve.push_back(pose);
  }

  return curve.size() >= 2;
}

HybridAStarPlanner::Pose2D HybridAStarPlanner::propagate(
  const Pose2D & pose, double steering, int direction, double distance) const
{
  Pose2D next = pose;
  const double signed_distance = direction >= 0 ? distance : -distance;
  const double curvature = std::tan(steering) / wheel_base_;

  if (std::fabs(curvature) < 1.0e-6) {
    next.x += signed_distance * std::cos(pose.yaw);
    next.y += signed_distance * std::sin(pose.yaw);
    next.yaw = normalizeAngle(pose.yaw);
    return next;
  }

  next.yaw = normalizeAngle(pose.yaw + signed_distance * curvature);
  next.x += (std::sin(next.yaw) - std::sin(pose.yaw)) / curvature;
  next.y -= (std::cos(next.yaw) - std::cos(pose.yaw)) / curvature;
  return next;
}

double HybridAStarPlanner::heuristic(const Pose2D & pose, const Pose2D & goal) const
{
  const double distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
  const double yaw_error = std::fabs(shortestAngularDistance(pose.yaw, goal.yaw));
  return distance + yaw_heuristic_weight_ * yaw_error;
}

bool HybridAStarPlanner::goalSatisfied(const Pose2D & pose, const Pose2D & goal) const
{
  return std::hypot(goal.x - pose.x, goal.y - pose.y) <= goal_xy_tolerance_ &&
    std::fabs(shortestAngularDistance(pose.yaw, goal.yaw)) <= goal_yaw_tolerance_;
}

std::vector<HybridAStarPlanner::Pose2D> HybridAStarPlanner::reconstructPath(
  const std::unordered_map<std::int64_t, SearchNode> & nodes,
  std::int64_t goal_key) const
{
  std::vector<Pose2D> reversed;
  std::int64_t key = goal_key;
  size_t guard = 0;

  while (key != kInvalidKey && guard <= nodes.size()) {
    const auto it = nodes.find(key);
    if (it == nodes.end()) {
      break;
    }
    reversed.push_back(it->second.pose);
    key = it->second.parent_key;
    ++guard;
  }

  std::reverse(reversed.begin(), reversed.end());
  return reversed;
}

std::vector<HybridAStarPlanner::Pose2D> HybridAStarPlanner::interpolatePath(
  const std::vector<Pose2D> & coarse_path) const
{
  if (coarse_path.empty()) {
    return {};
  }

  std::vector<Pose2D> path;
  path.reserve(coarse_path.size() * 3);
  path.push_back(coarse_path.front());

  for (size_t i = 1; i < coarse_path.size(); ++i) {
    const Pose2D prev = path.back();
    const Pose2D next = coarse_path[i];
    const double distance = std::hypot(next.x - prev.x, next.y - prev.y);
    const int steps = std::max(
      1, static_cast<int>(std::ceil(distance / path_interpolation_distance_)));
    const double dyaw = shortestAngularDistance(prev.yaw, next.yaw);

    for (int step = 1; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      Pose2D pose;
      pose.x = prev.x + (next.x - prev.x) * t;
      pose.y = prev.y + (next.y - prev.y) * t;
      pose.yaw = normalizeAngle(prev.yaw + dyaw * t);
      path.push_back(pose);
    }
  }

  return path;
}

void HybridAStarPlanner::publishPath(const std::vector<Pose2D> & path)
{
  nav_msgs::msg::Path msg;
  msg.header.stamp = now();
  msg.header.frame_id = global_frame_;
  msg.poses.reserve(path.size());

  for (const Pose2D & pose : path) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = msg.header;
    stamped.pose.position.x = pose.x;
    stamped.pose.position.y = pose.y;
    stamped.pose.position.z = 0.0;
    stamped.pose.orientation = yawToQuaternion(pose.yaw);
    msg.poses.push_back(stamped);
  }

  pub_path_->publish(msg);
}

void HybridAStarPlanner::publishEmptyPath()
{
  nav_msgs::msg::Path msg;
  msg.header.stamp = now();
  msg.header.frame_id = global_frame_;
  pub_path_->publish(msg);
}

void HybridAStarPlanner::publishMarkers(
  const Pose2D & start,
  const Pose2D & goal,
  const std::vector<Pose2D> & path,
  const std::vector<Pose2D> & expanded,
  const std::vector<DebugEdge> & debug_edges)
{
  if (!publish_markers_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  const auto stamp = now();
  int id = 0;
  const auto make_marker = [&](const std::string & ns, int type) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id++;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 0;
    return marker;
  };

  auto start_marker = make_marker("hybrid_astar_start", visualization_msgs::msg::Marker::SPHERE);
  start_marker.pose.position.x = start.x;
  start_marker.pose.position.y = start.y;
  start_marker.pose.position.z = 0.08;
  start_marker.scale.x = 0.35;
  start_marker.scale.y = 0.35;
  start_marker.scale.z = 0.16;
  start_marker.color.r = 0.1f;
  start_marker.color.g = 0.45f;
  start_marker.color.b = 1.0f;
  start_marker.color.a = 0.9f;
  markers.markers.push_back(start_marker);

  auto goal_marker = make_marker("hybrid_astar_goal", visualization_msgs::msg::Marker::SPHERE);
  goal_marker.pose.position.x = goal.x;
  goal_marker.pose.position.y = goal.y;
  goal_marker.pose.position.z = 0.08;
  goal_marker.scale.x = 0.35;
  goal_marker.scale.y = 0.35;
  goal_marker.scale.z = 0.16;
  goal_marker.color.r = 1.0f;
  goal_marker.color.g = 0.2f;
  goal_marker.color.b = 0.15f;
  goal_marker.color.a = 0.9f;
  markers.markers.push_back(goal_marker);

  if (!path.empty()) {
    auto path_marker = make_marker("hybrid_astar_path", visualization_msgs::msg::Marker::LINE_STRIP);
    path_marker.scale.x = 0.06;
    path_marker.color.r = 0.1f;
    path_marker.color.g = 1.0f;
    path_marker.color.b = 0.35f;
    path_marker.color.a = 0.95f;
    path_marker.points.reserve(path.size());
    for (const Pose2D & pose : path) {
      geometry_msgs::msg::Point point;
      point.x = pose.x;
      point.y = pose.y;
      point.z = 0.08;
      path_marker.points.push_back(point);
    }
    markers.markers.push_back(path_marker);

    auto forward_marker = make_marker(
      "hybrid_astar_forward_segments", visualization_msgs::msg::Marker::LINE_LIST);
    forward_marker.scale.x = 0.085;
    forward_marker.color.r = 0.05f;
    forward_marker.color.g = 1.0f;
    forward_marker.color.b = 0.25f;
    forward_marker.color.a = 0.95f;

    auto reverse_marker = make_marker(
      "hybrid_astar_reverse_segments", visualization_msgs::msg::Marker::LINE_LIST);
    reverse_marker.scale.x = 0.085;
    reverse_marker.color.r = 1.0f;
    reverse_marker.color.g = 0.48f;
    reverse_marker.color.b = 0.05f;
    reverse_marker.color.a = 0.95f;

    for (size_t i = 1; i < path.size(); ++i) {
      const Pose2D & prev = path[i - 1];
      const Pose2D & next = path[i];
      const double dx = next.x - prev.x;
      const double dy = next.y - prev.y;
      const double heading_dot = dx * std::cos(prev.yaw) + dy * std::sin(prev.yaw);

      geometry_msgs::msg::Point p0;
      p0.x = prev.x;
      p0.y = prev.y;
      p0.z = 0.11;
      geometry_msgs::msg::Point p1;
      p1.x = next.x;
      p1.y = next.y;
      p1.z = 0.11;

      auto & points = heading_dot >= 0.0 ? forward_marker.points : reverse_marker.points;
      points.push_back(p0);
      points.push_back(p1);
    }

    if (!forward_marker.points.empty()) {
      markers.markers.push_back(forward_marker);
    }
    if (!reverse_marker.points.empty()) {
      markers.markers.push_back(reverse_marker);
    }
  }

  if (publish_debug_markers_ && !debug_edges.empty() && max_debug_edges_ > 0) {
    auto tree_marker = make_marker("hybrid_astar_tree", visualization_msgs::msg::Marker::LINE_LIST);
    tree_marker.scale.x = 0.015;
    tree_marker.color.r = 0.55f;
    tree_marker.color.g = 0.6f;
    tree_marker.color.b = 0.65f;
    tree_marker.color.a = 0.22f;
    for (const auto & edge : debug_edges) {
      geometry_msgs::msg::Point p0;
      p0.x = edge.first.x;
      p0.y = edge.first.y;
      p0.z = 0.03;
      geometry_msgs::msg::Point p1;
      p1.x = edge.second.x;
      p1.y = edge.second.y;
      p1.z = 0.03;
      tree_marker.points.push_back(p0);
      tree_marker.points.push_back(p1);
    }
    markers.markers.push_back(tree_marker);
  }

  if (publish_debug_markers_ && !expanded.empty() && max_debug_points_ > 0) {
    auto expanded_marker = make_marker(
      "hybrid_astar_expanded", visualization_msgs::msg::Marker::POINTS);
    expanded_marker.scale.x = 0.035;
    expanded_marker.scale.y = 0.035;
    expanded_marker.color.r = 0.0f;
    expanded_marker.color.g = 0.85f;
    expanded_marker.color.b = 1.0f;
    expanded_marker.color.a = 0.45f;

    const size_t stride = std::max<size_t>(
      1, static_cast<size_t>(
        std::ceil(static_cast<double>(expanded.size()) / static_cast<double>(max_debug_points_))));
    for (size_t i = 0; i < expanded.size(); i += stride) {
      geometry_msgs::msg::Point point;
      point.x = expanded[i].x;
      point.y = expanded[i].y;
      point.z = 0.02;
      expanded_marker.points.push_back(point);
    }
    markers.markers.push_back(expanded_marker);
  }

  pub_markers_->publish(markers);
}

std::vector<double> HybridAStarPlanner::buildSteeringSamples() const
{
  std::vector<double> samples;
  if (!configured_steering_angles_.empty()) {
    samples = configured_steering_angles_;
    for (double & sample : samples) {
      sample = std::clamp(sample, -max_steering_angle_, max_steering_angle_);
    }
  } else if (steering_sample_count_ == 1) {
    samples.push_back(0.0);
  } else {
    int count = steering_sample_count_;
    if (count % 2 == 0) {
      ++count;
    }
    samples.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
      const double ratio = -1.0 + 2.0 * static_cast<double>(i) / static_cast<double>(count - 1);
      samples.push_back(ratio * max_steering_angle_);
    }
  }

  samples.push_back(0.0);
  std::sort(samples.begin(), samples.end());
  samples.erase(
    std::unique(
      samples.begin(), samples.end(),
      [](double a, double b) { return std::fabs(a - b) < 1.0e-6; }),
    samples.end());
  return samples;
}

geometry_msgs::msg::Quaternion HybridAStarPlanner::yawToQuaternion(double yaw) const
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double HybridAStarPlanner::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return normalizeAngle(std::atan2(siny_cosp, cosy_cosp));
}

double HybridAStarPlanner::normalizeAngle(double angle) const
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double HybridAStarPlanner::shortestAngularDistance(double from, double to) const
{
  return normalizeAngle(to - from);
}

}  // namespace hybrid_a_star_planner_node
