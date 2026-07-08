#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

namespace hybrid_a_star_planner_node
{

class HybridAStarPlanner : public rclcpp::Node
{
public:
  explicit HybridAStarPlanner(const std::string & node_name);

private:
  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct Bounds
  {
    double x_min{0.0};
    double y_min{0.0};
    double resolution{0.2};
    int width{0};
    int height{0};
    bool from_map{false};

    double x_max() const { return x_min + static_cast<double>(width) * resolution; }
    double y_max() const { return y_min + static_cast<double>(height) * resolution; }
  };

  struct SearchNode
  {
    Pose2D pose;
    double g{0.0};
    double h{0.0};
    std::int64_t parent_key{-1};
    double steering{0.0};
    int direction{1};
  };

  struct QueueItem
  {
    std::int64_t key{-1};
    double f{0.0};
    double g{0.0};

    bool operator<(const QueueItem & other) const
    {
      return f > other.f;
    }
  };

  using DebugEdge = std::pair<Pose2D, Pose2D>;

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  bool resolveStartPose(Pose2D & start_pose, std::string & source_name);
  bool poseStampedToGlobal(const geometry_msgs::msg::PoseStamped & input, Pose2D & pose);
  bool poseWithCovarianceToGlobal(
    const geometry_msgs::msg::PoseWithCovarianceStamped & input, Pose2D & pose);

  bool makePlan(
    const Pose2D & start,
    const Pose2D & goal,
    std::vector<Pose2D> & path,
    std::vector<Pose2D> & expanded,
    std::vector<DebugEdge> & debug_edges);

  bool buildBounds(Bounds & bounds) const;
  bool worldToGrid(const Bounds & bounds, double x, double y, int & ix, int & iy) const;
  std::int64_t stateKey(const Bounds & bounds, const Pose2D & pose) const;
  int yawToBin(double yaw) const;
  bool isStateValid(const Bounds & bounds, const Pose2D & pose) const;
  bool isCellOccupied(int ix, int iy) const;
  bool isSegmentValid(
    const Bounds & bounds, const Pose2D & start, double steering, int direction) const;
  bool isLineValid(const Bounds & bounds, const Pose2D & start, const Pose2D & goal) const;
  bool tryAnalyticExpansion(
    const Bounds & bounds,
    const Pose2D & start,
    const Pose2D & goal,
    std::vector<Pose2D> & curve) const;
  bool sampleLineCurve(
    const Bounds & bounds,
    const Pose2D & start,
    const Pose2D & goal,
    std::vector<Pose2D> & curve) const;

  Pose2D propagate(const Pose2D & pose, double steering, int direction, double distance) const;
  double heuristic(const Pose2D & pose, const Pose2D & goal) const;
  bool goalSatisfied(const Pose2D & pose, const Pose2D & goal) const;
  std::vector<Pose2D> reconstructPath(
    const std::unordered_map<std::int64_t, SearchNode> & nodes,
    std::int64_t goal_key) const;
  std::vector<Pose2D> interpolatePath(const std::vector<Pose2D> & coarse_path) const;

  void publishPath(const std::vector<Pose2D> & path);
  void publishEmptyPath();
  void publishMarkers(
    const Pose2D & start,
    const Pose2D & goal,
    const std::vector<Pose2D> & path,
    const std::vector<Pose2D> & expanded,
    const std::vector<DebugEdge> & debug_edges);

  std::vector<double> buildSteeringSamples() const;
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) const;
  double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const;
  double normalizeAngle(double angle) const;
  double shortestAngularDistance(double from, double to) const;

  std::string global_frame_;
  std::string base_frame_;
  std::string start_pose_source_;
  std::string goal_topic_;
  std::string initial_pose_topic_;
  std::string map_topic_;
  std::string path_topic_;
  std::string marker_topic_;
  std::string analytic_expansion_type_;

  bool use_occupancy_grid_{true};
  bool allow_unknown_{true};
  bool reverse_enabled_{true};
  bool publish_markers_{true};
  bool publish_debug_markers_{true};

  int occupied_threshold_{50};
  int heading_bins_{72};
  int steering_sample_count_{7};
  int max_iterations_{50000};
  int max_debug_points_{3000};
  int max_debug_edges_{3000};

  double transform_timeout_{0.2};
  double map_x_min_{-5.5};
  double map_x_max_{5.5};
  double map_y_min_{-8.0};
  double map_y_max_{8.0};
  double fallback_resolution_{0.2};
  double step_size_{0.35};
  double wheel_base_{0.8};
  double max_steering_angle_{0.52};
  double collision_radius_{0.35};
  double collision_check_step_{0.08};
  double goal_xy_tolerance_{0.35};
  double goal_yaw_tolerance_{0.45};
  double analytic_expansion_distance_{0.8};
  double analytic_expansion_yaw_tolerance_{0.7};
  double analytic_expansion_max_length_{8.0};
  double analytic_curve_sample_distance_{0.08};
  double minimum_turning_radius_{0.0};
  double path_interpolation_distance_{0.1};
  double reverse_penalty_{2.0};
  double direction_change_penalty_{1.0};
  double steering_penalty_{0.05};
  double steering_change_penalty_{0.15};
  double heuristic_weight_{1.15};
  double yaw_heuristic_weight_{0.15};

  std::vector<double> configured_steering_angles_;
  std::vector<double> steering_samples_;

  std::optional<Pose2D> latest_initial_pose_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
};

}  // namespace hybrid_a_star_planner_node
