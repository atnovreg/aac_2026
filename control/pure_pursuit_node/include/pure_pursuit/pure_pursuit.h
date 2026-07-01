#pragma once

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <string>
#include <vector>

class PurePursuit : public rclcpp::Node
{
public:
  explicit PurePursuit(const std::string & node_name);

private:
  struct CurvatureSample
  {
    geometry_msgs::msg::Point point;
    double curvature;
    size_t index;
  };

  void getParams();
  void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  bool makeCommand(ackermann_msgs::msg::AckermannDriveStamped & cmd);
  bool selectTargetPoint(geometry_msgs::msg::Point & target, size_t & target_index) const;
  double computeSteeringAngle(const geometry_msgs::msg::Point & target) const;
  double computeCurvatureLimitedSpeed() const;
  std::vector<CurvatureSample> collectCurvatureSamples() const;
  bool canUseGlobalVisualization() const;
  bool getGlobalVisualizationPoint(size_t index, geometry_msgs::msg::Point & point) const;
  std::string visualizationFrameId(const std::string & fallback_frame) const;
  double pointCurvature(
    const geometry_msgs::msg::PoseStamped & prev,
    const geometry_msgs::msg::PoseStamped & current,
    const geometry_msgs::msg::PoseStamped & next) const;
  double poseDistance(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to) const;
  void publishStop(const std::string & frame_id);
  void publishTargetMarker(
    const geometry_msgs::msg::Point & target,
    size_t target_index,
    const std::string & frame_id) const;
  void publishCurvatureMarkers(const std::string & frame_id) const;
  void clearVisualization(const std::string & frame_id) const;

  template<typename T>
  T clampValue(const T value, const T low, const T high) const
  {
    return std::max(low, std::min(value, high));
  }

  double wheel_base_;
  double lookahead_distance_;
  double max_steering_angle_;
  double max_speed_;
  double min_speed_;
  double lateral_accel_limit_;
  double curvature_epsilon_;
  double curvature_preview_distance_;
  double target_marker_scale_;
  double curvature_marker_scale_;
  double curvature_marker_height_;
  double max_visualized_curvature_;
  double min_forward_x_;
  int curvature_preview_points_;
  bool visualize_markers_;
  std::string local_path_topic_;
  std::string global_path_topic_;
  std::string command_topic_;
  std::string command_frame_;
  std::string target_marker_topic_;
  std::string curvature_marker_topic_;

  nav_msgs::msg::Path local_path_;
  nav_msgs::msg::Path global_path_;
  bool has_global_path_{false};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_local_path_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_command_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_target_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_curvature_markers_;
};
