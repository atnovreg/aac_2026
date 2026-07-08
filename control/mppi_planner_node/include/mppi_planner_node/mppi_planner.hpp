#pragma once

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cstddef>
#include <random>
#include <string>
#include <vector>

class MppiPlanner : public rclcpp::Node
{
public:
  explicit MppiPlanner(const std::string & node_name);

private:
  struct Control
  {
    double speed{0.0};
    double steering{0.0};
  };

  struct State
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct PathSegment
  {
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point end;
    double yaw{0.0};
    double length{0.0};
    double cumulative_s{0.0};
  };

  struct PathProjection
  {
    bool valid{false};
    geometry_msgs::msg::Point point;
    double distance{0.0};
    double heading{0.0};
    double progress{0.0};
  };

  struct RolloutVisualization
  {
    std::vector<State> states;
    double cost{0.0};
  };

  void getParams();
  rcl_interfaces::msg::SetParametersResult onParamsChanged(
    const std::vector<rclcpp::Parameter> & parameters);
  void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void timerCallback();

  bool computeCommand(
    const nav_msgs::msg::Path & path,
    ackermann_msgs::msg::AckermannDriveStamped & cmd,
    nav_msgs::msg::Path & trajectory_path);
  bool buildPathSegments(
    const nav_msgs::msg::Path & path,
    std::vector<PathSegment> & segments) const;
  double rolloutTrajectory(
    const std::vector<PathSegment> & segments,
    const std::vector<Control> * noise,
    std::vector<State> * trajectory) const;
  PathProjection projectToPath(
    const std::vector<PathSegment> & segments,
    double x,
    double y) const;
  PathProjection samplePathAtProgress(
    const std::vector<PathSegment> & segments,
    double progress) const;
  double cornerSpeedFromHeadingChange(
    double heading_change,
    double lookahead_distance) const;
  void updatePathDiagnostics(const std::vector<PathSegment> & segments);
  Control boundedControl(const Control & desired, const Control & previous) const;
  State simulateStep(const State & state, const Control & control) const;

  void ensureControlSequence();
  void resetControlSequence();
  void shiftControlSequence();
  nav_msgs::msg::Path statesToPath(
    const std::vector<State> & states,
    const std::string & frame_id) const;
  void publishStop(const std::string & frame_id);
  void publishMarkers(const nav_msgs::msg::Path & trajectory_path) const;
  void clearMarkers(const std::string & frame_id) const;
  void publishDebugStatus(
    const ackermann_msgs::msg::AckermannDriveStamped & cmd,
    const nav_msgs::msg::Path & trajectory_path) const;

  static double normalizeAngle(double angle);
  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

  template<typename T>
  T clampValue(const T value, const T low, const T high) const
  {
    return std::max(low, std::min(value, high));
  }

  std::string local_path_topic_;
  std::string command_topic_;
  std::string command_frame_;
  std::string trajectory_topic_;
  std::string marker_topic_;
  std::string debug_topic_;
  std::string tuning_profile_;

  double update_rate_;
  double path_timeout_;
  double wheel_base_;
  double time_step_;
  int horizon_steps_;
  int batch_size_;
  int iteration_count_;
  int random_seed_;

  double target_speed_;
  double min_speed_;
  double tracking_min_command_speed_;
  double max_speed_;
  double max_steering_angle_;
  double max_accel_;
  double max_steering_rate_;
  double speed_noise_std_;
  double steering_noise_std_;
  double temperature_;

  double path_distance_weight_;
  double heading_weight_;
  double heading_lookahead_distance_;
  double heading_lookahead_weight_;
  double steering_feedforward_lookahead_distance_;
  double steering_feedforward_weight_;
  double steering_feedforward_command_gain_;
  double curvature_lookahead_distance_;
  double curvature_speed_weight_;
  double curvature_speed_gain_;
  double curvature_min_speed_;
  double lateral_accel_limit_;
  bool use_lateral_accel_speed_limit_;
  bool use_corner_speed_as_speed_target_;
  bool limit_command_speed_to_corner_;
  double corner_speed_margin_;
  double terminal_distance_weight_;
  double terminal_heading_weight_;
  double speed_weight_;
  double steering_weight_;
  double steering_rate_weight_;
  double accel_weight_;
  double progress_weight_;

  double min_path_segment_length_;
  double min_path_forward_x_;
  int min_path_points_;
  bool visualize_markers_;
  bool visualize_rollouts_;
  bool publish_debug_status_;
  int max_visualized_rollouts_;
  int rollout_visualization_stride_;
  double marker_line_width_;
  double rollout_line_width_;
  double rollout_alpha_;
  double marker_height_;

  nav_msgs::msg::Path latest_path_;
  bool has_path_{false};
  rclcpp::Time last_path_time_;

  std::vector<Control> nominal_controls_;
  std::vector<RolloutVisualization> sampled_rollouts_;
  RolloutVisualization best_sampled_rollout_;
  bool has_best_sampled_rollout_{false};
  double last_selected_cost_{0.0};
  double last_best_sampled_rollout_cost_{0.0};
  double last_best_rollout_cost_{0.0};
  double last_worst_rollout_cost_{0.0};
  double last_path_distance_{0.0};
  double last_path_progress_{0.0};
  double last_path_heading_{0.0};
  double last_heading_lookahead_error_{0.0};
  double last_heading_lookahead_actual_distance_{0.0};
  double last_curvature_heading_change_{0.0};
  double last_curvature_heading_change_signed_{0.0};
  double last_curvature_lookahead_actual_distance_{0.0};
  double last_steering_target_{0.0};
  double last_corner_speed_target_{0.0};
  double last_effective_speed_target_{0.0};
  double last_lateral_accel_speed_target_{0.0};
  double last_tracking_speed_floor_{0.0};
  double last_command_speed_before_limit_{0.0};
  double last_command_steering_{0.0};
  bool last_command_speed_limited_{false};
  bool last_command_speed_floor_applied_{false};
  bool has_path_diagnostics_{false};
  bool has_heading_lookahead_point_{false};
  bool has_curvature_lookahead_point_{false};
  geometry_msgs::msg::Point last_projection_point_;
  geometry_msgs::msg::Point last_heading_lookahead_point_;
  geometry_msgs::msg::Point last_curvature_lookahead_point_;
  Control previous_command_;
  bool has_previous_command_{false};

  mutable std::mt19937 rng_;
  mutable std::normal_distribution<double> normal_distribution_{0.0, 1.0};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_local_path_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_command_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_status_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};
