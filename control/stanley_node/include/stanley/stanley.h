#ifndef STANLEY_H
#define STANLEY_H

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cstddef>
#include <string>

class Stanley : public rclcpp::Node
{
public:
  explicit Stanley(const std::string & node_name);

private:
  struct PidController
  {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double integral_limit{0.0};
    double integral{0.0};
    double previous_error{0.0};
    bool has_previous{false};

    void reset();
    double update(double error, double dt);
  };

  struct TrackingError
  {
    double cross_track_error{0.0};
    double heading_error{0.0};
    double path_yaw{0.0};
    double distance{0.0};
    size_t segment_index{0};
    geometry_msgs::msg::Point projection;
    geometry_msgs::msg::Point segment_start;
    geometry_msgs::msg::Point segment_end;
  };

  void getParams();
  void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  bool makeCommand(
    const nav_msgs::msg::Path & path,
    ackermann_msgs::msg::AckermannDriveStamped & cmd,
    TrackingError & error);
  bool computeTrackingError(const nav_msgs::msg::Path & path, TrackingError & error) const;
  double computeCommandSpeed(const TrackingError & error) const;
  double computeControlDt();
  void resetControlState();
  void publishStop(const std::string & frame_id);
  void publishErrorMarkers(const TrackingError & error, const std::string & frame_id) const;
  void clearErrorMarkers(const std::string & frame_id) const;

  static double normalizeAngle(double angle);

  template<typename T>
  T clampValue(const T value, const T low, const T high) const
  {
    return std::max(low, std::min(value, high));
  }

  std::string local_path_topic_;
  std::string command_topic_;
  std::string command_frame_;
  std::string error_marker_topic_;

  double stanley_gain_;
  double softening_speed_;
  double max_steering_angle_;
  double target_speed_;
  double max_speed_;
  double min_speed_;
  double cte_speed_reduction_gain_;
  double heading_speed_reduction_gain_;
  double default_dt_;
  double max_dt_;
  double min_segment_length_;
  double min_segment_forward_x_;
  double marker_scale_;
  double marker_height_;
  bool visualize_markers_;

  PidController cross_track_pid_;
  PidController heading_pid_;

  rclcpp::Time last_control_time_;
  bool has_last_control_time_{false};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_local_path_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_command_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_error_markers_;
};

#endif  // STANLEY_H
