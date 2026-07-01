#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>
#include <limits>

class LocalPathPlanner : public rclcpp::Node
{
public:
  explicit LocalPathPlanner(const std::string & node_name);

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGlobalPath(const nav_msgs::msg::Path::SharedPtr msg);
  void timerCallback();

  // Returns the closest waypoint index using distance plus optional heading error.
  size_t findClosestIndex(double rx, double ry, double ryaw) const;

  // Extracts lookahead_dist_ metres of path from idx onwards (wraps for loops).
  // Fills both the global-frame and body-frame Path messages.
  void extractLocalPath(
    size_t start_idx, double rx, double ry, double ryaw,
    const std::string & global_frame, const std::string & body_frame,
    nav_msgs::msg::Path & out_global,
    nav_msgs::msg::Path & out_body) const;

  // Parameters
  double      lookahead_dist_;
  double      heading_error_weight_;
  double      update_rate_;
  std::string global_frame_;
  std::string body_frame_;
  std::string odom_topic_;
  std::string global_path_topic_;
  std::string local_global_topic_;
  std::string local_body_topic_;

  // State
  nav_msgs::msg::Odometry current_odom_;
  nav_msgs::msg::Path global_path_;
  bool                has_odom_{false};
  bool                has_path_{false};
  size_t              last_nearest_idx_{0};

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr    pub_local_global_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr    pub_local_body_;
};
