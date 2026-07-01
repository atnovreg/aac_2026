#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>
#include <vector>

class TrackPathPublisher : public rclcpp::Node
{
public:
  explicit TrackPathPublisher(const std::string & node_name);

private:
  struct Pt { double x, y; };

  void loadAndPublish();

  std::string resolveCsvPath(const std::string & configured_path) const;

  std::vector<Pt> parseCsvColumns(
    const std::string & csv_path,
    const std::string & col_x_mm,
    const std::string & col_y_mm) const;

  std::vector<Pt> resample(
    const std::vector<Pt> & pts, double spacing_m) const;

  nav_msgs::msg::Path toPathMsg(const std::vector<Pt> & pts) const;

  Pt mmToWorld(double x_mm, double y_mm) const;

  // coordinate transform (mirrors image_match_node map params)
  double map_x_min_, map_x_max_;
  double map_y_min_, map_y_max_;
  bool   map_swap_xy_, map_flip_x_, map_flip_y_;
  double scale_width_mm_, scale_height_mm_;

  void timerCallback();

  std::string csv_path_;
  double      waypoint_sampling_dist_;
  double      publish_rate_;
  std::string frame_id_;

  // Paths built once at startup, republished every timer tick
  nav_msgs::msg::Path path_left_;
  nav_msgs::msg::Path path_center_;
  nav_msgs::msg::Path path_right_;

  rclcpp::TimerBase::SharedPtr                      timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_left_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_center_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_right_;
};
