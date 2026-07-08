#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <string>
#include <vector>

class ParkingLinesPublisher : public rclcpp::Node
{
public:
  explicit ParkingLinesPublisher(const std::string & node_name);

private:
  struct LineSegment
  {
    int line_id{0};
    std::string line_type;
    std::string line_role;
    std::string orientation;
    double x1{0.0};
    double y1{0.0};
    double x2{0.0};
    double y2{0.0};
    double length{0.0};
  };

  struct Color
  {
    float r{1.0f};
    float g{1.0f};
    float b{1.0f};
    float a{1.0f};
  };

  void loadCsv();
  std::string resolveCsvPath(const std::string & configured_path) const;
  void timerCallback();
  visualization_msgs::msg::MarkerArray buildMarkers() const;

  Color colorForRole(const std::string & role) const;
  std::string namespaceForRole(const std::string & role) const;

  std::string csv_path_;
  std::string marker_topic_;
  std::string frame_id_;
  double publish_rate_{1.0};
  double line_width_{0.06};
  double z_offset_{0.08};
  bool publish_labels_{false};

  std::vector<LineSegment> lines_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
};
