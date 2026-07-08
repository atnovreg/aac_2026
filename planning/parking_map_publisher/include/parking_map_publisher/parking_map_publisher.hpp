#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

class ParkingMapPublisher : public rclcpp::Node
{
public:
  explicit ParkingMapPublisher(const std::string & node_name);

private:
  struct Point2D
  {
    double x{0.0};
    double y{0.0};
  };

  struct ParkingLine
  {
    int line_id{0};
    std::string line_role;
    Point2D p1;
    Point2D p2;
  };

  std::string resolveTrackCsvPath(const std::string & configured_path) const;
  std::string resolveParkingCsvPath(const std::string & configured_path) const;
  void loadInputs();
  void buildMap();
  void publishMap();

  std::vector<Point2D> loadTrackColumnPair(
    const std::string & csv_path,
    const std::string & x_column,
    const std::string & y_column) const;
  std::vector<ParkingLine> loadParkingLines(const std::string & csv_path) const;
  Point2D mmToWorld(double x_mm, double y_mm) const;

  static std::vector<std::string> splitCsvLine(const std::string & line);
  static std::string trim(const std::string & value);
  static size_t findColumnIndex(
    const std::vector<std::string> & header,
    const std::string & column_name);

  static bool pointInPolygon(const std::vector<Point2D> & polygon, const Point2D & point);
  static std::vector<Point2D> convexHull(std::vector<Point2D> points);
  static double cross(const Point2D & origin, const Point2D & a, const Point2D & b);
  static double distanceToSegment(const Point2D & point, const Point2D & a, const Point2D & b);

  bool isIgnoredParkingLineRole(const std::string & role) const;
  bool worldToGrid(double x, double y, int & ix, int & iy) const;
  Point2D gridToWorld(int ix, int iy) const;
  void setCell(int ix, int iy, int value);
  void drawLineObstacle(const Point2D & a, const Point2D & b, double width_m);
  void drawLineValue(const Point2D & a, const Point2D & b, double width_m, int value);

  std::string track_csv_path_;
  std::string parking_csv_path_;
  std::string map_topic_;
  std::string frame_id_;

  double publish_rate_{1.0};
  double map_x_min_{-5.5};
  double map_x_max_{5.5};
  double map_y_min_{-8.0};
  double map_y_max_{8.0};
  double resolution_{0.05};
  bool map_swap_xy_{true};
  bool map_flip_x_{true};
  bool map_flip_y_{true};
  double scale_width_mm_{16000.0};
  double scale_height_mm_{11000.0};
  int free_value_{0};
  int occupied_value_{100};
  int unknown_value_{-1};
  bool unknown_outside_drivable_{false};
  double parking_hull_margin_{0.35};
  double parking_line_obstacle_width_{0.10};
  double road_boundary_obstacle_width_{0.08};
  bool mark_inner_road_boundary_obstacle_{false};
  bool mark_outer_road_boundary_obstacle_{true};
  bool log_map_summary_{true};
  std::vector<std::string> ignored_parking_line_roles_;

  std::vector<Point2D> road_inner_;
  std::vector<Point2D> road_outer_;
  std::vector<ParkingLine> parking_lines_;
  std::vector<Point2D> parking_hull_;

  nav_msgs::msg::OccupancyGrid map_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
};
