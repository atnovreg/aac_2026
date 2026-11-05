#include <limits>
#include <stdexcept>
#include "front_filter.hpp"

void require(bool condition) {
  if (!condition) throw std::runtime_error("front filter contract failed");
}

int main() {
  livox_ros_driver2::msg::CustomMsg source;
  source.header.frame_id = "livox_frame";
  source.header.stamp.sec = 42;
  source.timebase = 123456789;
  source.lidar_id = 3;
  source.rsvd = {1, 2, 3};
  livox_ros_driver2::msg::CustomPoint point;
  point.x = 1; point.y = 2; point.z = 0.25;
  point.offset_time = 123; point.reflectivity = 37; point.tag = 16; point.line = 2;
  source.points.push_back(point);
  point.x = -1; source.points.push_back(point);
  point.x = 0; source.points.push_back(point);
  point.y = 0; point.z = 0; source.points.push_back(point);
  point.x = std::numeric_limits<float>::quiet_NaN(); source.points.push_back(point);
  point.x = 1; point.y = std::numeric_limits<float>::infinity(); source.points.push_back(point);
  source.point_num = source.points.size();
  const auto before = source.points.size();
  const auto out = filter_front(source, 0.0);
  require(out.point_num == 1 && out.points.size() == 1);
  require(out.points[0] == source.points[0]);
  require(out.header == source.header && out.timebase == source.timebase);
  require(out.lidar_id == source.lidar_id && out.rsvd == source.rsvd);
  const auto view = to_pointcloud2(out);
  require(view.header == out.header && view.width == 1 && view.height == 1);
  sensor_msgs::PointCloud2ConstIterator<float> view_x(view, "x");
  sensor_msgs::PointCloud2ConstIterator<float> view_y(view, "y");
  sensor_msgs::PointCloud2ConstIterator<float> view_z(view, "z");
  require(*view_x == 1.0f && *view_y == 2.0f && *view_z == 0.25f);
  require(source.points.size() == before && source.point_num == before);
  livox_ros_driver2::msg::CustomMsg cardinal;
  point = livox_ros_driver2::msg::CustomPoint();
  point.y = 1; cardinal.points.push_back(point);
  point.y = -1; cardinal.points.push_back(point);
  point.y = 0; point.x = 1; cardinal.points.push_back(point);
  cardinal.point_num = 3;
  const auto rotated = filter_front(cardinal, 90.0);
  require(rotated.point_num == 1 && rotated.points[0] == cardinal.points[0]);
  require(filter_front(livox_ros_driver2::msg::CustomMsg(), 0.0).point_num == 0);
  require(to_pointcloud2(livox_ros_driver2::msg::CustomMsg()).width == 0);
}
