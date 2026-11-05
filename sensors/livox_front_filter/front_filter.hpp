#pragma once
#include <cmath>
#include <stdexcept>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// yaw describes vehicle forward in the input frame, from +X toward +Y.
// Keep point coordinates, ordering, and acquisition timestamps unchanged.
inline livox_ros_driver2::msg::CustomMsg filter_front(
    const livox_ros_driver2::msg::CustomMsg &source, double yaw_degrees) {
  if (!std::isfinite(yaw_degrees)) throw std::invalid_argument("nonfinite yaw");
  const double yaw = yaw_degrees * std::acos(-1.0) / 180.0;
  double fx = std::cos(yaw), fy = std::sin(yaw);
  if (std::abs(fx) < 1e-12) fx = 0.0;
  if (std::abs(fy) < 1e-12) fy = 0.0;
  livox_ros_driver2::msg::CustomMsg out;
  out.header = source.header;
  out.timebase = source.timebase;
  out.lidar_id = source.lidar_id;
  out.rsvd = source.rsvd;
  out.points.reserve(source.points.size());
  for (const auto &point : source.points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)
        && point.x * fx + point.y * fy > 0.0) {
      out.points.push_back(point);
    }
  }
  out.point_num = static_cast<uint32_t>(out.points.size());
  return out;
}

// RViz-only XYZ view; the recorded CustomMsg retains all Livox attributes.
inline sensor_msgs::msg::PointCloud2 to_pointcloud2(
    const livox_ros_driver2::msg::CustomMsg &source) {
  sensor_msgs::msg::PointCloud2 out;
  out.header = source.header;
  out.height = 1;
  out.is_bigendian = false;
  out.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(out);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(source.points.size());
  sensor_msgs::PointCloud2Iterator<float> x(out, "x"), y(out, "y"), z(out, "z");
  for (const auto &point : source.points) {
    *x = point.x; *y = point.y; *z = point.z;
    ++x; ++y; ++z;
  }
  return out;
}
