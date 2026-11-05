#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <algorithm>
#include <cstring>
#include <map>
#include "timestamp.hpp"

template<class T> T read(const uint8_t* p) { T value; std::memcpy(&value, p, sizeof(T)); return value; }
class Adapter : public rclcpp::Node {
 public:
  Adapter() : Node("pointcloud_to_livox") {
    pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/lidar_custom", 100);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar", 100,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { convert(*msg); });
  }
 private:
  void convert(const sensor_msgs::msg::PointCloud2& msg) {
    try {
      using F = sensor_msgs::msg::PointField;
      const std::map<std::string, std::pair<uint32_t, uint8_t>> expected = {
        {"x",{0,F::FLOAT32}}, {"y",{4,F::FLOAT32}}, {"z",{8,F::FLOAT32}},
        {"intensity",{12,F::FLOAT32}}, {"tag",{16,F::UINT8}},
        {"line",{17,F::UINT8}}, {"timestamp",{18,F::FLOAT64}}};
      if (msg.is_bigendian || msg.point_step != 26 || msg.fields.size() != expected.size() ||
          msg.row_step < msg.width * uint64_t(msg.point_step) ||
          msg.data.size() != uint64_t(msg.row_step) * msg.height)
        throw std::invalid_argument("Unsupported PointCloud2 layout; expected recorded little-endian Livox 26-byte points");
      for (const auto& [name, spec] : expected) {
        const auto it = std::find_if(msg.fields.begin(), msg.fields.end(), [&](const auto& f){return f.name == name;});
        if (it == msg.fields.end() || it->offset != spec.first || it->datatype != spec.second || it->count != 1)
          throw std::invalid_argument("Missing or invalid PointCloud2 field: " + name);
      }
      livox_ros_driver2::msg::CustomMsg out;
      out.header = msg.header;
      const int64_t base = int64_t(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec;
      if (base < 0) throw std::invalid_argument("Negative timestamp");
      out.timebase = base;
      out.points.reserve(uint64_t(msg.width) * msg.height);
      for (uint32_t row=0; row<msg.height; ++row) for(uint32_t col=0; col<msg.width; ++col) {
        const auto* p = msg.data.data() + uint64_t(row)*msg.row_step + uint64_t(col)*msg.point_step;
        livox_ros_driver2::msg::CustomPoint point;
        point.offset_time = offset_ns(read<double>(p+18), base);
        point.x=read<float>(p); point.y=read<float>(p+4); point.z=read<float>(p+8);
        const float intensity=read<float>(p+12);
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) || !std::isfinite(intensity)) continue;
        point.reflectivity=static_cast<uint8_t>(std::clamp(intensity,0.0f,255.0f));
        point.tag=p[16]; point.line=p[17];
        if (point.line >= 4) throw std::invalid_argument("Expected MID-360 line in [0,3]");
        out.points.push_back(point);
      }
      std::stable_sort(out.points.begin(), out.points.end(), [](const auto& a,const auto& b){return a.offset_time < b.offset_time;});
      out.point_num=out.points.size();
      if (out.point_num) pub_->publish(out);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "%s", e.what());
      throw; // Fail the run instead of silently losing scans.
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<Adapter>());} catch(const std::exception&) {rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}
