#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "front_filter.hpp"

class FrontFilter : public rclcpp::Node {
 public:
  FrontFilter() : Node("livox_front_filter") {
    yaw_ = declare_parameter<double>("front_yaw_deg", 0.0);
    if (!std::isfinite(yaw_)) throw std::invalid_argument("nonfinite yaw");
    publisher_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
        "/livox/lidar_front", rclcpp::QoS(10).reliable());
    view_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar_front_points", rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "/livox/lidar", rclcpp::QoS(10).reliable(),
        [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr source) {
          auto out = filter_front(*source, yaw_);
          ++frames_;
          input_points_ += source->points.size();
          output_points_ += out.points.size();
          publisher_->publish(out);
          if (view_publisher_->get_subscription_count() > 0) {
            view_publisher_->publish(to_pointcloud2(out));
          }
        });
    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      RCLCPP_INFO(get_logger(), "frames=%llu points=%llu->%llu",
                  static_cast<unsigned long long>(frames_),
                  static_cast<unsigned long long>(input_points_),
                  static_cast<unsigned long long>(output_points_));
    });
    RCLCPP_INFO(get_logger(), "Front 180 degrees, yaw=%.1f deg", yaw_);
  }

 private:
  double yaw_;
  uint64_t frames_ = 0, input_points_ = 0, output_points_ = 0;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr view_publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontFilter>());
  rclcpp::shutdown();
  return 0;
}
