#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanFrameRelay : public rclcpp::Node
{
public:
  ScanFrameRelay()
  : Node("scan_frame_relay")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/scan_lidar");
    output_frame_ = declare_parameter<std::string>(
      "output_frame", "lidar_link_lidar");

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanFrameRelay::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Relaying %s to %s with frame_id '%s'",
      input_topic_.c_str(), output_topic_.c_str(), output_frame_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    auto relayed_scan = *message;
    relayed_scan.header.frame_id = output_frame_;
    publisher_->publish(relayed_scan);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFrameRelay>());
  rclcpp::shutdown();
  return 0;
}
