#include "parking_lines_publisher/parking_lines_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParkingLinesPublisher>("parking_lines_publisher");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
