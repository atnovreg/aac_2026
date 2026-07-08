#include "parking_map_publisher/parking_map_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParkingMapPublisher>("parking_map_publisher");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
