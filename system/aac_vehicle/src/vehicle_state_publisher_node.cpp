#include "aac_vehicle/vehicle_state_publisher.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleStatePublisher>("vehicle_state_publisher");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
