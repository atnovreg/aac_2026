#include "joy_control_node/joy_control.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyControl>("joy_control_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
