#include "arduino_mega_control_node/arduino_mega_control.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArduinoMegaControl>("arduino_mega_control_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
