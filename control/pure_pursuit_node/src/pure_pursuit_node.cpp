#include "pure_pursuit/pure_pursuit.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuit>("pure_pursuit_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
