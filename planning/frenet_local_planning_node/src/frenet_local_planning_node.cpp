#include "frenet_local_planning_node/frenet_local_planning.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrenetLocalPlanning>("frenet_local_planning_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
