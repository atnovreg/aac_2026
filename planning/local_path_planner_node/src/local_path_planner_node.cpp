#include "local_path_planner_node/local_path_planner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalPathPlanner>("local_path_planner_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
