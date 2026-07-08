#include "hybrid_a_star_planner_node/hybrid_a_star_planner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hybrid_a_star_planner_node::HybridAStarPlanner>(
    "hybrid_a_star_planner_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
