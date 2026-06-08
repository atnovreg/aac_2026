#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ipm_bev_node/ipm_bev_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ipm_bev_node::IpmBevNode>("ipm_bev_node"));
  rclcpp::shutdown();
  return 0;
}
