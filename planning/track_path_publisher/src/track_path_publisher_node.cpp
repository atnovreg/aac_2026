#include "track_path_publisher/track_path_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackPathPublisher>("track_path_publisher");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
