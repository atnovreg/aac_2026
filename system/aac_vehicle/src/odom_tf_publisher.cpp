#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

class OdomTfPublisher : public rclcpp::Node
{
public:
  OdomTfPublisher()
  : Node("odom_tf_publisher")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom_lidar");
    base_frame_ = declare_parameter<std::string>(
      "base_frame", "base_footprint_lidar");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OdomTfPublisher::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Publishing odometry TF from %s: %s -> %s",
      input_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = message->header.stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = message->pose.pose.position.x;
    transform.transform.translation.y = message->pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = message->pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  std::string input_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfPublisher>());
  rclcpp::shutdown();
  return 0;
}
