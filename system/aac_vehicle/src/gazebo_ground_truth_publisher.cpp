#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

class GazeboGroundTruthPublisher : public rclcpp::Node
{
public:
  GazeboGroundTruthPublisher()
  : Node("gazebo_ground_truth_publisher")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/gazebo/dynamic_pose");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/odom_gt");
    model_name_ = declare_parameter<std::string>("model_name", "aac_vehicle");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    child_frame_id_ = declare_parameter<std::string>(
      "child_frame_id", "base_link_gt");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, rclcpp::SensorDataQoS());
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(
        &GazeboGroundTruthPublisher::poseCallback, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Gazebo ground truth: %s[%s] -> %s (%s -> %s)",
      input_topic_.c_str(), model_name_.c_str(), output_topic_.c_str(),
      frame_id_.c_str(), child_frame_id_.c_str());
  }

private:
  void poseCallback(const tf2_msgs::msg::TFMessage::SharedPtr message)
  {
    const auto transform = std::find_if(
      message->transforms.begin(), message->transforms.end(),
      [this](const geometry_msgs::msg::TransformStamped & candidate) {
        return candidate.child_frame_id == model_name_;
      });
    if (transform == message->transforms.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Model '%s' is missing from Gazebo dynamic poses.",
        model_name_.c_str());
      return;
    }

    const rclcpp::Time stamp = now();
    const double x = transform->transform.translation.x;
    const double y = transform->transform.translation.y;
    const double yaw = yawFromQuaternion(transform->transform.rotation);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = frame_id_;
    odometry.child_frame_id = child_frame_id_;
    odometry.pose.pose.position.x = x;
    odometry.pose.pose.position.y = y;
    odometry.pose.pose.position.z = transform->transform.translation.z;
    odometry.pose.pose.orientation = transform->transform.rotation;

    if (has_previous_pose_) {
      const double dt = (stamp - previous_stamp_).seconds();
      if (dt > 1e-6) {
        const double world_vx = (x - previous_x_) / dt;
        const double world_vy = (y - previous_y_) / dt;
        odometry.twist.twist.linear.x =
          std::cos(yaw) * world_vx + std::sin(yaw) * world_vy;
        odometry.twist.twist.linear.y =
          -std::sin(yaw) * world_vx + std::cos(yaw) * world_vy;
        odometry.twist.twist.angular.z =
          normalizeAngle(yaw - previous_yaw_) / dt;
      }
    }

    publisher_->publish(odometry);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = frame_id_;
      tf_msg.child_frame_id = child_frame_id_;
      tf_msg.transform.translation.x = x;
      tf_msg.transform.translation.y = y;
      tf_msg.transform.translation.z = transform->transform.translation.z;
      tf_msg.transform.rotation = transform->transform.rotation;
      tf_broadcaster_->sendTransform(tf_msg);
    }

    previous_stamp_ = stamp;
    previous_x_ = x;
    previous_y_ = y;
    previous_yaw_ = yaw;
    has_previous_pose_ = true;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string model_name_;
  std::string frame_id_;
  std::string child_frame_id_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool publish_tf_{false};
  bool has_previous_pose_{false};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  double previous_x_{0.0};
  double previous_y_{0.0};
  double previous_yaw_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboGroundTruthPublisher>());
  rclcpp::shutdown();
  return 0;
}
