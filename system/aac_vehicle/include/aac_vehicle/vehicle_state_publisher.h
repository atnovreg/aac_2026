#ifndef AAC_VEHICLE__VEHICLE_STATE_PUBLISHER_H_
#define AAC_VEHICLE__VEHICLE_STATE_PUBLISHER_H_

#include <rclcpp/rclcpp.hpp>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <memory>

class VehicleStatePublisher : public rclcpp::Node
{
public:
  explicit VehicleStatePublisher(const std::string & node_name);
  ~VehicleStatePublisher();

private:
  void getParams();
  void timerCallback();

  void ackermannCallback(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  visualization_msgs::msg::Marker makeChassisMarker() const;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Publisher — Ackermann command for Gazebo AckermannSteering plugin
  // Twist: linear.x = speed [m/s],  angular.z = steer_angle [rad]  (각속도 아닌 직접 각도)
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_ackermann_cmd_;

  // Publisher — visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

  // Subscribers
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_ackermann_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  // Parameters
  double wheelbase_;         // [m]
  double track_width_;       // [m]
  double wheel_radius_;      // [m]
  double max_steering_angle_;// [rad]
  double max_speed_;         // [m/s]

  // Latest received commands & state
  ackermann_msgs::msg::AckermannDriveStamped ackermann_cmd_;
  nav_msgs::msg::Odometry odom_;

  double dt_{0.0};
};

#endif  // AAC_VEHICLE__VEHICLE_STATE_PUBLISHER_H_
