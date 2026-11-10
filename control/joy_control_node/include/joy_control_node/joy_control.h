#ifndef JOY_CONTROL_NODE__JOY_CONTROL_H_
#define JOY_CONTROL_NODE__JOY_CONTROL_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <cmath>
#include <chrono>

// ----------------------------------------------------------------
// Button / axis mapping (PS3/PS4 layout via joy_node)
//   Button[0] : Cross  (×)  — E-STOP
//   Button[1] : Circle (○)  — enable run mode
//   Button[5] : R1          — hold for manual joystick control
//   Axes[0]   : left  stick horizontal  (steering)
//   Axes[4]   : right stick vertical    (speed)
// ----------------------------------------------------------------

class JoyControl : public rclcpp::Node
{
public:
  explicit JoyControl(const std::string & node_name);
  ~JoyControl();

private:
  void getParams();
  void timerCallback();
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void autoCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
  void publishStop();

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_auto_cmd_;

  // Publisher
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_ackermann_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double maximum_speed_;
  double maximum_steering_angle_;
  bool   reverse_angle_;
  bool   manual_only_;
  double input_timeout_;

  // State
  ackermann_msgs::msg::AckermannDriveStamped ackermann_cmd_;
  ackermann_msgs::msg::AckermannDriveStamped auto_cmd_;
  sensor_msgs::msg::Joy::SharedPtr latest_joy_;
  bool run_mode_{false};
  bool has_auto_cmd_{false};
  bool arm_was_pressed_{true};
  std::chrono::steady_clock::time_point last_joy_received_;
};

#endif  // JOY_CONTROL_NODE__JOY_CONTROL_H_
