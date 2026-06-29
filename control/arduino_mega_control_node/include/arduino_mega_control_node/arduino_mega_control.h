#ifndef ARDUINO_MEGA_CONTROL_NODE__ARDUINO_MEGA_CONTROL_H_
#define ARDUINO_MEGA_CONTROL_NODE__ARDUINO_MEGA_CONTROL_H_

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <string>

class ArduinoMegaControl : public rclcpp::Node
{
public:
  explicit ArduinoMegaControl(const std::string & node_name);
  ~ArduinoMegaControl();

private:
  void getParams();
  void commandCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
  void timerCallback();

  bool openSerial();
  void closeSerial();
  bool writeSerial(const std::string & line);
  bool serialConnected() const;

  std::string makeCommandLine(double speed, double steering_angle) const;
  std::string makeLegacyCharCommand(double speed, double steering_angle) const;
  std::string makePwmCommand(double speed, double steering_angle) const;
  void publishStatus(const std::string & command_line);
  void sendSafeStop();

  static double clamp(double value, double min_value, double max_value);
  static int clampInt(int value, int min_value, int max_value);
  static int scaledSignedPwm(
    double value,
    double deadband,
    double max_abs_value,
    int pwm_min,
    int pwm_max,
    int direction);

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_command_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_connected_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_tx_;
  rclcpp::TimerBase::SharedPtr timer_;

  ackermann_msgs::msg::AckermannDriveStamped latest_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_reconnect_try_;
  bool has_cmd_{false};

  std::string command_topic_;
  std::string serial_port_;
  std::string usb_interface_;
  int baud_rate_{9600};
  double command_rate_hz_{50.0};
  double command_timeout_sec_{0.3};
  double reconnect_period_sec_{1.0};
  bool auto_connect_{true};
  bool append_newline_{true};
  std::string command_protocol_{"pwm_line"};

  double wheelbase_{0.80};
  double track_width_{0.46};
  double max_speed_{3.0};
  double max_steering_angle_{0.524};
  double speed_deadband_{0.05};
  double steering_deadband_{0.05};

  int steering_pwm_min_{0};
  int steering_pwm_max_{255};
  int left_motor_pwm_min_{0};
  int left_motor_pwm_max_{255};
  int right_motor_pwm_min_{0};
  int right_motor_pwm_max_{255};
  int steering_direction_{1};
  int left_motor_direction_{1};
  int right_motor_direction_{1};

  std::string command_prefix_{"<"};
  std::string command_suffix_{">"};
  std::string field_separator_{","};

  int serial_fd_{-1};
};

#endif  // ARDUINO_MEGA_CONTROL_NODE__ARDUINO_MEGA_CONTROL_H_
