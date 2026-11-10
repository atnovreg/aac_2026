#include "joy_control_node/joy_control.h"
#include <chrono>
#include <algorithm>
#include <stdexcept>

using namespace std::chrono_literals;

JoyControl::JoyControl(const std::string & node_name)
: Node(node_name)
{
  getParams();

  pub_ackermann_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    "/ackermann_drive", rclcpp::QoS(10));

  sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", rclcpp::QoS(10),
    std::bind(&JoyControl::joyCallback, this, std::placeholders::_1));

  sub_auto_cmd_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "/ackermann_auto", rclcpp::QoS(10),
    std::bind(&JoyControl::autoCallback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    10ms, std::bind(&JoyControl::timerCallback, this));
}

JoyControl::~JoyControl()
{
  RCLCPP_INFO(get_logger(), "JoyControl destroyed.");
}

void JoyControl::getParams()
{
  declare_parameter<double>("maximum_speed",          1.5);
  declare_parameter<double>("maximum_steering_angle", 0.524);
  declare_parameter<bool>  ("reverse_angle",          true);
  declare_parameter<bool>  ("manual_only",            false);
  declare_parameter<double>("input_timeout",         0.3);

  maximum_speed_          = get_parameter("maximum_speed").as_double();
  maximum_steering_angle_ = get_parameter("maximum_steering_angle").as_double();
  reverse_angle_          = get_parameter("reverse_angle").as_bool();
  manual_only_            = get_parameter("manual_only").as_bool();
  input_timeout_          = get_parameter("input_timeout").as_double();
  if (!std::isfinite(input_timeout_) || input_timeout_ <= 0.0) {
    throw std::invalid_argument("input_timeout must be finite and positive");
  }

  RCLCPP_INFO(get_logger(),
    "max_speed: %.2f m/s | max_steer: %.1f deg | reverse: %s",
    maximum_speed_, maximum_steering_angle_ * 180.0 / M_PI,
    reverse_angle_ ? "true" : "false");
}

void JoyControl::autoCallback(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  auto_cmd_     = *msg;
  has_auto_cmd_ = true;
}

void JoyControl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons.size() <= 5 || msg->axes.size() <= 4 ||
      !std::isfinite(msg->axes[0]) || !std::isfinite(msg->axes[4])) {
    latest_joy_.reset();
    run_mode_ = false;
    arm_was_pressed_ = true;
    has_auto_cmd_ = false;
    publishStop();
    return;
  }
  latest_joy_ = msg;
  last_joy_received_ = std::chrono::steady_clock::now();
}

void JoyControl::publishStop()
{
  ackermann_cmd_.header.stamp = now();
  ackermann_cmd_.header.frame_id = "base_link";
  ackermann_cmd_.drive.speed = 0.0f;
  ackermann_cmd_.drive.steering_angle = 0.0f;
  pub_ackermann_->publish(ackermann_cmd_);
}

void JoyControl::timerCallback()
{
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_joy_received_).count();
  if (!latest_joy_ || age > input_timeout_) {
    run_mode_ = false;
    arm_was_pressed_ = true;
    has_auto_cmd_ = false;
    publishStop();
    return;
  }
  if (latest_joy_) {
    const auto & buttons = latest_joy_->buttons;
    const auto & axes    = latest_joy_->axes;

    // Stop wins; a held arm button must be released before rearming.
    if (buttons.at(0) != 0) {
      run_mode_ = false;
      arm_was_pressed_ = true;
      has_auto_cmd_ = false;
      publishStop();
      return;
    }
    const bool arm_pressed = buttons.at(1) != 0;
    if (arm_pressed && !arm_was_pressed_) {
      run_mode_ = true;
      RCLCPP_INFO(get_logger(), "RUN MODE enabled (○)");
    }
    arm_was_pressed_ = arm_pressed;

    ackermann_cmd_.header.stamp    = now();
    ackermann_cmd_.header.frame_id = "base_link";

    if (run_mode_) {
      const double reverse = reverse_angle_ ? -1.0 : 1.0;

      if (buttons.at(5) != 0) {
        // R1 홀드 → 조이스틱 수동 제어
        ackermann_cmd_.drive.speed =
          static_cast<float>(maximum_speed_ *
            std::max(-1.0, std::min(1.0, static_cast<double>(axes.at(4)))));
        ackermann_cmd_.drive.steering_angle =
          static_cast<float>(reverse * maximum_steering_angle_ *
            std::max(-1.0, std::min(1.0, static_cast<double>(axes.at(0)))));
      } else if (!manual_only_ && has_auto_cmd_) {
        // TODO: auto 모드 패스스루 (나중에 모드 선택 추가)
        ackermann_cmd_.drive.speed          = auto_cmd_.drive.speed;
        ackermann_cmd_.drive.steering_angle =
          static_cast<float>(reverse) * auto_cmd_.drive.steering_angle;
      } else {
        ackermann_cmd_.drive.speed          = 0.0f;
        ackermann_cmd_.drive.steering_angle = 0.0f;
      }
    } else {
      ackermann_cmd_.drive.speed          = 0.0f;
      ackermann_cmd_.drive.steering_angle = 0.0f;
    }

    pub_ackermann_->publish(ackermann_cmd_);
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100,
    "[%s]  speed: %+.2f m/s  steer: %+.1f deg",
    run_mode_ ? "RUN" : "STOP",
    ackermann_cmd_.drive.speed,
    static_cast<double>(ackermann_cmd_.drive.steering_angle) * 180.0 / M_PI);
}
