#include "arduino_mega_control_node/arduino_mega_control.h"

#include <cerrno>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace
{
speed_t toBaudRate(const int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:
      throw std::invalid_argument("unsupported baud rate");
  }
}
}  // namespace

ArduinoMegaControl::ArduinoMegaControl(const std::string & node_name)
: Node(node_name)
{
  getParams();

  sub_command_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    command_topic_, rclcpp::QoS(10),
    std::bind(&ArduinoMegaControl::commandCallback, this, std::placeholders::_1));

  pub_connected_ = create_publisher<std_msgs::msg::Bool>("~/serial_connected", rclcpp::QoS(10));
  pub_tx_ = create_publisher<std_msgs::msg::String>("~/tx", rclcpp::QoS(10));

  last_cmd_time_ = now();
  last_reconnect_try_ = now();

  const auto period_ms = static_cast<int>(1000.0 / std::max(command_rate_hz_, 1.0));
  timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&ArduinoMegaControl::timerCallback, this));

  if (auto_connect_) {
    openSerial();
  }
}

ArduinoMegaControl::~ArduinoMegaControl()
{
  sendSafeStop();
  closeSerial();
}

void ArduinoMegaControl::getParams()
{
  declare_parameter<std::string>("command_topic", "/ackermann_drive");
  declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  declare_parameter<std::string>("usb_interface", "arduino_mega");
  declare_parameter<int>("baud_rate", 9600);
  declare_parameter<double>("command_rate_hz", 50.0);
  declare_parameter<double>("command_timeout_sec", 0.3);
  declare_parameter<double>("reconnect_period_sec", 1.0);
  declare_parameter<bool>("auto_connect", true);
  declare_parameter<bool>("append_newline", true);
  declare_parameter<std::string>("command_protocol", "pwm_line");

  declare_parameter<double>("wheelbase", 0.80);
  declare_parameter<double>("track_width", 0.46);
  declare_parameter<double>("max_speed", 3.0);
  declare_parameter<double>("max_steering_angle", 0.524);
  declare_parameter<double>("speed_deadband", 0.05);
  declare_parameter<double>("steering_deadband", 0.05);

  declare_parameter<int>("steering_pwm_min", 0);
  declare_parameter<int>("steering_pwm_max", 255);
  declare_parameter<int>("left_motor_pwm_min", 0);
  declare_parameter<int>("left_motor_pwm_max", 255);
  declare_parameter<int>("right_motor_pwm_min", 0);
  declare_parameter<int>("right_motor_pwm_max", 255);
  declare_parameter<int>("steering_direction", 1);
  declare_parameter<int>("left_motor_direction", 1);
  declare_parameter<int>("right_motor_direction", 1);

  declare_parameter<std::string>("command_prefix", "<");
  declare_parameter<std::string>("command_suffix", ">");
  declare_parameter<std::string>("field_separator", ",");

  command_topic_ = get_parameter("command_topic").as_string();
  serial_port_ = get_parameter("serial_port").as_string();
  usb_interface_ = get_parameter("usb_interface").as_string();
  baud_rate_ = static_cast<int>(get_parameter("baud_rate").as_int());
  command_rate_hz_ = get_parameter("command_rate_hz").as_double();
  command_timeout_sec_ = get_parameter("command_timeout_sec").as_double();
  reconnect_period_sec_ = get_parameter("reconnect_period_sec").as_double();
  auto_connect_ = get_parameter("auto_connect").as_bool();
  append_newline_ = get_parameter("append_newline").as_bool();
  command_protocol_ = get_parameter("command_protocol").as_string();

  wheelbase_ = get_parameter("wheelbase").as_double();
  track_width_ = get_parameter("track_width").as_double();
  max_speed_ = get_parameter("max_speed").as_double();
  max_steering_angle_ = get_parameter("max_steering_angle").as_double();
  speed_deadband_ = get_parameter("speed_deadband").as_double();
  steering_deadband_ = get_parameter("steering_deadband").as_double();

  steering_pwm_min_ = static_cast<int>(get_parameter("steering_pwm_min").as_int());
  steering_pwm_max_ = static_cast<int>(get_parameter("steering_pwm_max").as_int());
  left_motor_pwm_min_ = static_cast<int>(get_parameter("left_motor_pwm_min").as_int());
  left_motor_pwm_max_ = static_cast<int>(get_parameter("left_motor_pwm_max").as_int());
  right_motor_pwm_min_ = static_cast<int>(get_parameter("right_motor_pwm_min").as_int());
  right_motor_pwm_max_ = static_cast<int>(get_parameter("right_motor_pwm_max").as_int());
  steering_direction_ = static_cast<int>(get_parameter("steering_direction").as_int());
  left_motor_direction_ = static_cast<int>(get_parameter("left_motor_direction").as_int());
  right_motor_direction_ = static_cast<int>(get_parameter("right_motor_direction").as_int());

  command_prefix_ = get_parameter("command_prefix").as_string();
  command_suffix_ = get_parameter("command_suffix").as_string();
  field_separator_ = get_parameter("field_separator").as_string();

  RCLCPP_INFO(
    get_logger(),
    "Arduino Mega control: topic=%s port=%s usb_interface=%s baud=%d rate=%.1f Hz protocol=%s",
    command_topic_.c_str(), serial_port_.c_str(), usb_interface_.c_str(),
    baud_rate_, command_rate_hz_, command_protocol_.c_str());
}

void ArduinoMegaControl::commandCallback(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  latest_cmd_ = *msg;
  last_cmd_time_ = now();
  has_cmd_ = true;
}

void ArduinoMegaControl::timerCallback()
{
  if (auto_connect_ && !serialConnected()) {
    const auto t = now();
    if ((t - last_reconnect_try_).seconds() >= reconnect_period_sec_) {
      last_reconnect_try_ = t;
      openSerial();
    }
  }

  double speed = 0.0;
  double steering_angle = 0.0;

  if (has_cmd_ && (now() - last_cmd_time_).seconds() <= command_timeout_sec_) {
    speed = latest_cmd_.drive.speed;
    steering_angle = latest_cmd_.drive.steering_angle;
  }

  speed = clamp(speed, -max_speed_, max_speed_);
  steering_angle = clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

  const std::string command_line = makeCommandLine(speed, steering_angle);
  const std::string serial_bytes = append_newline_ ? command_line + "\n" : command_line;
  if (serialConnected() && !writeSerial(serial_bytes)) {
    closeSerial();
  }

  publishStatus(command_line);
}

bool ArduinoMegaControl::openSerial()
{
  if (serialConnected()) {
    return true;
  }

  int fd = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Cannot open serial port %s (%s). Check usb_interface=%s.",
      serial_port_.c_str(), std::strerror(errno), usb_interface_.c_str());
    return false;
  }

  termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
    ::close(fd);
    return false;
  }

  try {
    const speed_t baud = toBaudRate(baud_rate_);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
  } catch (const std::invalid_argument &) {
    RCLCPP_ERROR(get_logger(), "Unsupported baud_rate: %d", baud_rate_);
    ::close(fd);
    return false;
  }

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 5;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
    ::close(fd);
    return false;
  }

  serial_fd_ = fd;
  RCLCPP_INFO(
    get_logger(), "Connected to Arduino Mega on %s (%s, %d baud)",
    serial_port_.c_str(), usb_interface_.c_str(), baud_rate_);
  return true;
}

void ArduinoMegaControl::closeSerial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_WARN(get_logger(), "Serial port closed.");
  }
}

bool ArduinoMegaControl::writeSerial(const std::string & line)
{
  const ssize_t written = ::write(serial_fd_, line.c_str(), line.size());
  if (written < 0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Serial write failed: %s", std::strerror(errno));
    return false;
  }

  if (static_cast<size_t>(written) != line.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Serial write incomplete: %zd/%zu bytes", written, line.size());
    return false;
  }

  return true;
}

bool ArduinoMegaControl::serialConnected() const
{
  return serial_fd_ >= 0;
}

std::string ArduinoMegaControl::makeCommandLine(
  const double speed, const double steering_angle) const
{
  if (command_protocol_ == "pwm_line") {
    return makePwmCommand(speed, steering_angle);
  }

  return makeLegacyCharCommand(speed, steering_angle);
}

std::string ArduinoMegaControl::makeLegacyCharCommand(
  const double speed, const double steering_angle) const
{
  const bool forward = speed > speed_deadband_;
  const bool reverse = speed < -speed_deadband_;
  const bool left = steering_angle > steering_deadband_;
  const bool right = steering_angle < -steering_deadband_;

  if (forward && left) {
    return "G";
  }
  if (forward && right) {
    return "I";
  }
  if (reverse && left) {
    return "H";
  }
  if (reverse && right) {
    return "J";
  }
  if (forward) {
    return "F";
  }
  if (reverse) {
    return "B";
  }
  if (left) {
    return "L";
  }
  if (right) {
    return "R";
  }

  return "S";
}

std::string ArduinoMegaControl::makePwmCommand(
  const double speed, const double steering_angle) const
{
  double left_speed = speed;
  double right_speed = speed;
  if (std::fabs(steering_angle) > 1e-6 && std::fabs(wheelbase_) > 1e-6) {
    const double yaw_rate = speed * std::tan(steering_angle) / wheelbase_;
    left_speed = speed - yaw_rate * track_width_ * 0.5;
    right_speed = speed + yaw_rate * track_width_ * 0.5;
  }

  const int steering_pwm = scaledSignedPwm(
    steering_angle,
    steering_deadband_,
    max_steering_angle_,
    steering_pwm_min_,
    steering_pwm_max_,
    steering_direction_);
  const int left_pwm = scaledSignedPwm(
    left_speed,
    speed_deadband_,
    max_speed_,
    left_motor_pwm_min_,
    left_motor_pwm_max_,
    left_motor_direction_);
  const int right_pwm = scaledSignedPwm(
    right_speed,
    speed_deadband_,
    max_speed_,
    right_motor_pwm_min_,
    right_motor_pwm_max_,
    right_motor_direction_);

  std::ostringstream ss;
  ss << command_prefix_
     << "S:" << steering_pwm << field_separator_
     << "L:" << left_pwm << field_separator_
     << "R:" << right_pwm
     << command_suffix_;
  return ss.str();
}

void ArduinoMegaControl::publishStatus(const std::string & command_line)
{
  std_msgs::msg::Bool connected;
  connected.data = serialConnected();
  pub_connected_->publish(connected);

  std_msgs::msg::String tx;
  tx.data = command_line;
  pub_tx_->publish(tx);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "tx=%s connected=%s", command_line.c_str(), serialConnected() ? "true" : "false");
}

void ArduinoMegaControl::sendSafeStop()
{
  if (serialConnected()) {
    const std::string stop_command = makeCommandLine(0.0, 0.0);
    writeSerial(append_newline_ ? stop_command + "\n" : stop_command);
  }
}

double ArduinoMegaControl::clamp(
  const double value, const double min_value, const double max_value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::min(std::max(value, min_value), max_value);
}

int ArduinoMegaControl::clampInt(
  const int value, const int min_value, const int max_value)
{
  return std::min(std::max(value, min_value), max_value);
}

int ArduinoMegaControl::scaledSignedPwm(
  const double value,
  const double deadband,
  const double max_abs_value,
  const int pwm_min,
  const int pwm_max,
  const int direction)
{
  if (!std::isfinite(value) || max_abs_value <= 0.0 || pwm_max <= 0) {
    return 0;
  }

  const double abs_value = std::fabs(value);
  const double abs_deadband = std::max(0.0, std::fabs(deadband));
  if (abs_value <= abs_deadband) {
    return 0;
  }

  const int pwm_min_clamped = clampInt(std::max(0, pwm_min), 0, pwm_max);
  const int pwm_max_clamped = std::max(0, pwm_max);
  const double usable_range = std::max(max_abs_value - abs_deadband, 1e-6);
  const double normalized = clamp((abs_value - abs_deadband) / usable_range, 0.0, 1.0);
  const int magnitude = static_cast<int>(std::lround(
    pwm_min_clamped + normalized * (pwm_max_clamped - pwm_min_clamped)));
  const int sign = value >= 0.0 ? 1 : -1;
  const int dir = direction >= 0 ? 1 : -1;

  return clampInt(sign * dir * magnitude, -pwm_max_clamped, pwm_max_clamped);
}
