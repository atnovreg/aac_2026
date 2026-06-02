#include "aac_vehicle/vehicle_state_publisher.h"
#include <chrono>

using namespace std::chrono_literals;

VehicleStatePublisher::VehicleStatePublisher(const std::string & node_name)
: Node(node_name)
{
  getParams();

  const auto period_ms = std::chrono::duration<double, std::milli>(1000.0 / 20.0);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period_ms),
    std::bind(&VehicleStatePublisher::timerCallback, this));

  pub_ackermann_cmd_ = create_publisher<geometry_msgs::msg::Twist>(
    "/ackermann_cmd", rclcpp::QoS(10));

  pub_marker_ = create_publisher<visualization_msgs::msg::Marker>(
    "/vehicle/marker", rclcpp::SensorDataQoS());

  sub_ackermann_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "/ackermann_drive", rclcpp::QoS(10),
    std::bind(&VehicleStatePublisher::ackermannCallback, this, std::placeholders::_1));

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SensorDataQoS(),
    std::bind(&VehicleStatePublisher::odomCallback, this, std::placeholders::_1));
}

VehicleStatePublisher::~VehicleStatePublisher() {}

void VehicleStatePublisher::getParams()
{
  declare_parameter<double>("wheelbase",          0.80);
  declare_parameter<double>("track_width",        0.46);
  declare_parameter<double>("wheel_radius",       0.08);
  declare_parameter<double>("max_steering_angle", 0.524);
  declare_parameter<double>("max_speed",          3.0);

  wheelbase_          = get_parameter("wheelbase").as_double();
  track_width_        = get_parameter("track_width").as_double();
  wheel_radius_       = get_parameter("wheel_radius").as_double();
  max_steering_angle_ = get_parameter("max_steering_angle").as_double();
  max_speed_          = get_parameter("max_speed").as_double();

  RCLCPP_INFO(get_logger(),
    "Vehicle params — wheelbase: %.2f m, max_steer: %.1f deg, max_speed: %.1f m/s",
    wheelbase_, max_steering_angle_ * 180.0 / M_PI, max_speed_);
}

void VehicleStatePublisher::ackermannCallback(
  const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  ackermann_cmd_ = *msg;
  // Clamp steering and speed to physical limits
  ackermann_cmd_.drive.steering_angle = static_cast<float>(
    std::clamp(static_cast<double>(ackermann_cmd_.drive.steering_angle),
               -max_steering_angle_, max_steering_angle_));
  ackermann_cmd_.drive.speed = static_cast<float>(
    std::clamp(static_cast<double>(ackermann_cmd_.drive.speed),
               -max_speed_, max_speed_));
}

void VehicleStatePublisher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_ = *msg;
}

visualization_msgs::msg::Marker VehicleStatePublisher::makeChassisMarker() const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp    = now();
  marker.ns   = "aac_vehicle";
  marker.id   = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Chassis size: 99 × 55 × 25 cm
  marker.scale.x = 0.99;
  marker.scale.y = 0.55;
  marker.scale.z = 0.25;

  // Centered above base_link
  marker.pose.position.z    = 0.125;
  marker.pose.orientation.w = 1.0;

  marker.color.r = 0.15f;
  marker.color.g = 0.35f;
  marker.color.b = 0.75f;
  marker.color.a = 0.8f;

  return marker;
}

void VehicleStatePublisher::timerCallback()
{
  static auto prev_time = std::chrono::high_resolution_clock::now();
  auto cur_time = std::chrono::high_resolution_clock::now();
  dt_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    cur_time - prev_time).count() / 1e9;
  prev_time = cur_time;

  const double speed = ackermann_cmd_.drive.speed;
  const double delta = ackermann_cmd_.drive.steering_angle;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = speed;
  cmd.angular.z = delta;
  pub_ackermann_cmd_->publish(cmd);

  pub_marker_->publish(makeChassisMarker());

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "speed: %+.2f m/s | steer: %+.1f deg | x: %.2f y: %.2f",
    speed, delta * 180.0 / M_PI,
    odom_.pose.pose.position.x, odom_.pose.pose.position.y);
}
