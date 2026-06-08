#include "ipm_bev_node/ipm_bev_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ipm_bev_node
{

namespace
{

int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

bool cameraInfoChanged(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & previous,
  const sensor_msgs::msg::CameraInfo & current)
{
  if (!previous) {
    return true;
  }
  return previous->width != current.width ||
         previous->height != current.height ||
         previous->k != current.k ||
         previous->d != current.d ||
         previous->distortion_model != current.distortion_model;
}

}  // namespace

IpmBevNode::IpmBevNode(const std::string & node_name)
: Node(node_name),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  declareAndLoadParameters();

  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto info_qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();

  const auto front_image_topic =
    this->get_parameter("front_image_topic").as_string();
  const auto rear_image_topic =
    this->get_parameter("rear_image_topic").as_string();
  const auto front_info_topic =
    this->get_parameter("front_camera_info_topic").as_string();
  const auto rear_info_topic =
    this->get_parameter("rear_camera_info_topic").as_string();

  front_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    front_image_topic, sensor_qos,
    std::bind(&IpmBevNode::frontImageCallback, this, std::placeholders::_1));
  rear_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    rear_image_topic, sensor_qos,
    std::bind(&IpmBevNode::rearImageCallback, this, std::placeholders::_1));
  front_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    front_info_topic, info_qos,
    std::bind(&IpmBevNode::frontInfoCallback, this, std::placeholders::_1));
  rear_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    rear_info_topic, info_qos,
    std::bind(&IpmBevNode::rearInfoCallback, this, std::placeholders::_1));

  front_bev_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    this->get_parameter("front_bev_topic").as_string(), sensor_qos);
  rear_bev_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    this->get_parameter("rear_bev_topic").as_string(), sensor_qos);
  fused_bev_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    this->get_parameter("fused_bev_topic").as_string(), sensor_qos);
  valid_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    this->get_parameter("valid_mask_topic").as_string(), sensor_qos);
  source_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    this->get_parameter("source_mask_topic").as_string(), sensor_qos);
  debug_bev_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    this->get_parameter("debug_bev_topic").as_string(), sensor_qos);

  RCLCPP_INFO(
    this->get_logger(),
    "IPM BEV ready: frame=%s size=%dx%d resolution=%.3fm, x=[%.1f, %.1f], y=[%.1f, %.1f]",
    base_frame_.c_str(), bev_cols_, bev_rows_, resolution_,
    x_min_, x_max_, y_min_, y_max_);
  RCLCPP_INFO(
    this->get_logger(),
    "Inputs: front=%s (%s), rear=%s (%s), projection=%s",
    front_image_topic.c_str(), front_camera_.fallback_frame.c_str(),
    rear_image_topic.c_str(), rear_camera_.fallback_frame.c_str(),
    camera_frame_convention_.c_str());
}

void IpmBevNode::declareAndLoadParameters()
{
  this->declare_parameter<std::string>("front_image_topic", "/camera_front/image_raw");
  this->declare_parameter<std::string>("rear_image_topic", "/camera_rear/image_raw");
  this->declare_parameter<std::string>("front_camera_info_topic", "/camera_info");
  this->declare_parameter<std::string>("rear_camera_info_topic", "/camera_info");
  this->declare_parameter<std::string>("front_camera_frame", "camera_front_link");
  this->declare_parameter<std::string>("rear_camera_frame", "camera_rear_link");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("camera_frame_convention", "link_x_forward");
  this->declare_parameter<bool>("prefer_image_frame_id", false);
  this->declare_parameter<bool>("use_camera_info", true);
  this->declare_parameter<bool>("allow_fov_fallback", true);
  this->declare_parameter<double>("horizontal_fov", 1.2217);
  this->declare_parameter<double>("sync_tolerance_sec", 0.15);
  this->declare_parameter<double>("min_camera_depth", 0.05);

  this->declare_parameter<double>("x_min", -10.0);
  this->declare_parameter<double>("x_max", 10.0);
  this->declare_parameter<double>("y_min", -6.0);
  this->declare_parameter<double>("y_max", 6.0);
  this->declare_parameter<double>("resolution", 0.025);
  this->declare_parameter<double>("ground_z", 0.0);
  this->declare_parameter<double>("fusion_split_x", 0.0);

  this->declare_parameter<bool>("mask_vehicle_footprint", true);
  this->declare_parameter<double>("vehicle_x_min", -0.55);
  this->declare_parameter<double>("vehicle_x_max", 0.55);
  this->declare_parameter<double>("vehicle_y_min", -0.30);
  this->declare_parameter<double>("vehicle_y_max", 0.30);

  this->declare_parameter<std::string>("front_bev_topic", "/ipm_bev/front");
  this->declare_parameter<std::string>("rear_bev_topic", "/ipm_bev/rear");
  this->declare_parameter<std::string>("fused_bev_topic", "/ipm_bev/fused");
  this->declare_parameter<std::string>("valid_mask_topic", "/ipm_bev/valid_mask");
  this->declare_parameter<std::string>("source_mask_topic", "/ipm_bev/source_mask");
  this->declare_parameter<std::string>("debug_bev_topic", "/ipm_bev/debug");

  base_frame_ = this->get_parameter("base_frame").as_string();
  camera_frame_convention_ =
    this->get_parameter("camera_frame_convention").as_string();
  prefer_image_frame_id_ =
    this->get_parameter("prefer_image_frame_id").as_bool();
  use_camera_info_ = this->get_parameter("use_camera_info").as_bool();
  allow_fov_fallback_ =
    this->get_parameter("allow_fov_fallback").as_bool();
  horizontal_fov_ = this->get_parameter("horizontal_fov").as_double();
  sync_tolerance_sec_ =
    this->get_parameter("sync_tolerance_sec").as_double();
  min_camera_depth_ =
    this->get_parameter("min_camera_depth").as_double();

  x_min_ = this->get_parameter("x_min").as_double();
  x_max_ = this->get_parameter("x_max").as_double();
  y_min_ = this->get_parameter("y_min").as_double();
  y_max_ = this->get_parameter("y_max").as_double();
  resolution_ = this->get_parameter("resolution").as_double();
  ground_z_ = this->get_parameter("ground_z").as_double();
  fusion_split_x_ = this->get_parameter("fusion_split_x").as_double();

  mask_vehicle_footprint_ =
    this->get_parameter("mask_vehicle_footprint").as_bool();
  vehicle_x_min_ = this->get_parameter("vehicle_x_min").as_double();
  vehicle_x_max_ = this->get_parameter("vehicle_x_max").as_double();
  vehicle_y_min_ = this->get_parameter("vehicle_y_min").as_double();
  vehicle_y_max_ = this->get_parameter("vehicle_y_max").as_double();

  if (resolution_ <= 0.0 || x_max_ <= x_min_ || y_max_ <= y_min_) {
    throw std::invalid_argument("Invalid BEV bounds or resolution.");
  }
  if (camera_frame_convention_ != "link_x_forward" &&
    camera_frame_convention_ != "ros_optical")
  {
    throw std::invalid_argument(
            "camera_frame_convention must be 'link_x_forward' or 'ros_optical'.");
  }

  bev_rows_ = static_cast<int>(std::ceil((x_max_ - x_min_) / resolution_));
  bev_cols_ = static_cast<int>(std::ceil((y_max_ - y_min_) / resolution_));

  front_camera_.name = "front";
  front_camera_.fallback_frame =
    this->get_parameter("front_camera_frame").as_string();
  rear_camera_.name = "rear";
  rear_camera_.fallback_frame =
    this->get_parameter("rear_camera_frame").as_string();
}

void IpmBevNode::frontImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  front_camera_.latest_image = msg;
  maybeProcess();
}

void IpmBevNode::rearImageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  rear_camera_.latest_image = msg;
  maybeProcess();
}

void IpmBevNode::frontInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  if (cameraInfoChanged(front_camera_.latest_info, *msg)) {
    front_camera_.map_ready = false;
  }
  front_camera_.latest_info = msg;
}

void IpmBevNode::rearInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  if (cameraInfoChanged(rear_camera_.latest_info, *msg)) {
    rear_camera_.map_ready = false;
  }
  rear_camera_.latest_info = msg;
}

IpmBevNode::Intrinsics IpmBevNode::getIntrinsics(
  const CameraState & camera,
  const sensor_msgs::msg::Image & image) const
{
  Intrinsics intrinsics;

  if (use_camera_info_ && camera.latest_info &&
    camera.latest_info->k[0] > 0.0 && camera.latest_info->k[4] > 0.0)
  {
    const auto & info = *camera.latest_info;
    intrinsics.width = static_cast<int>(info.width);
    intrinsics.height = static_cast<int>(info.height);
    intrinsics.camera_matrix = cv::Matx33d(
      info.k[0], info.k[1], info.k[2],
      info.k[3], info.k[4], info.k[5],
      info.k[6], info.k[7], info.k[8]);
    intrinsics.distortion = cv::Mat(info.d, true).reshape(1, 1);
  } else {
    if (!allow_fov_fallback_) {
      throw std::runtime_error(
              camera.name + " CameraInfo is unavailable and FOV fallback is disabled.");
    }
    intrinsics.width = static_cast<int>(image.width);
    intrinsics.height = static_cast<int>(image.height);
    const double fx =
      static_cast<double>(intrinsics.width) /
      (2.0 * std::tan(horizontal_fov_ * 0.5));
    const double fy = fx;
    const double cx = (static_cast<double>(intrinsics.width) - 1.0) * 0.5;
    const double cy = (static_cast<double>(intrinsics.height) - 1.0) * 0.5;
    intrinsics.camera_matrix = cv::Matx33d(
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0);
    intrinsics.distortion = cv::Mat::zeros(1, 5, CV_64F);
  }

  const double scale_x =
    static_cast<double>(image.width) / static_cast<double>(intrinsics.width);
  const double scale_y =
    static_cast<double>(image.height) / static_cast<double>(intrinsics.height);
  intrinsics.camera_matrix(0, 0) *= scale_x;
  intrinsics.camera_matrix(0, 2) *= scale_x;
  intrinsics.camera_matrix(1, 1) *= scale_y;
  intrinsics.camera_matrix(1, 2) *= scale_y;
  intrinsics.width = static_cast<int>(image.width);
  intrinsics.height = static_cast<int>(image.height);
  return intrinsics;
}

bool IpmBevNode::buildProjectionMap(
  CameraState & camera,
  const sensor_msgs::msg::Image & image)
{
  std::string camera_frame = camera.fallback_frame;
  if (prefer_image_frame_id_ && !image.header.frame_id.empty()) {
    camera_frame = image.header.frame_id;
  }

  geometry_msgs::msg::TransformStamped transform_msg;
  try {
    transform_msg = tf_buffer_.lookupTransform(
      base_frame_, camera_frame, tf2::TimePointZero,
      std::chrono::milliseconds(300));
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waiting for TF %s <- %s: %s",
      base_frame_.c_str(), camera_frame.c_str(), error.what());
    return false;
  }

  Intrinsics intrinsics;
  try {
    intrinsics = getIntrinsics(camera, image);
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "%s", error.what());
    return false;
  }

  tf2::Transform base_from_camera;
  tf2::fromMsg(transform_msg.transform, base_from_camera);
  const tf2::Transform camera_from_base = base_from_camera.inverse();

  camera.map_x = cv::Mat(bev_rows_, bev_cols_, CV_32FC1, cv::Scalar(-1.0F));
  camera.map_y = cv::Mat(bev_rows_, bev_cols_, CV_32FC1, cv::Scalar(-1.0F));
  camera.valid_mask = cv::Mat::zeros(bev_rows_, bev_cols_, CV_8UC1);

  std::vector<cv::Point3f> optical_points;
  std::vector<int> bev_indices;
  optical_points.reserve(static_cast<size_t>(bev_rows_ * bev_cols_));
  bev_indices.reserve(static_cast<size_t>(bev_rows_ * bev_cols_));

  for (int row = 0; row < bev_rows_; ++row) {
    const double x = xFromRow(row);
    for (int col = 0; col < bev_cols_; ++col) {
      const double y = yFromCol(col);
      const tf2::Vector3 point_base(x, y, ground_z_);
      const tf2::Vector3 point_camera = camera_from_base * point_base;

      cv::Point3f point_optical;
      double depth = 0.0;
      if (camera_frame_convention_ == "link_x_forward") {
        point_optical = cv::Point3f(
          static_cast<float>(-point_camera.y()),
          static_cast<float>(-point_camera.z()),
          static_cast<float>(point_camera.x()));
        depth = point_camera.x();
      } else {
        point_optical = cv::Point3f(
          static_cast<float>(point_camera.x()),
          static_cast<float>(point_camera.y()),
          static_cast<float>(point_camera.z()));
        depth = point_camera.z();
      }

      if (depth > min_camera_depth_) {
        optical_points.push_back(point_optical);
        bev_indices.push_back(row * bev_cols_ + col);
      }
    }
  }

  std::vector<cv::Point2f> image_points;
  const cv::Mat camera_matrix(intrinsics.camera_matrix);
  cv::projectPoints(
    optical_points, cv::Vec3d::all(0.0), cv::Vec3d::all(0.0),
    camera_matrix, intrinsics.distortion, image_points);

  for (size_t index = 0; index < image_points.size(); ++index) {
    const float u = image_points[index].x;
    const float v = image_points[index].y;
    if (u < 0.0F || v < 0.0F ||
      u >= static_cast<float>(intrinsics.width - 1) ||
      v >= static_cast<float>(intrinsics.height - 1))
    {
      continue;
    }

    const int bev_index = bev_indices[index];
    const int row = bev_index / bev_cols_;
    const int col = bev_index % bev_cols_;
    camera.map_x.at<float>(row, col) = u;
    camera.map_y.at<float>(row, col) = v;
    camera.valid_mask.at<uint8_t>(row, col) = 255;
  }

  maskVehicleFootprint(camera);
  camera.map_image_width = static_cast<int>(image.width);
  camera.map_image_height = static_cast<int>(image.height);
  camera.map_frame = camera_frame;
  camera.map_ready = true;

  const int valid_pixels = cv::countNonZero(camera.valid_mask);
  RCLCPP_INFO(
    this->get_logger(),
    "%s IPM map built: frame=%s image=%dx%d valid=%d/%d (%.1f%%)",
    camera.name.c_str(), camera_frame.c_str(),
    camera.map_image_width, camera.map_image_height,
    valid_pixels, bev_rows_ * bev_cols_,
    100.0 * static_cast<double>(valid_pixels) /
    static_cast<double>(bev_rows_ * bev_cols_));
  return true;
}

void IpmBevNode::maskVehicleFootprint(CameraState & camera) const
{
  if (!mask_vehicle_footprint_) {
    return;
  }

  const int row_min = std::max(0, rowFromX(vehicle_x_max_));
  const int row_max = std::min(bev_rows_ - 1, rowFromX(vehicle_x_min_));
  const int col_min = std::max(0, colFromY(vehicle_y_max_));
  const int col_max = std::min(bev_cols_ - 1, colFromY(vehicle_y_min_));
  if (row_max < row_min || col_max < col_min) {
    return;
  }

  const cv::Rect vehicle_rect(
    col_min, row_min, col_max - col_min + 1, row_max - row_min + 1);
  camera.valid_mask(vehicle_rect).setTo(0);
  camera.map_x(vehicle_rect).setTo(-1.0F);
  camera.map_y(vehicle_rect).setTo(-1.0F);
}

cv::Mat IpmBevNode::remapCamera(
  const CameraState & camera,
  const sensor_msgs::msg::Image & image) const
{
  const auto cv_image = cv_bridge::toCvShare(
    std::make_shared<sensor_msgs::msg::Image>(image),
    sensor_msgs::image_encodings::BGR8);
  cv::Mat bev;
  cv::remap(
    cv_image->image, bev, camera.map_x, camera.map_y,
    cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  bev.setTo(cv::Scalar(0, 0, 0), camera.valid_mask == 0);
  return bev;
}

cv::Mat IpmBevNode::fuseBev(
  const cv::Mat & front_bev,
  const cv::Mat & rear_bev,
  const cv::Mat & front_mask,
  const cv::Mat & rear_mask) const
{
  cv::Mat fused = cv::Mat::zeros(bev_rows_, bev_cols_, CV_8UC3);
  for (int row = 0; row < bev_rows_; ++row) {
    const bool prefer_front = xFromRow(row) >= fusion_split_x_;
    for (int col = 0; col < bev_cols_; ++col) {
      const bool front_valid = front_mask.at<uint8_t>(row, col) != 0;
      const bool rear_valid = rear_mask.at<uint8_t>(row, col) != 0;
      if ((prefer_front && front_valid) || (!rear_valid && front_valid)) {
        fused.at<cv::Vec3b>(row, col) = front_bev.at<cv::Vec3b>(row, col);
      } else if (rear_valid) {
        fused.at<cv::Vec3b>(row, col) = rear_bev.at<cv::Vec3b>(row, col);
      }
    }
  }
  return fused;
}

cv::Mat IpmBevNode::makeSourceMask(
  const cv::Mat & front_mask,
  const cv::Mat & rear_mask) const
{
  cv::Mat source = cv::Mat::zeros(bev_rows_, bev_cols_, CV_8UC1);
  for (int row = 0; row < bev_rows_; ++row) {
    const bool prefer_front = xFromRow(row) >= fusion_split_x_;
    for (int col = 0; col < bev_cols_; ++col) {
      const bool front_valid = front_mask.at<uint8_t>(row, col) != 0;
      const bool rear_valid = rear_mask.at<uint8_t>(row, col) != 0;
      if ((prefer_front && front_valid) || (!rear_valid && front_valid)) {
        source.at<uint8_t>(row, col) = 100;
      } else if (rear_valid) {
        source.at<uint8_t>(row, col) = 200;
      }
    }
  }
  return source;
}

cv::Mat IpmBevNode::makeDebugImage(const cv::Mat & fused_bev) const
{
  cv::Mat debug = fused_bev.clone();

  const int x_start = static_cast<int>(std::ceil(x_min_));
  const int x_end = static_cast<int>(std::floor(x_max_));
  for (int x = x_start; x <= x_end; ++x) {
    const int row = rowFromX(static_cast<double>(x));
    if (row >= 0 && row < bev_rows_) {
      const cv::Scalar color = x == 0 ?
        cv::Scalar(0, 255, 255) : cv::Scalar(80, 80, 80);
      cv::line(debug, cv::Point(0, row), cv::Point(bev_cols_ - 1, row), color, 1);
    }
  }

  const int y_start = static_cast<int>(std::ceil(y_min_));
  const int y_end = static_cast<int>(std::floor(y_max_));
  for (int y = y_start; y <= y_end; ++y) {
    const int col = colFromY(static_cast<double>(y));
    if (col >= 0 && col < bev_cols_) {
      const cv::Scalar color = y == 0 ?
        cv::Scalar(0, 255, 255) : cv::Scalar(80, 80, 80);
      cv::line(debug, cv::Point(col, 0), cv::Point(col, bev_rows_ - 1), color, 1);
    }
  }

  const int vehicle_top = rowFromX(vehicle_x_max_);
  const int vehicle_bottom = rowFromX(vehicle_x_min_);
  const int vehicle_left = colFromY(vehicle_y_max_);
  const int vehicle_right = colFromY(vehicle_y_min_);
  cv::rectangle(
    debug,
    cv::Point(vehicle_left, vehicle_top),
    cv::Point(vehicle_right, vehicle_bottom),
    cv::Scalar(0, 0, 255), 2);
  cv::putText(
    debug, "FRONT", cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
    0.7, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
  return debug;
}

void IpmBevNode::publishImage(
  const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher,
  const cv::Mat & image,
  const std::string & encoding,
  const builtin_interfaces::msg::Time & stamp) const
{
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = base_frame_;
  publisher->publish(*cv_bridge::CvImage(header, encoding, image).toImageMsg());
}

void IpmBevNode::maybeProcess()
{
  if (!front_camera_.latest_image || !rear_camera_.latest_image) {
    return;
  }

  const int64_t front_stamp =
    stampToNanoseconds(front_camera_.latest_image->header.stamp);
  const int64_t rear_stamp =
    stampToNanoseconds(rear_camera_.latest_image->header.stamp);
  if (front_stamp == last_front_stamp_ns_ && rear_stamp == last_rear_stamp_ns_) {
    return;
  }
  if (last_front_stamp_ns_ >= 0 &&
    (front_stamp == last_front_stamp_ns_ || rear_stamp == last_rear_stamp_ns_))
  {
    return;
  }

  const double skew_sec =
    std::abs(static_cast<double>(front_stamp - rear_stamp)) * 1e-9;
  if (skew_sec > sync_tolerance_sec_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Front/rear image skew %.3fs exceeds %.3fs; waiting for a closer pair.",
      skew_sec, sync_tolerance_sec_);
    return;
  }

  const auto & front_image = *front_camera_.latest_image;
  const auto & rear_image = *rear_camera_.latest_image;
  if (!front_camera_.map_ready ||
    front_camera_.map_image_width != static_cast<int>(front_image.width) ||
    front_camera_.map_image_height != static_cast<int>(front_image.height))
  {
    if (!buildProjectionMap(front_camera_, front_image)) {
      return;
    }
  }
  if (!rear_camera_.map_ready ||
    rear_camera_.map_image_width != static_cast<int>(rear_image.width) ||
    rear_camera_.map_image_height != static_cast<int>(rear_image.height))
  {
    if (!buildProjectionMap(rear_camera_, rear_image)) {
      return;
    }
  }

  const auto start = std::chrono::steady_clock::now();
  cv::Mat front_bev;
  cv::Mat rear_bev;
  try {
    front_bev = remapCamera(front_camera_, front_image);
    rear_bev = remapCamera(rear_camera_, rear_image);
  } catch (const cv_bridge::Exception & error) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Image conversion failed: %s", error.what());
    return;
  }

  const cv::Mat fused = fuseBev(
    front_bev, rear_bev,
    front_camera_.valid_mask, rear_camera_.valid_mask);
  cv::Mat valid_mask;
  cv::bitwise_or(
    front_camera_.valid_mask, rear_camera_.valid_mask, valid_mask);
  const cv::Mat source_mask = makeSourceMask(
    front_camera_.valid_mask, rear_camera_.valid_mask);
  const cv::Mat debug = makeDebugImage(fused);

  const auto output_stamp =
    front_stamp >= rear_stamp ?
    front_image.header.stamp : rear_image.header.stamp;
  publishImage(
    front_bev_pub_, front_bev,
    sensor_msgs::image_encodings::BGR8, output_stamp);
  publishImage(
    rear_bev_pub_, rear_bev,
    sensor_msgs::image_encodings::BGR8, output_stamp);
  publishImage(
    fused_bev_pub_, fused,
    sensor_msgs::image_encodings::BGR8, output_stamp);
  publishImage(
    valid_mask_pub_, valid_mask,
    sensor_msgs::image_encodings::MONO8, output_stamp);
  publishImage(
    source_mask_pub_, source_mask,
    sensor_msgs::image_encodings::MONO8, output_stamp);
  publishImage(
    debug_bev_pub_, debug,
    sensor_msgs::image_encodings::BGR8, output_stamp);

  last_front_stamp_ns_ = front_stamp;
  last_rear_stamp_ns_ = rear_stamp;
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Published fused BEV: skew=%.1fms processing=%.1fms",
    skew_sec * 1000.0, elapsed_ms);
}

int IpmBevNode::rowFromX(double x) const
{
  return static_cast<int>(std::floor((x_max_ - x) / resolution_));
}

int IpmBevNode::colFromY(double y) const
{
  return static_cast<int>(std::floor((y_max_ - y) / resolution_));
}

double IpmBevNode::xFromRow(int row) const
{
  return x_max_ - (static_cast<double>(row) + 0.5) * resolution_;
}

double IpmBevNode::yFromCol(int col) const
{
  return y_max_ - (static_cast<double>(col) + 0.5) * resolution_;
}

}  // namespace ipm_bev_node
