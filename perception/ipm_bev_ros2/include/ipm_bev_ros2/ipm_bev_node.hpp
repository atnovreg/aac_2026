#ifndef IPM_BEV_ROS2__IPM_BEV_NODE_HPP_
#define IPM_BEV_ROS2__IPM_BEV_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ipm_bev_ros2
{

class IpmBevNode : public rclcpp::Node
{
public:
  explicit IpmBevNode(const std::string & node_name);

private:
  struct CameraState
  {
    std::string name;
    std::string fallback_frame;
    sensor_msgs::msg::Image::ConstSharedPtr latest_image;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info;
    cv::Mat map_x;
    cv::Mat map_y;
    cv::Mat valid_mask;
    bool map_ready{false};
    int map_image_width{0};
    int map_image_height{0};
    std::string map_frame;
  };

  struct Intrinsics
  {
    cv::Matx33d camera_matrix{cv::Matx33d::eye()};
    cv::Mat distortion;
    int width{0};
    int height{0};
  };

  void declareAndLoadParameters();
  void frontImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void rearImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void frontInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void rearInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void maybeProcess();

  Intrinsics getIntrinsics(
    const CameraState & camera,
    const sensor_msgs::msg::Image & image) const;
  bool buildProjectionMap(
    CameraState & camera,
    const sensor_msgs::msg::Image & image);
  cv::Mat remapCamera(
    const CameraState & camera,
    const sensor_msgs::msg::Image & image) const;
  cv::Mat fuseBev(
    const cv::Mat & front_bev,
    const cv::Mat & rear_bev,
    const cv::Mat & front_mask,
    const cv::Mat & rear_mask) const;
  cv::Mat makeSourceMask(
    const cv::Mat & front_mask,
    const cv::Mat & rear_mask) const;
  cv::Mat makeDebugImage(const cv::Mat & fused_bev) const;
  void maskVehicleFootprint(CameraState & camera) const;
  void publishImage(
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher,
    const cv::Mat & image,
    const std::string & encoding,
    const builtin_interfaces::msg::Time & stamp) const;

  int rowFromX(double x) const;
  int colFromY(double y) const;
  double xFromRow(int row) const;
  double yFromCol(int col) const;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  CameraState front_camera_;
  CameraState rear_camera_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rear_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr front_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rear_info_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr front_bev_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rear_bev_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fused_bev_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr valid_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr source_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_bev_pub_;

  std::string base_frame_;
  std::string camera_frame_convention_;
  bool prefer_image_frame_id_{false};
  bool use_camera_info_{true};
  bool allow_fov_fallback_{true};
  bool mask_vehicle_footprint_{true};

  double horizontal_fov_{1.2217};
  double sync_tolerance_sec_{0.15};
  double min_camera_depth_{0.05};
  double x_min_{-10.0};
  double x_max_{10.0};
  double y_min_{-6.0};
  double y_max_{6.0};
  double resolution_{0.025};
  double ground_z_{0.0};
  double fusion_split_x_{0.0};
  double vehicle_x_min_{-0.55};
  double vehicle_x_max_{0.55};
  double vehicle_y_min_{-0.30};
  double vehicle_y_max_{0.30};

  int bev_rows_{0};
  int bev_cols_{0};
  int64_t last_front_stamp_ns_{-1};
  int64_t last_rear_stamp_ns_{-1};
};

}  // namespace ipm_bev_ros2

#endif  // IPM_BEV_ROS2__IPM_BEV_NODE_HPP_
