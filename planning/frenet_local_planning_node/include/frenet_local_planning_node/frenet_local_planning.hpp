#ifndef FRENET_LOCAL_PLANNING_NODE__FRENET_LOCAL_PLANNING_HPP_
#define FRENET_LOCAL_PLANNING_NODE__FRENET_LOCAL_PLANNING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <aac_msgs/msg/path_with_cost.hpp>
#include <aac_msgs/msg/path_with_cost_array.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "frenet_path.h"
#include "frenet_path_generator.h"

class FrenetLocalPlanning : public rclcpp::Node {
public:
  explicit FrenetLocalPlanning(const std::string &node_name);
  ~FrenetLocalPlanning() override = default;

private:
  using FrenetPathPtr = std::shared_ptr<FrenetPath>;

  void getParams();
  void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void timerCallback();
  void publishCachedMapVisualization();

  bool extractPathXY(const nav_msgs::msg::Path &path,
                     std::vector<double> &points_x,
                     std::vector<double> &points_y) const;
  std::vector<FrenetPathPtr>
  generateForReferencePath(const std::vector<double> &ref_x,
                           const std::vector<double> &ref_y);
  geometry_msgs::msg::PolygonStamped
  makeReferenceBoundary(const std::vector<double> &ref_x,
                        const std::vector<double> &ref_y,
                        const std_msgs::msg::Header &header) const;

  aac_msgs::msg::PathWithCost
  makePathWithCost(const FrenetPath &frenet_path,
                   const std_msgs::msg::Header &header,
                   int index,
                   const std::vector<geometry_msgs::msg::Point> &obstacles) const;
  bool makeObstaclePoints(
      const std::string &target_frame,
      std::vector<geometry_msgs::msg::Point> &obstacles);
  double obstaclePotentialCost(
      const FrenetPath &frenet_path,
      const std::vector<geometry_msgs::msg::Point> &obstacles,
      bool &in_collision,
      double &minimum_clearance) const;
  nav_msgs::msg::Path makeRosPath(const FrenetPath &frenet_path,
                                  const std_msgs::msg::Header &header) const;
  sensor_msgs::msg::PointCloud2
  makeCandidateCloud(const std::vector<FrenetPathPtr> &frenet_paths,
                     const std_msgs::msg::Header &header) const;
  std_msgs::msg::Header makeVisualizationHeader(
      const std_msgs::msg::Header &source_header) const;
  nav_msgs::msg::Path makeVisualizationPath(
      const nav_msgs::msg::Path &source_path,
      const nav_msgs::msg::Odometry &odom,
      const std_msgs::msg::Header &visualization_header) const;
  sensor_msgs::msg::PointCloud2 makeVisualizationCandidateCloud(
      const std::vector<FrenetPathPtr> &frenet_paths,
      const nav_msgs::msg::Odometry &odom,
      const std_msgs::msg::Header &visualization_header) const;
  geometry_msgs::msg::PolygonStamped makeVisualizationBoundary(
      const geometry_msgs::msg::PolygonStamped &source_boundary,
      const nav_msgs::msg::Odometry &odom,
      const std_msgs::msg::Header &visualization_header) const;

  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q);
  static double normalizeAngle(double angle);
  static geometry_msgs::msg::Point transformBodyPointToMap(
      double body_x,
      double body_y,
      double body_z,
      const nav_msgs::msg::Odometry &odom);
  static geometry_msgs::msg::Point32 transformBodyPointToMap(
      const geometry_msgs::msg::Point32 &body_point,
      const nav_msgs::msg::Odometry &odom);
  static double maxAbsCurvature(const FrenetPath &frenet_path);
  static double minDistanceToOrigin(const FrenetPath &frenet_path);
  double pathScore(const aac_msgs::msg::PathWithCost &path_with_cost) const;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<aac_msgs::msg::PathWithCostArray>::SharedPtr motion_primitives_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr candidate_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      reference_boundary_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_path_visualization_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      candidate_cloud_visualization_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      reference_boundary_visualization_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex path_mutex_;
  nav_msgs::msg::Path latest_local_path_;
  bool has_local_path_ = false;
  bool planning_started_logged_ = false;
  mutable std::mutex odom_mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool has_odom_ = false;
  mutable std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan latest_scan_;
  bool has_scan_ = false;
  mutable std::mutex map_visualization_mutex_;
  nav_msgs::msg::Path last_optimal_path_visualization_;
  sensor_msgs::msg::PointCloud2 last_candidate_cloud_visualization_;
  geometry_msgs::msg::PolygonStamped last_reference_boundary_visualization_;
  bool has_last_map_visualization_ = false;

  std::string local_path_topic_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string motion_primitives_topic_;
  std::string optimal_path_topic_;
  std::string candidate_cloud_topic_;
  std::string reference_boundary_topic_;
  std::string optimal_path_visualization_topic_;
  std::string candidate_cloud_visualization_topic_;
  std::string reference_boundary_visualization_topic_;
  std::string visualization_frame_id_;
  std::string fallback_frame_id_;

  bool publish_map_visualization_ = true;
  bool republish_cached_map_visualization_ = false;
  bool require_scan_for_planning_ = true;
  double update_rate_ = 50.0;
  int input_stride_ = 2;
  int min_path_points_ = 4;
  int parallel_path_count_ = 1;
  double parallel_scale_ = 0.0;
  double left_width_margin_ = -0.5;
  double right_width_margin_ = 0.5;
  double width_delta_ = 0.25;
  double min_prediction_time_ = 3.0;
  double max_prediction_time_ = 3.21;
  double prediction_time_step_ = 0.2;
  double planning_speed_mps_ = 1.5;
  double visualization_x_offset_ = 0.0;

  // Obstacle potential parameters. obstacle_psi controls how gradually a
  // candidate becomes expensive as it approaches a LaserScan return.
  double obstacle_psi_ = 0.45;
  double obstacle_influence_distance_ = 1.20;
  double robot_collision_radius_ = 0.34;
  double collision_penalty_ = 1000.0;
  double scan_timeout_ = 0.50;
  double obstacle_max_range_ = 4.0;
  double obstacle_min_x_ = -0.30;
  int scan_stride_ = 2;
  double self_filter_min_x_ = -0.35;
  double self_filter_max_x_ = 0.35;
  double self_filter_half_width_ = 0.28;

  double obstacle_cost_weight_ = 50.0;
  double reference_cost_weight_ = 1.0;
  double curvature_cost_weight_ = 0.10;
  double transient_cost_weight_ = 2.0;
  double origin_cost_weight_ = 0.10;
  mutable double last_selected_lateral_ = 0.0;
  mutable bool has_last_selected_lateral_ = false;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif // FRENET_LOCAL_PLANNING_NODE__FRENET_LOCAL_PLANNING_HPP_
