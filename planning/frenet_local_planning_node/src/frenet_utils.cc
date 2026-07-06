#include "frenet_utils.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace frenet_utils
{

void GetParallelRefPath(const std::vector<double> &map_track_x_in,
                        const std::vector<double> &map_track_y_in,
                        double offset,
                        std::vector<double> &perp_x_out,
                        std::vector<double> &perp_y_out)
{
  perp_x_out.clear();
  perp_y_out.clear();

  if (map_track_x_in.size() < 2 || map_track_x_in.size() != map_track_y_in.size()) {
    return;
  }

  for (std::size_t i = 0; i + 1 < map_track_x_in.size(); ++i) {
    const double x_diff = map_track_x_in[i + 1] - map_track_x_in[i];
    const double y_diff = map_track_y_in[i + 1] - map_track_y_in[i];
    const double len = std::hypot(x_diff, y_diff);
    if (len <= 1.0e-6) {
      continue;
    }

    const double perp_x_offset = y_diff / len * offset;
    const double perp_y_offset = -x_diff / len * offset;

    perp_x_out.push_back(map_track_x_in[i] + perp_x_offset);
    perp_y_out.push_back(map_track_y_in[i] + perp_y_offset);
  }

  if (!perp_x_out.empty()) {
    const std::size_t last = map_track_x_in.size() - 1;
    const std::size_t prev = map_track_x_in.size() - 2;
    const double x_diff = map_track_x_in[last] - map_track_x_in[prev];
    const double y_diff = map_track_y_in[last] - map_track_y_in[prev];
    const double len = std::hypot(x_diff, y_diff);
    if (len > 1.0e-6) {
      perp_x_out.push_back(map_track_x_in[last] + y_diff / len * offset);
      perp_y_out.push_back(map_track_y_in[last] - x_diff / len * offset);
    }
  }
}

void DrawParallelPathPolygon(const std::vector<double> &x_parallel_in,
                             const std::vector<double> &y_parallel_in,
                             double parallel_in,
                             geometry_msgs::msg::PolygonStamped &ref_path_boundary_out,
                             double parallel_scale)
{
  if (x_parallel_in.size() != y_parallel_in.size()) {
    return;
  }

  if (std::abs(parallel_in - parallel_scale) <= 1.0e-6) {
    for (std::size_t i = 0; i < x_parallel_in.size(); ++i) {
      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>(x_parallel_in[i]);
      point.y = static_cast<float>(y_parallel_in[i]);
      ref_path_boundary_out.polygon.points.push_back(point);
    }
  } else if (std::abs(parallel_in + parallel_scale) <= 1.0e-6) {
    for (std::size_t i = x_parallel_in.size(); i > 0; --i) {
      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>(x_parallel_in[i - 1]);
      point.y = static_cast<float>(y_parallel_in[i - 1]);
      ref_path_boundary_out.polygon.points.push_back(point);
    }
  }
}

void DrawVehicleCollisionPolygon(const std::vector<double> &local_maptrack_x_in,
                                 const std::vector<double> &local_maptrack_y_in,
                                 geometry_msgs::msg::PolygonStamped &collision_boundary_out,
                                 double veh_width)
{
  std::vector<double> perp_x_left;
  std::vector<double> perp_y_left;
  GetParallelRefPath(local_maptrack_x_in, local_maptrack_y_in, -veh_width / 2.0,
                     perp_x_left, perp_y_left);
  for (std::size_t i = 0; i < perp_x_left.size(); ++i) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(perp_x_left[i]);
    point.y = static_cast<float>(perp_y_left[i]);
    collision_boundary_out.polygon.points.push_back(point);
  }

  std::vector<double> perp_x_right;
  std::vector<double> perp_y_right;
  GetParallelRefPath(local_maptrack_x_in, local_maptrack_y_in, veh_width / 2.0,
                     perp_x_right, perp_y_right);
  for (std::size_t i = perp_x_right.size(); i > 0; --i) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(perp_x_right[i - 1]);
    point.y = static_cast<float>(perp_y_right[i - 1]);
    collision_boundary_out.polygon.points.push_back(point);
  }
}

double CalculateProjDist(const geometry_msgs::msg::Point &point_in,
                         const nav_msgs::msg::Path &ref_path_in)
{
  bool first_node = true;
  double prev_x = 0.0;
  double prev_y = 0.0;
  double min_distance = DBL_MAX;

  for (const auto &ref_pose : ref_path_in.poses) {
    if (first_node) {
      prev_x = ref_pose.pose.position.x;
      prev_y = ref_pose.pose.position.y;
      first_node = false;
      continue;
    }

    const double x_diff = ref_pose.pose.position.x - prev_x;
    const double y_diff = ref_pose.pose.position.y - prev_y;
    const double seg_len = std::hypot(x_diff, y_diff);
    if (seg_len <= 1.0e-6) {
      continue;
    }

    const double normal_distance =
        std::abs(y_diff * point_in.x - x_diff * point_in.y +
                 ref_pose.pose.position.x * prev_y - ref_pose.pose.position.y * prev_x) /
        seg_len;

    const double prod1 = (point_in.x - prev_x) * x_diff +
                         (point_in.y - prev_y) * y_diff;
    const double prod2 = (point_in.x - ref_pose.pose.position.x) * (-x_diff) +
                         (point_in.y - ref_pose.pose.position.y) * (-y_diff);

    if (prod1 >= 0.0 && prod2 >= 0.0) {
      min_distance = std::min(min_distance, normal_distance);
    }

    prev_x = ref_pose.pose.position.x;
    prev_y = ref_pose.pose.position.y;
  }

  return min_distance;
}

double CalculateSignedProjDist(const geometry_msgs::msg::Point &point_in,
                               const nav_msgs::msg::Path &ref_path_in)
{
  bool first_node = true;
  double prev_x = 0.0;
  double prev_y = 0.0;
  double min_distance = DBL_MAX;
  double signed_min_distance = DBL_MAX;

  for (const auto &ref_pose : ref_path_in.poses) {
    if (first_node) {
      prev_x = ref_pose.pose.position.x;
      prev_y = ref_pose.pose.position.y;
      first_node = false;
      continue;
    }

    const double x_diff = ref_pose.pose.position.x - prev_x;
    const double y_diff = ref_pose.pose.position.y - prev_y;
    const double seg_len = std::hypot(x_diff, y_diff);
    if (seg_len <= 1.0e-6) {
      continue;
    }

    const double signed_distance =
        (y_diff * point_in.x - x_diff * point_in.y +
         ref_pose.pose.position.x * prev_y - ref_pose.pose.position.y * prev_x) /
        seg_len;

    const double prod1 = (point_in.x - prev_x) * x_diff +
                         (point_in.y - prev_y) * y_diff;
    const double prod2 = (point_in.x - ref_pose.pose.position.x) * (-x_diff) +
                         (point_in.y - ref_pose.pose.position.y) * (-y_diff);

    if (prod1 >= 0.0 && prod2 >= 0.0 && std::abs(signed_distance) < min_distance) {
      min_distance = std::abs(signed_distance);
      signed_min_distance = signed_distance;
    }

    prev_x = ref_pose.pose.position.x;
    prev_y = ref_pose.pose.position.y;
  }

  return signed_min_distance;
}

bool isInside(const geometry_msgs::msg::PolygonStamped &polygon_in,
              const geometry_msgs::msg::Point32 &p)
{
  const auto &points = polygon_in.polygon.points;
  if (points.size() < 3) {
    return false;
  }

  bool inside = false;
  for (std::size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
    const auto &pi = points[i];
    const auto &pj = points[j];
    const bool crosses =
        ((pi.y > p.y) != (pj.y > p.y)) &&
        (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

} // namespace frenet_utils
